#include <Arduino.h>
#include <ArduinoBLE.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "hmac.h"
#include "sha256.h"

const char *targetLocalName = "DM-STATION";
const char *sharedKey = "DM--010225";

BLEDevice peripheral;
BLECharacteristic nonceChar;
BLECharacteristic authTokenChar;
BLECharacteristic batteryFullChar;
BLECharacteristic chargerOKChar;
BLECharacteristic jumperRelayChar;
BLECharacteristic robotRelayChar;
BLECharacteristic dockingStatusChar;

byte lastDockingStatus = 0xFF;

char nonce[9];
char tokenHex[17];

bool authenticated = false;
unsigned long lastRSSILog = 0;

byte lastBatteryFull = 0xFF;
byte lastChargerOK = 0xFF;
byte lastJumperRelay = 0xFF;

// RS485 리포트 타이머
unsigned long lastReportTime = 0;

// Light Scan Watchdog (SCANNING 전용)
static unsigned long lastScanEventMs   = 0;
static unsigned long lastScanRestartMs = 0;
static unsigned long lastStateChangeMs = 0;

const unsigned long SCAN_STALL_MS       = 8000;
const unsigned long MIN_RESTART_GAP_MS  = 1500;
const unsigned long POST_EVENT_GRACE_MS = 6000;

// 타깃 광고 마지막 시각
static unsigned long lastTargetAdvMs = 0;
// 타깃 광고 미수신 시 RSSI 관련 상태 리셋
const unsigned long TARGET_ADV_MISS_MS = 3000;

// 스테이션 광고 기준 시각
static unsigned long noStationBaselineMs = 0;
const unsigned long HARD_RESET_NO_ADV_MS = 30UL * 60UL * 1000UL; // 30분

// 인증 실패 보호
static uint8_t       authFailStreak = 0;
static unsigned long lastAuthFailMs  = 0;
const uint8_t        AUTH_FAIL_HARDRESET_THRESHOLD = 3;
const unsigned long  AUTH_FAIL_DECAY_MS            = 120000UL;

// ===================== RSSI 연결/해제 정책(연속 N회) =====================
// 스캔 단계(인증 전): 필터 결과가 이 값 이상을 연속 N회 만족하면 연결 시도
//EOS : -70
//TP : -50
//const long    SCAN_RSSI_GOOD_DBM     = -70;  // -60 이상이면 양호
const long    SCAN_RSSI_GOOD_DBM     = -50;  // -60 이상이면 양호
const uint8_t SCAN_RSSI_GOOD_CONSEC  = 10;   // 연속 10회(광고 프레임 기준)
static uint8_t scanRssiGoodStreak = 0;

// 연결 후: 필터 결과가 이 값 이하를 연속 N회 만족하면 연결 해제
//EOS : -95
//TP : -65
//const long    CONNECTED_RSSI_BAD_DBM    = -95; // 연속 해제
const long    CONNECTED_RSSI_BAD_DBM    = -65; // 연속 해제
const uint8_t CONNECTED_RSSI_BAD_CONSEC = 5;
static uint8_t connRssiBadStreak = 0;

// ===================== RSSI 정규화 + 필터(공통) =====================
// 유효하지 않은 RSSI 표식
static const int16_t RSSI_INVALID = -128;

// HCI 관행 범위
static const int MIN_DBM = -127;
static const int MAX_DBM =  20;   // 실측은 대부분 음수지만 안전 범위

// RSSI 유효성 검사
static inline bool rssiValidRaw(int v) {
  if (v == 127 || v == 0) return false;   // 미측정/초기값 취급
  if (v < MIN_DBM || v > MAX_DBM) return false;
  return true;
}

// dBm 음수로 정규화
static inline int16_t normDbm(int v) {
  if (!rssiValidRaw(v)) return RSSI_INVALID;
  if (v > 0) v = -v;          // 양수로 오면 음수 dBm으로 치환
  if (v < MIN_DBM) v = MIN_DBM;
  if (v > -1)     v = -1;
  return (int16_t)v;
}

// Median(5) + EMA(1/8) + 스파이크가드(±12 dB, 2연속 수용)
struct RssiFilter {
  int16_t win[5];
  uint8_t widx = 0;
  uint8_t wcount = 0;
  int16_t ema = -100;
  bool    emaInit = false;
  uint8_t spikeStreak = 0;
};
static RssiFilter g_rssi;

static const int   EMA_ALPHA_NUM = 1;  // 1/8
static const int   EMA_ALPHA_DEN = 8;
static const int   SPIKE_GUARD_DB = 12;

static inline void rssiFilterReset() { g_rssi = RssiFilter(); }

static inline int16_t median5(const int16_t* arr, uint8_t n) {
  int16_t t[5];
  for (uint8_t i=0;i<n;i++) t[i]=arr[i];
  for (uint8_t i=0;i+1<n;i++){
    for (uint8_t j=i+1;j<n;j++){
      if (t[j] < t[i]) { int16_t k=t[i]; t[i]=t[j]; t[j]=k; }
    }
  }
  if (n==0) return -100;
  if (n&1)  return t[n/2];
  return (int16_t)((t[n/2 - 1] + t[n/2]) / 2);
}

static inline int16_t rssiFilterUpdate(int16_t rawDbm) {
  if (rawDbm == RSSI_INVALID) return g_rssi.emaInit ? g_rssi.ema : -100;

  g_rssi.win[g_rssi.widx] = rawDbm;
  g_rssi.widx = (g_rssi.widx + 1) % 5;
  if (g_rssi.wcount < 5) g_rssi.wcount++;

  int16_t med = median5(g_rssi.win, g_rssi.wcount);

  if (g_rssi.emaInit && abs(med - g_rssi.ema) >= SPIKE_GUARD_DB) {
    g_rssi.spikeStreak++;
    if (g_rssi.spikeStreak < 2) {
      med = g_rssi.ema; // 단발 스파이크 무시
    } else {
      g_rssi.spikeStreak = 0; // 2연속이면 수용
    }
  } else {
    g_rssi.spikeStreak = 0;
  }

  if (!g_rssi.emaInit) {
    g_rssi.ema = med;
    g_rssi.emaInit = true;
  } else {
    g_rssi.ema = (int16_t)(((long)g_rssi.ema * (EMA_ALPHA_DEN - EMA_ALPHA_NUM)
                           + (long)med * EMA_ALPHA_NUM) / EMA_ALPHA_DEN);
  }
  return g_rssi.ema;
}

// 연결 후 RSSI 업데이트 주기(5 Hz)
static unsigned long lastRssiUpdateMs = 0;
static const unsigned long RSSI_UPDATE_MS = 200;

// RSSI 게이트/필터 초기화
static inline void resetRssiGate(const char* why = nullptr) {
  scanRssiGoodStreak = 0;
  lastTargetAdvMs = 0;
  rssiFilterReset();
  if (why) { Serial.print("[RSSI gate reset] "); Serial.println(why); }
}

// ===================== HMAC/RS485/RESET =====================

void generateHMAC_SHA256(const char *key, const char *message, char *outputHex) {
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i) sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  outputHex[16] = '\0';
}

void sendStatus(const char *label, byte value) {
  Serial1.print("ST,0,");
  Serial1.print(label);
  Serial1.print(",");
  Serial1.print(value);
  Serial1.println(",ED");
}

void rs485_reportRelayState(byte relayState) {
  Serial1.print("ST,0,BMS_STATION_BAT_ON,");
  Serial1.print(relayState == 1 ? "1" : "0");
  Serial1.println(",ED");
}

void hardReset(const char* reason) {
  setRelay(false);
  delay(30);
  Serial.print(">>> HARD RESET: ");
  Serial.println(reason ? reason : "(no reason)");
  delay(20);
  #if defined(ESP32)
    ESP.restart();
  #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_NANO_RP2040_CONNECT)
    NVIC_SystemReset();
  #else
    void(*resetFunc)(void) = 0; resetFunc();
  #endif
}

static void onAuthFailure(const char* reason) {
  unsigned long now = millis();
  if (now - lastAuthFailMs > AUTH_FAIL_DECAY_MS) {
    authFailStreak = 0;
  }
  lastAuthFailMs = now;
  authFailStreak++;

  Serial.print("Auth failure: "); Serial.println(reason);
  Serial.print("Auth fail streak: "); Serial.println(authFailStreak);

  if (authFailStreak >= AUTH_FAIL_HARDRESET_THRESHOLD) {
    hardReset("Auth failures threshold exceeded");
  } else {
    ble_reset();
  }
}

// ===================== BLE 진입/리셋 =====================

void ble_init() {
  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
      Serial.println("BLE init OK");
      BLE.scan(true);
      robotState = SCANNING;

      unsigned long now = millis();
      lastScanEventMs   = now;
      lastScanRestartMs = 0;
      lastStateChangeMs = now;

      resetRssiGate("init");
      noStationBaselineMs = now;
      return;
    }
    Serial.println("BLE init failed - retrying...");
    delay(200);
  }
  Serial.println("BLE init failed (final)");
}

void ble_reset() {
  if (peripheral && peripheral.connected()) {
    peripheral.disconnect();
    delay(100);
  }
  BLE.stopScan();
  delay(100);

  Serial.println("BLE resetting...");

  authenticated = false;
  robotState = IDLE;

  peripheral = BLEDevice();
  nonceChar = BLECharacteristic();
  authTokenChar = BLECharacteristic();
  batteryFullChar = BLECharacteristic();
  chargerOKChar = BLECharacteristic();
  jumperRelayChar = BLECharacteristic();
  robotRelayChar = BLECharacteristic();
  dockingStatusChar = BLECharacteristic();

  connRssiBadStreak = 0;
  resetRssiGate("ble_reset");

  delay(100);
  BLE.scan(true);
  robotState = SCANNING;

  unsigned long now = millis();
  lastScanEventMs   = now;
  lastScanRestartMs = 0;
  lastStateChangeMs = now;

  noStationBaselineMs = now;
}

// ===================== 메인 러너 =====================

void ble_run() {
  unsigned long now = millis();
  unsigned long currentMillis = now;

  if (robotState == SCANNING) {
    BLEDevice device = BLE.available();
    if (device) {
      lastScanEventMs = now;

      if (device.hasLocalName() && device.localName() == targetLocalName) {
        lastTargetAdvMs = now;
        noStationBaselineMs = now;

        int16_t rawNorm  = normDbm(device.rssi());
        int16_t rssiFilt = rssiFilterUpdate(rawNorm);

        if (now - lastRSSILog >= 1000) {
          Serial.print("RSSI(raw->filt): ");
          Serial.print(rawNorm);
          Serial.print(" -> ");
          Serial.println(rssiFilt);
          lastRSSILog = now;
        }

        // === 인증 전: 임계 이상 연속 N회 ===
        if (rssiFilt >= SCAN_RSSI_GOOD_DBM) {
          if (scanRssiGoodStreak < 255) scanRssiGoodStreak++;
          if (scanRssiGoodStreak >= SCAN_RSSI_GOOD_CONSEC) {
            scanRssiGoodStreak = 0; // 소비
            BLE.stopScan();
            Serial.println("RSSI OK (filtered, consecutive) -> connecting...");
            robotState = CONNECTING;
            lastStateChangeMs = now;

            if (device.connect()) {
              Serial.println("Connected");
              peripheral = device;

              bool discovered = false;
              for (int i = 0; i < 3 && !discovered; ++i) {
                delay(120);
                discovered = peripheral.discoverAttributes();
              }
              if (!discovered) {
                Serial.println("GATT discover failed");
                ble_reset();
                return;
              }

              Serial.println("GATT discover OK");

              nonceChar         = peripheral.characteristic("2A03");
              authTokenChar     = peripheral.characteristic("2A04");
              batteryFullChar   = peripheral.characteristic("2A01");
              chargerOKChar     = peripheral.characteristic("2A02");
              jumperRelayChar   = peripheral.characteristic("AA05");
              robotRelayChar    = peripheral.characteristic("AA10");
              dockingStatusChar = peripheral.characteristic("AA06");

              if (nonceChar && nonceChar.canRead() && authTokenChar && authTokenChar.canWrite()) {
                bool nonceOk = false;
                for (int i = 0; i < 3 && !nonceOk; ++i) {
                  byte buf[20];
                  int len = nonceChar.readValue(buf, sizeof(buf));
                  if (len > 0 && len < (int)sizeof(nonce)) {
                    memcpy(nonce, buf, len);
                    nonce[len] = '\0';
                    nonceOk = true;
                  } else {
                    delay(60);
                  }
                }
                if (!nonceOk) {
                  onAuthFailure("nonce read failed");
                  return;
                }

                Serial.print("nonce: ");
                Serial.println(nonce);

                generateHMAC_SHA256(sharedKey, nonce, tokenHex);
                Serial.print("token: ");
                Serial.println(tokenHex);

                bool tokenSent = false;
                for (int i = 0; i < 2 && !tokenSent; ++i) {
                  tokenSent = authTokenChar.writeValue((const unsigned char *)tokenHex, 16);
                  if (!tokenSent) delay(60);
                }

                if (tokenSent) {
                  authenticated     = true;
                  robotState        = CONNECTED;
                  connRssiBadStreak = 0;
                  lastStateChangeMs = now;
                  lastRssiUpdateMs  = now; // 연결 직후 RSSI 주기 시작
                } else {
                  onAuthFailure("token write failed");
                  return;
                }
              } else {
                onAuthFailure("auth characteristics invalid");
                return;
              }
            } else {
              Serial.println("Connect failed");
              robotState = SCANNING;
              resetRssiGate("connect() false");
              BLE.scan(true);

              lastScanEventMs   = now;
              lastScanRestartMs = 0;
              lastStateChangeMs = now;
            }
          }
        } else {
          scanRssiGoodStreak = 0;
        }
      }
    }

    // 타깃 광고가 끊기면 스캔 연속 카운트/필터 리셋
    if (scanRssiGoodStreak > 0 && lastTargetAdvMs > 0 && (now - lastTargetAdvMs) > TARGET_ADV_MISS_MS) {
      resetRssiGate("target adv missed");
      Serial.println("Target adv missed -> reset RSSI gate");
    }

    // Light Scan Watchdog
    if ((now - lastStateChangeMs) > POST_EVENT_GRACE_MS) {
      if ((now - lastScanEventMs) > SCAN_STALL_MS &&
          (now - lastScanRestartMs) > MIN_RESTART_GAP_MS) {
        Serial.println("Scan stalled -> restart scan (light)");
        BLE.stopScan();
        delay(60);
        BLE.scan(true);
        lastScanEventMs   = now;
        lastScanRestartMs = now;
      }
    }

  } else if (robotState == CONNECTED) {
    if (!peripheral.connected()) {
      Serial.println("Disconnected -> rescan");
      sendStatus("BMSBLE", 0);
      ble_reset();
      return;
    }

    // 200 ms마다 RSSI 샘플을 필터에 흡수
    if (now - lastRssiUpdateMs >= RSSI_UPDATE_MS) {
      lastRssiUpdateMs = now;
      int16_t rawNorm = normDbm(peripheral.rssi());
      rssiFilterUpdate(rawNorm);
    }

    // 1 s마다 로그와 해제 판정(연속 N회)
    if (now - lastRSSILog >= 1000) {
      lastRSSILog = now;
      int16_t rssiFilt = g_rssi.emaInit ? g_rssi.ema : -100;
      Serial.print("RSSI(filt): ");
      Serial.println(rssiFilt);

      if (rssiFilt <= CONNECTED_RSSI_BAD_DBM) {
        if (connRssiBadStreak < 255) connRssiBadStreak++;
        Serial.print("RSSI weak (streak=");
        Serial.print(connRssiBadStreak);
        Serial.println(")");
        if (connRssiBadStreak >= CONNECTED_RSSI_BAD_CONSEC) {
          Serial.println("RSSI weak N-consec -> disconnect");
          ble_reset();
          return;
        }
      } else {
        if (connRssiBadStreak) Serial.println("RSSI recovered -> streak reset");
        connRssiBadStreak = 0;
      }
    }

    // 5초마다 릴레이 상태/도킹 상태 보고
    if (currentMillis - lastReportTime >= 5000) {
      lastReportTime = currentMillis;

      if (dockingStatusChar && dockingStatusChar.canRead()) {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue)) {
          if (dockingValue != lastDockingStatus) {
            Serial.print("Docking: ");
            Serial.println(dockingValue);
            lastDockingStatus = dockingValue;
            sendStatus("DOCK", dockingValue);
          }
        } else {
          Serial.println("Docking read failed");
        }
      }

      byte relayState = getRelayState() ? 1 : 0;

      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite()) {
        if (!robotRelayChar.writeValue((uint8_t)relayState)) {
          Serial.println("Relay write failed -> reconnect");
          ble_reset();
          return;
        } else {
          rs485_reportRelayState(relayState);
        }
      } else {
        Serial.println("robotRelayChar invalid -> reconnect");
        ble_reset();
        return;
      }
    }
  }

  // 조건부 주기 하드리셋
  if (robotState != CONNECTED) {
    if ((now - noStationBaselineMs) > HARD_RESET_NO_ADV_MS) {
      hardReset("No station adv for long while (not connected)");
      return;
    }
  }
}

// ===================== Getter =====================

bool getBleConnectionState() {
  return (robotState == CONNECTED && peripheral.connected());
}
bool getBatteryFullStatus() {
  return lastBatteryFull;
}
bool getChargerOkStatus() {
  return lastChargerOK;
}
bool getChargerRelayStatus() {
  return lastJumperRelay;
}
bool getDockingStatus() {
  return lastDockingStatus == 1;
}
