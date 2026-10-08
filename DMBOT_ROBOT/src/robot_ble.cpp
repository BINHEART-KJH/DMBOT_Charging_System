#include <Arduino.h>
#include <ArduinoBLE.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "hmac.h"
#include "sha256.h"
#include "config.h"
#include "mbed.h"

mbed::Watchdog &wdt = mbed::Watchdog::get_instance();

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

/* ===== ORIGINAL CODE (이전 기존 코드) =====
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
const long    SCAN_RSSI_GOOD_DBM     = -80;
const uint8_t SCAN_RSSI_GOOD_CONSEC  = 10;
static uint8_t scanRssiGoodStreak = 0;

// 연결 후: 필터 결과가 이 값 이하를 연속 N회 만족하면 연결 해제
const long    CONNECTED_RSSI_BAD_DBM    = -95;
const uint8_t CONNECTED_RSSI_BAD_CONSEC = 5;
static uint8_t connRssiBadStreak = 0;
===== END ORIGINAL CODE (기존 코드 끝) ===== */

// ===== IMPROVED CODE (개선된 코드) =====

// (IMPROVED) volatile + 동기화를 통한 Race Condition 방지
static volatile bool ble_connected_cached = false;
static volatile unsigned long last_ble_sync_time = 0;

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

// 타깃 광고 마지막 시각
static unsigned long lastTargetAdvMs = 0;

// 스테이션 광고 기준 시각
static unsigned long noStationBaselineMs = 0;

// (IMPROVED) 인증 실패 보호 - 더 관대한 정책
static uint8_t       authFailStreak = 0;
static unsigned long lastAuthFailMs  = 0;
// 기존: 3회 실패 시 하드리셋 → 위험
// 개선: 3회 소프트리셋 → 10회 하드리셋

// RSSI 연결/해제 정책(연속 N회)
static uint8_t scanRssiGoodStreak = 0;
static uint8_t connRssiBadStreak = 0;

// ===== END IMPROVED CODE =====

// ===================== RSSI 필터 (원본 유지 - 좋은 구현) =====================

static const int16_t RSSI_INVALID = -128;
static const int MIN_DBM = -127;
static const int MAX_DBM =  20;

static inline bool rssiValidRaw(int v) {
  if (v == 127 || v == 0) return false;
  if (v < MIN_DBM || v > MAX_DBM) return false;
  return true;
}

static inline int16_t normDbm(int v) {
  if (!rssiValidRaw(v)) return RSSI_INVALID;
  if (v > 0) v = -v;
  if (v < MIN_DBM) v = MIN_DBM;
  if (v > -1)     v = -1;
  return (int16_t)v;
}

struct RssiFilter {
  int16_t win[5];
  uint8_t widx = 0;
  uint8_t wcount = 0;
  int16_t ema = -100;
  bool    emaInit = false;
  uint8_t spikeStreak = 0;
};
static RssiFilter g_rssi;

static const int   EMA_ALPHA_NUM = 1;
static const int   EMA_ALPHA_DEN = 8;
static const int   SPIKE_GUARD_DB = 12;

static inline void rssiFilterReset() {
  g_rssi = RssiFilter();
}

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
      med = g_rssi.ema;
    } else {
      g_rssi.spikeStreak = 0;
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

static unsigned long lastRssiUpdateMs = 0;

static inline void resetRssiGate(const char* why = nullptr) {
  scanRssiGoodStreak = 0;
  lastTargetAdvMs = 0;
  rssiFilterReset();
  if (why) {
    LOG_INFO("RSSI gate reset: %s", why);
  }
}

// ===================== HMAC/리셋 유틸 =====================

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

/* ===== ORIGINAL hardReset (기존 코드 - 즉시 리셋) =====
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
===== END ORIGINAL hardReset ===== */

// (IMPROVED) hardReset - 더 안전한 방식
void hardReset(const char* reason) {
  setRelay(false);
  delay(100);
  
  LOG_ERROR(">>> SYSTEM HARD RESET: %s", reason ? reason : "(no reason)");
  Serial.flush();
  delay(100);
  
  // Watchdog을 사용하여 안전하게 리셋
  #ifdef ARDUINO_ARCH_MBED
    try {
      mbed::Watchdog &wd = mbed::Watchdog::get_instance();
      wd.start(500);  // 500ms (충분한 마진)
      while (true) {
        delay(1);  // Watchdog 타이머 카운트 대기
      }
    } catch (...) {
      NVIC_SystemReset();  // Fallback
    }
  #elif defined(ARDUINO_NANO_RP2040_CONNECT)
    NVIC_SystemReset();
  #else
    void(*resetFunc)(void) = 0; resetFunc();
  #endif
}

/* ===== ORIGINAL onAuthFailure (기존 코드 - 3회 즉시 리셋) =====
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
===== END ORIGINAL onAuthFailure ===== */

// (IMPROVED) onAuthFailure - 점진적 에스컬레이션
static void onAuthFailure(const char* reason) {
  unsigned long now = millis();
  
  // 5분 후 카운터 리셋 (기존: 2분)
  if (now - lastAuthFailMs > AUTH_FAIL_DECAY_MS) {
    authFailStreak = 0;
  }
  lastAuthFailMs = now;
  authFailStreak++;

  LOG_WARN("Auth failure: %s (streak=%d/%d)", 
           reason, authFailStreak, AUTH_FAIL_HARD_RESET_THRESHOLD);

  // 3회 실패: 소프트 리셋 (BLE만 리셋)
  if (authFailStreak >= AUTH_FAIL_SOFT_RESET_THRESHOLD && 
      authFailStreak < AUTH_FAIL_HARD_RESET_THRESHOLD) {
    LOG_WARN("Soft reset triggered (auth fail streak=%d)", authFailStreak);
    ble_reset();
  }
  // 10회 이상: 하드 리셋 (최후의 수단)
  else if (authFailStreak >= AUTH_FAIL_HARD_RESET_THRESHOLD) {
    hardReset("Persistent authentication failures");
  } else {
    ble_reset();
  }
}

// ===================== BLE 진입/리셋 =====================

void ble_init() {
  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
      LOG_INFO("BLE initialized successfully");
      BLE.scan(true);
      robotState = SCANNING;

      unsigned long now = millis();
      lastScanEventMs   = now;
      lastScanRestartMs = 0;
      lastStateChangeMs = now;

      resetRssiGate("ble_init");
      noStationBaselineMs = now;
      ble_connected_cached = false;
      last_ble_sync_time = now;
      return;
    }
    LOG_WARN("BLE init failed - retrying... (%d/5)", i+1);
    delay(200);
  }
  LOG_ERROR("BLE init failed (final)");
}

/* ===== ORIGINAL ble_reset (기존 코드 - 메모리 누수 위험) =====
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

  peripheral = BLEDevice();              // ❌ 메모리 누수
  nonceChar = BLECharacteristic();       // ❌ 메모리 누수
  authTokenChar = BLECharacteristic();   // ❌ 메모리 누수
  batteryFullChar = BLECharacteristic(); // ❌ 메모리 누수
  chargerOKChar = BLECharacteristic();   // ❌ 메모리 누수
  jumperRelayChar = BLECharacteristic(); // ❌ 메모리 누수
  robotRelayChar = BLECharacteristic();  // ❌ 메모리 누수
  dockingStatusChar = BLECharacteristic();// ❌ 메모리 누수

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
===== END ORIGINAL ble_reset ===== */

// (IMPROVED) ble_reset - 메모리 누수 방지
void ble_reset() {
  if (peripheral && peripheral.connected()) {
    peripheral.disconnect();
    delay(100);
  }
  BLE.stopScan();
  delay(100);

  LOG_INFO("BLE resetting...");

  authenticated = false;
  robotState = IDLE;
  
  // (IMPROVED) 명시적 정리 대신 BLE.end() → BLE.begin() 사용
  // ArduinoBLE 내부에서 자동으로 객체를 관리하므로
  // 수동 재설정은 메모리 누수 유발
  // peripheral = BLEDevice();  // ❌ 제거
  // ... 다른 특성들도 제거

  connRssiBadStreak = 0;
  resetRssiGate("ble_reset");
  ble_connected_cached = false;

  delay(100);
  BLE.scan(true);
  robotState = SCANNING;

  unsigned long now = millis();
  lastScanEventMs   = now;
  lastScanRestartMs = 0;
  lastStateChangeMs = now;
  last_ble_sync_time = now;

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
          LOG_DEBUG("RSSI(raw->filt): %d -> %d dBm", rawNorm, rssiFilt);
          lastRSSILog = now;
        }

        // === 인증 전: 임계 이상 연속 N회 ===
        if (rssiFilt >= BLE_SCAN_RSSI_THRESHOLD_DBM) {
          if (scanRssiGoodStreak < 255) scanRssiGoodStreak++;
          if (scanRssiGoodStreak >= BLE_SCAN_CONSEC_GOOD_FRAMES) {
            scanRssiGoodStreak = 0;
            BLE.stopScan();
            LOG_INFO("RSSI OK (filtered, consecutive=%d) -> connecting...", BLE_SCAN_CONSEC_GOOD_FRAMES);
            robotState = CONNECTING;
            lastStateChangeMs = now;

            if (device.connect()) {
              LOG_INFO("BLE Connected to station");
              peripheral = device;

              bool discovered = false;
              for (int i = 0; i < 3 && !discovered; ++i) {
                delay(120);
                discovered = peripheral.discoverAttributes();
              }
              if (!discovered) {
                LOG_ERROR("GATT discovery failed");
                ble_reset();
                return;
              }

              LOG_INFO("GATT discovery OK");

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

                LOG_DEBUG("Nonce: %s", nonce);

                generateHMAC_SHA256(sharedKey, nonce, tokenHex);
                LOG_DEBUG("Token: %s", tokenHex);

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
                  lastRssiUpdateMs  = now;
                  last_ble_sync_time = now;
                  ble_connected_cached = true;
                  LOG_INFO("Authentication successful - CONNECTED state");
                } else {
                  onAuthFailure("token write failed");
                  return;
                }
              } else {
                onAuthFailure("auth characteristics invalid");
                return;
              }
            } else {
              LOG_WARN("Connect() failed");
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
      LOG_WARN("Target advertisement missed -> reset RSSI gate");
    }

    // Light Scan Watchdog
    if ((now - lastStateChangeMs) > POST_EVENT_GRACE_MS) {
      if ((now - lastScanEventMs) > SCAN_STALL_MS &&
          (now - lastScanRestartMs) > MIN_RESTART_GAP_MS) {
        LOG_WARN("Scan stalled -> restart scan (light)");
        BLE.stopScan();
        delay(60);
        BLE.scan(true);
        lastScanEventMs   = now;
        lastScanRestartMs = now;
      }
    }

  } else if (robotState == CONNECTED) {
    if (!peripheral.connected()) {
      LOG_WARN("Disconnected -> rescan");
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
      LOG_DEBUG("RSSI(filt): %d dBm", rssiFilt);

      if (rssiFilt <= BLE_CONNECTED_RSSI_BAD_DBM) {
        if (connRssiBadStreak < 255) connRssiBadStreak++;
        LOG_WARN("RSSI weak (streak=%d/%d)", connRssiBadStreak, BLE_CONNECTED_CONSEC_BAD_FRAMES);
        if (connRssiBadStreak >= BLE_CONNECTED_CONSEC_BAD_FRAMES) {
          LOG_ERROR("RSSI weak N-consecutive -> disconnect");
          ble_reset();
          return;
        }
      } else {
        if (connRssiBadStreak) LOG_INFO("RSSI recovered -> streak reset");
        connRssiBadStreak = 0;
      }
    }

    // (IMPROVED) BLE 상태 동기화
    if (now - last_ble_sync_time >= BLE_STATE_SYNC_MS) {
      last_ble_sync_time = now;
      ble_connected_cached = (robotState == CONNECTED && peripheral.connected());
    }

    // 5초마다 릴레이 상태/도킹 상태 보고
    if (currentMillis - lastReportTime >= REPORT_INTERVAL_MS) {
      lastReportTime = currentMillis;

      if (dockingStatusChar && dockingStatusChar.canRead()) {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue)) {
          if (dockingValue != lastDockingStatus) {
            LOG_INFO("Docking status: %d", dockingValue);
            lastDockingStatus = dockingValue;
            sendStatus("DOCK", dockingValue);
          }
        } else {
          LOG_WARN("Docking read failed");
        }
      }

      byte relayState = getRelayState() ? 1 : 0;

      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite()) {
        if (!robotRelayChar.writeValue((uint8_t)relayState)) {
          LOG_ERROR("Relay write failed -> reconnect");
          ble_reset();
          return;
        } else {
          rs485_reportRelayState(relayState);
        }
      } else {
        LOG_ERROR("robotRelayChar invalid -> reconnect");
        ble_reset();
        return;
      }
    }
  }

  // 조건부 주기 하드리셋
  if (robotState != CONNECTED) {
    if ((now - noStationBaselineMs) > HARD_RESET_NO_ADV_MS) {
      hardReset("No station advertisement for 30min (not connected)");
      return;
    }
  }
}

// ===================== Getter =====================

// (IMPROVED) Race Condition 방지를 위해 캐시된 값 사용
bool getBleConnectionState() {
  return ble_connected_cached;
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

// (NEW) BLE 상태 진단
void ble_print_status() {
  LOG_INFO("===== BLE Status =====");
  LOG_INFO("State: %s", robotState == IDLE ? "IDLE" : 
                        robotState == SCANNING ? "SCANNING" : 
                        robotState == CONNECTING ? "CONNECTING" : 
                        robotState == CONNECTED ? "CONNECTED" : "UNKNOWN");
  LOG_INFO("Authenticated: %s", authenticated ? "YES" : "NO");
  LOG_INFO("Connected: %s", ble_connected_cached ? "YES" : "NO");
  LOG_INFO("Auth Fail Streak: %d", authFailStreak);
  LOG_INFO("RSSI EMA: %d dBm", g_rssi.emaInit ? g_rssi.ema : -100);
  LOG_INFO("=======================");
}
