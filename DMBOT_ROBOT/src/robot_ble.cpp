/*#include <Arduino.h>
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
unsigned long lastStatusRead = 0;
unsigned long lastRs485Report = 0;
unsigned long lastRSSILog = 0;

byte lastBLEState = 0xFF;
byte lastBatteryFull = 0xFF;
byte lastChargerOK = 0xFF;
byte lastJumperRelay = 0xFF;

unsigned long rssiOkStart = 0;
unsigned long rssiBadStart = 0;

#define RSSI_BUFFER_SIZE 10
int rssiBuffer[RSSI_BUFFER_SIZE];
int rssiIndex = 0;
bool rssiBufferFilled = false;

#define RELAY_PIN 4
unsigned long lastReportTime = 0;

int getAverageRSSI()
{
  int count = rssiBufferFilled ? RSSI_BUFFER_SIZE : rssiIndex;
  if (count == 0)
    return -100;
  long sum = 0;
  for (int i = 0; i < count; i++)
    sum += rssiBuffer[i];
  return sum / count;
}

void addRSSIValue(int rssi)
{
  rssiBuffer[rssiIndex++] = rssi;
  if (rssiIndex >= RSSI_BUFFER_SIZE)
  {
    rssiIndex = 0;
    rssiBufferFilled = true;
  }
}

void generateHMAC_SHA256(const char *key, const char *message, char *outputHex)
{
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i)
    sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  outputHex[16] = '\0';
}

void sendStatus(const char *label, byte value)
{
  Serial1.print("ST,0,");
  Serial1.print(label);
  Serial1.print(",");
  Serial1.print(value);
  Serial1.println(",ED");
}

void rs485_reportRelayState(byte relayState)
{
  Serial1.print("ST,0,BMS_STATION_BAT_ON,");
  Serial1.print(relayState == 1 ? "1" : "0");
  Serial1.println(",ED");
}

void ble_init()
{
  for (int i = 0; i < 5; i++)
  {
    if (BLE.begin())
    {
      Serial.println("BLE 초기화 완료");
      BLE.scan(true);
      robotState = SCANNING;
      return;
    }
    Serial.println("BLE 초기화 실패 - 재시도 중...");
    delay(200);
  }
  Serial.println("BLE 초기화 실패 (최종)");
}

void ble_reset()
{
  if (peripheral && peripheral.connected())
  {
    peripheral.disconnect();
    delay(100);
  }
  BLE.stopScan();
  delay(100);

  Serial.println("BLE 리셋 중...");

  authenticated = false;
  robotState = IDLE;

  peripheral = BLEDevice(); // 안전 초기화
  nonceChar = BLECharacteristic();
  authTokenChar = BLECharacteristic();
  batteryFullChar = BLECharacteristic();
  chargerOKChar = BLECharacteristic();
  jumperRelayChar = BLECharacteristic();
  robotRelayChar = BLECharacteristic();
  dockingStatusChar = BLECharacteristic();

  rssiOkStart = 0;
  rssiBadStart = 0;
  rssiIndex = 0;
  rssiBufferFilled = false;

  delay(100);
  BLE.scan(true);
  robotState = SCANNING;
}

void ble_run()
{
  unsigned long currentMillis = millis();

  if (robotState == SCANNING)
  {
    BLEDevice device = BLE.available();
    if (device && device.hasLocalName() && device.localName() == targetLocalName)
    {
      int rssi = device.rssi();
      addRSSIValue(rssi);

      if (millis() - lastRSSILog >= 1000)
      {
        Serial.print("RSSI(평균): ");
        Serial.println(getAverageRSSI());
        lastRSSILog = millis();
      }

      if (getAverageRSSI() >= -85)
      {
        if (rssiOkStart == 0)
          rssiOkStart = millis();
        if (millis() - rssiOkStart >= 10000)
        {
          rssiOkStart = 0;
          BLE.stopScan();
          Serial.println("RSSI OK → 연결 시도 중...");
          robotState = CONNECTING;

          if (device.connect())
          {
            Serial.println("연결 성공!");
            peripheral = device;

            if (peripheral.discoverAttributes())
            {
              Serial.println("GATT 속성 탐색 완료");

              nonceChar = peripheral.characteristic("2A03");
              authTokenChar = peripheral.characteristic("2A04");
              batteryFullChar = peripheral.characteristic("2A01");
              chargerOKChar = peripheral.characteristic("2A02");
              jumperRelayChar = peripheral.characteristic("AA05");
              robotRelayChar = peripheral.characteristic("AA10");
              dockingStatusChar = peripheral.characteristic("AA06");

              if (nonceChar && nonceChar.canRead() && authTokenChar && authTokenChar.canWrite())
              {
                byte buf[20];
                int len = nonceChar.readValue(buf, sizeof(buf));
                if (len > 0 && len < sizeof(nonce))
                {
                  memcpy(nonce, buf, len);
                  nonce[len] = '\0';

                  Serial.print("nonce 수신: ");
                  Serial.println(nonce);

                  generateHMAC_SHA256(sharedKey, nonce, tokenHex);
                  Serial.print("토큰 전송: ");
                  Serial.println(tokenHex);

                  if (authTokenChar.writeValue((const unsigned char *)tokenHex, 16))
                  {
                    authenticated = true;
                    robotState = CONNECTED;
                    lastStatusRead = millis();
                    lastRs485Report = millis();
                    rssiBadStart = 0;
                  }
                  else
                  {
                    Serial.println("토큰 전송 실패");
                    ble_reset();
                  }
                }
                else
                {
                  Serial.println("nonce 읽기 실패");
                  ble_reset();
                }
              }
              else
              {
                Serial.println("GATT 인증 캐릭터리스틱 유효성 실패");
                ble_reset();
              }
            }
            else
            {
              Serial.println("GATT 탐색 실패");
              ble_reset();
            }
          }
          else
          {
            Serial.println("연결 실패");
            robotState = SCANNING;
            BLE.scan(true);
          }
        }
      }
      else
      {
        rssiOkStart = 0;
      }
    }
  }
  else if (robotState == CONNECTED)
  {
    if (!peripheral.connected())
    {
      Serial.println("연결 끊김 → 재스캔");
      sendStatus("BMSBLE", 0);
      lastBLEState = 0;
      ble_reset();
      return;
    }

    // 연결 후 RSSI 감시
    if (millis() - lastRSSILog >= 1000)
    {
      int rssi = peripheral.rssi();
      Serial.print("연결 후 RSSI: ");
      Serial.println(rssi);
      lastRSSILog = millis();

      if (rssi <= -120)
      {
        Serial.println("RSSI 너무 약함 → 연결 해제");
        ble_reset();
        return;
      }
    }

    // 5초마다 릴레이 상태 전송
    if (currentMillis - lastReportTime >= 5000)
    {
      lastReportTime = currentMillis;

      // Docking 상태 읽기
      if (dockingStatusChar && dockingStatusChar.canRead())
      {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue))
        {
          if (dockingValue != lastDockingStatus)
          {
            Serial.print("Docking 상태 수신: ");
            Serial.println(dockingValue);
            lastDockingStatus = dockingValue;
            sendStatus("DOCK", dockingValue);
          }
        }
        else
        {
          Serial.println("Docking 상태 읽기 실패");
        }
      }
      byte relayState = digitalRead(RELAY_PIN);

      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite())
      {
        if (!robotRelayChar.writeValue((uint8_t)relayState))
        {
          Serial.println("릴레이 상태 전송 실패 → 재연결 시도");
          ble_reset();
          return;
        }
        else
        {
          rs485_reportRelayState(relayState);
        }
      }
      else
      {
        Serial.println("robotRelayChar 유효하지 않음 → 재연결 시도");
        ble_reset();
        return;
      }
    }
  }
}

bool getBleConnectionState()
{
  return (robotState == CONNECTED && peripheral.connected());
}
bool getBatteryFullStatus()
{
  return lastBatteryFull;
}
bool getChargerOkStatus()
{
  return lastChargerOK;
}
bool getChargerRelayStatus()
{
  return lastJumperRelay;
}
bool getDockingStatus()
{
  return lastDockingStatus == 1;
}
  */
/*
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
unsigned long lastStatusRead = 0;
unsigned long lastRs485Report = 0;
unsigned long lastRSSILog = 0;

byte lastBLEState = 0xFF;
byte lastBatteryFull = 0xFF;
byte lastChargerOK = 0xFF;
byte lastJumperRelay = 0xFF;

unsigned long rssiOkStart = 0;
unsigned long rssiBadStart = 0;

#define RSSI_BUFFER_SIZE 10
int rssiBuffer[RSSI_BUFFER_SIZE];
int rssiIndex = 0;
bool rssiBufferFilled = false;

// === RS485 리포트 타이머 ===
unsigned long lastReportTime = 0;

// --- Light Scan Watchdog (SCANNING 전용, no-regret) ---
static unsigned long lastScanEventMs   = 0; // 어떤 광고든 마지막으로 본 시각
static unsigned long lastScanRestartMs = 0; // 최근 스캔 재시작 시각
static unsigned long lastStateChangeMs = 0; // 상태 전환(스캔 시작/연결 시도) 시각

const unsigned long SCAN_STALL_MS       = 8000; // 광고 자체가 8초 동안 0이면 스캔 재시작
const unsigned long MIN_RESTART_GAP_MS  = 1500; // 스캔 재시작 최소 간격
const unsigned long POST_EVENT_GRACE_MS = 6000; // 상태 전환 직후 그레이스(재도킹/재광고 대기)

// 타깃 광고(= DM-STATION) 마지막으로 본 시점
static unsigned long lastTargetAdvMs = 0;
// 타깃 광고가 이 시간 이상 안 보이면 RSSI 집계/홀드 완전 리셋
const unsigned long TARGET_ADV_MISS_MS = 3000;

// [옵션] 아주 가벼운 스캔 워치독: 아무 광고도 8초간 안 잡히면 스캔만 재시작
static unsigned long lastScanEventMs = 0;
const unsigned long SCAN_WD_MS = 8000;


// === 유틸 ===
int getAverageRSSI()
{
  int count = rssiBufferFilled ? RSSI_BUFFER_SIZE : rssiIndex;
  if (count == 0) return -100;
  long sum = 0;
  for (int i = 0; i < count; i++) sum += rssiBuffer[i];
  return sum / count;
}

void addRSSIValue(int rssi)
{
  rssiBuffer[rssiIndex++] = rssi;
  if (rssiIndex >= RSSI_BUFFER_SIZE) {
    rssiIndex = 0;
    rssiBufferFilled = true;
  }
}

void generateHMAC_SHA256(const char *key, const char *message, char *outputHex)
{
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i) sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  outputHex[16] = '\0';
}

void sendStatus(const char *label, byte value)
{
  Serial1.print("ST,0,");
  Serial1.print(label);
  Serial1.print(",");
  Serial1.print(value);
  Serial1.println(",ED");
}

void rs485_reportRelayState(byte relayState)
{
  Serial1.print("ST,0,BMS_STATION_BAT_ON,");
  Serial1.print(relayState == 1 ? "1" : "0");
  Serial1.println(",ED");
}

void ble_init()
{
  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
      Serial.println("BLE 초기화 완료");
      BLE.scan(true);
      robotState = SCANNING;

      // 워치독 타임스탬프 초기화
      lastScanEventMs   = millis();
      lastScanRestartMs = 0;
      lastTargetAdvMs = 0;
      lastStateChangeMs = millis();

      return;
    }
    Serial.println("BLE 초기화 실패 - 재시도 중...");
    delay(200);
  }
  Serial.println("BLE 초기화 실패 (최종)");
}

void ble_reset()
{
  if (peripheral && peripheral.connected()) {
    peripheral.disconnect();
    delay(100);
  }
  BLE.stopScan();
  delay(100);

  Serial.println("BLE 리셋 중...");

  authenticated = false;
  robotState = IDLE;

  peripheral = BLEDevice(); // 안전 초기화
  nonceChar = BLECharacteristic();
  authTokenChar = BLECharacteristic();
  batteryFullChar = BLECharacteristic();
  chargerOKChar = BLECharacteristic();
  jumperRelayChar = BLECharacteristic();
  robotRelayChar = BLECharacteristic();
  dockingStatusChar = BLECharacteristic();

  rssiOkStart = 0;
  rssiBadStart = 0;
  rssiIndex = 0;
  rssiBufferFilled = false;

  delay(100);
  BLE.scan(true);
  robotState = SCANNING;

  // 워치독 타임스탬프 초기화
  lastScanEventMs   = millis();
  lastScanRestartMs = 0;
  lastStateChangeMs = millis();

  lastTargetAdvMs = 0;
  lastScanEventMs = millis();
}

void ble_run()
{
  unsigned long now = millis();
  unsigned long currentMillis = now;

  if (robotState == SCANNING)
  {
    BLEDevice device = BLE.available();
    if (device) {
      // 어떤 광고든 본 것 → 스캔이 살아있음을 표시
      lastScanEventMs = now;

      // 타깃만 처리
      if (device.hasLocalName() && device.localName() == targetLocalName)
      {
        int rssi = device.rssi();
        addRSSIValue(rssi);

        if (now - lastRSSILog >= 1000) {
          Serial.print("RSSI(평균): ");
          Serial.println(getAverageRSSI());
          lastRSSILog = now;
        }

        // RSSI 검증(사용자 기준 유지)
        if (getAverageRSSI() >= -85) {
          if (rssiOkStart == 0) rssiOkStart = now;
          if (now - rssiOkStart >= 10000) {
            rssiOkStart = 0;
            BLE.stopScan();
            Serial.println("RSSI OK → 연결 시도 중...");
            robotState = CONNECTING;

            // 상태 전환 기록(워치독 그레이스)
            lastStateChangeMs = now;

            if (device.connect()) {
              Serial.println("연결 성공!");
              peripheral = device;

              if (peripheral.discoverAttributes()) {
                Serial.println("GATT 속성 탐색 완료");

                nonceChar         = peripheral.characteristic("2A03");
                authTokenChar     = peripheral.characteristic("2A04");
                batteryFullChar   = peripheral.characteristic("2A01");
                chargerOKChar     = peripheral.characteristic("2A02");
                jumperRelayChar   = peripheral.characteristic("AA05");
                robotRelayChar    = peripheral.characteristic("AA10");
                dockingStatusChar = peripheral.characteristic("AA06");

                if (nonceChar && nonceChar.canRead() && authTokenChar && authTokenChar.canWrite()) {
                  byte buf[20];
                  int len = nonceChar.readValue(buf, sizeof(buf));
                  if (len > 0 && len < sizeof(nonce)) {
                    memcpy(nonce, buf, len);
                    nonce[len] = '\0';

                    Serial.print("nonce 수신: ");
                    Serial.println(nonce);

                    generateHMAC_SHA256(sharedKey, nonce, tokenHex);
                    Serial.print("토큰 전송: ");
                    Serial.println(tokenHex);

                    if (authTokenChar.writeValue((const unsigned char *)tokenHex, 16)) {
                      authenticated = true;
                      robotState = CONNECTED;
                      lastStatusRead  = now;
                      lastRs485Report = now;
                      rssiBadStart = 0;

                      // 상태 전환 기록(워치독 그레이스)
                      lastStateChangeMs = now;
                    }
                    else {
                      Serial.println("토큰 전송 실패");
                      ble_reset();
                    }
                  }
                  else {
                    Serial.println("nonce 읽기 실패");
                    ble_reset();
                  }
                }
                else {
                  Serial.println("GATT 인증 캐릭터리스틱 유효성 실패");
                  ble_reset();
                }
              }
              else {
                Serial.println("GATT 탐색 실패");
                ble_reset();
              }
            }
            else {
              Serial.println("연결 실패");
              robotState = SCANNING;
              BLE.scan(true);

              // 스캔 재시작 기준 갱신
              lastScanEventMs   = now;
              lastScanRestartMs = 0;
              lastStateChangeMs = now;
            }
          }
        } else {
          rssiOkStart = 0;
        }
      }
    }

    // ---- Light Scan Watchdog (SCANNING 전용) ----
    // 상태 전환 직후 그레이스 타임 동안은 건들지 않음
    if ((now - lastStateChangeMs) > POST_EVENT_GRACE_MS) {
      // 광고가 너무 오래 '완전 0'이면 스캔만 재시작
      if ((now - lastScanEventMs) > SCAN_STALL_MS &&
          (now - lastScanRestartMs) > MIN_RESTART_GAP_MS) {
        Serial.println("스캔 무응답 → 스캔 재시작(라이트)");
        BLE.stopScan();
        delay(60);
        BLE.scan(true);
        lastScanEventMs   = now;
        lastScanRestartMs = now;
        // lastStateChangeMs는 그대로(상태 변화 아님)
      }
    }
  }
  else if (robotState == CONNECTED)
  {
    if (!peripheral.connected()) {
      Serial.println("연결 끊김 → 재스캔");
      sendStatus("BMSBLE", 0);
      lastBLEState = 0;
      ble_reset();
      return;
    }

    // 연결 후 RSSI 감시
    if (now - lastRSSILog >= 1000) {
      int rssi = peripheral.rssi();
      Serial.print("연결 후 RSSI: ");
      Serial.println(rssi);
      lastRSSILog = now;

      if (rssi <= -120) {  // 연결 약화/에러 → 리셋(보수적 유지)
        Serial.println("RSSI 너무 약함 → 연결 해제");
        ble_reset();
        return;
      }
    }

    // 5초마다 릴레이 상태/도킹 상태 보고(하트비트)
    if (currentMillis - lastReportTime >= 5000) {
      lastReportTime = currentMillis;

      // Docking 상태 읽기 (있으면)
      if (dockingStatusChar && dockingStatusChar.canRead()) {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue)) {
          if (dockingValue != lastDockingStatus) {
            Serial.print("Docking 상태 수신: ");
            Serial.println(dockingValue);
            lastDockingStatus = dockingValue;
            sendStatus("DOCK", dockingValue);  // RS485 보고
          }
        } else {
          Serial.println("Docking 상태 읽기 실패");
        }
      }

      // 현재 릴레이 상태 전송(로봇 → 스테이션 동기)
      byte relayState = getRelayState() ? 1 : 0;  // ★ 핀 직접읽기 대신 상태 함수 사용

      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite()) {
        if (!robotRelayChar.writeValue((uint8_t)relayState)) {
          Serial.println("릴레이 상태 전송 실패 → 재연결 시도");
          ble_reset();
          return;
        } else {
          rs485_reportRelayState(relayState);  // RS485에도 현재 상태 보고
        }
      } else {
        Serial.println("robotRelayChar 유효하지 않음 → 재연결 시도");
        ble_reset();
        return;
      }
    }
  }
}

bool getBleConnectionState()
{
  return (robotState == CONNECTED && peripheral.connected());
}
bool getBatteryFullStatus()
{
  return lastBatteryFull;
}
bool getChargerOkStatus()
{
  return lastChargerOK;
}
bool getChargerRelayStatus()
{
  return lastJumperRelay;
}
bool getDockingStatus()
{
  return lastDockingStatus == 1;
}
*/

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
unsigned long lastStatusRead = 0;
unsigned long lastRs485Report = 0;
unsigned long lastRSSILog = 0;

byte lastBLEState = 0xFF;
byte lastBatteryFull = 0xFF;
byte lastChargerOK = 0xFF;
byte lastJumperRelay = 0xFF;

unsigned long rssiOkStart = 0;
unsigned long rssiBadStart = 0;

#define RSSI_BUFFER_SIZE 10
int rssiBuffer[RSSI_BUFFER_SIZE];
int rssiIndex = 0;
bool rssiBufferFilled = false;

// === RS485 리포트 타이머 ===
unsigned long lastReportTime = 0;

// --- Light Scan Watchdog (SCANNING 전용, no-regret) ---
static unsigned long lastScanEventMs   = 0; // 어떤 광고든 마지막으로 본 시각
static unsigned long lastScanRestartMs = 0; // 최근 스캔 재시작 시각
static unsigned long lastStateChangeMs = 0; // 상태 전환(스캔 시작/연결 시도) 시각

const unsigned long SCAN_STALL_MS       = 8000; // 광고 자체가 8초 동안 0이면 스캔 재시작
const unsigned long MIN_RESTART_GAP_MS  = 1500; // 스캔 재시작 최소 간격
const unsigned long POST_EVENT_GRACE_MS = 6000; // 상태 전환 직후 그레이스(재도킹/재광고 대기)

// 타깃 광고(= DM-STATION) 마지막으로 본 시점
static unsigned long lastTargetAdvMs = 0;
// 타깃 광고가 이 시간 이상 안 보이면 RSSI 집계/홀드 완전 리셋
const unsigned long TARGET_ADV_MISS_MS = 3000;

// "스테이션 광고를 마지막으로 본 기준" (처음엔 now로 세팅)
// 연결되어 있지 않고 이 기준으로 오래 광고가 없으면 하드리셋
static unsigned long noStationBaselineMs = 0;
const unsigned long HARD_RESET_NO_ADV_MS = 30UL * 60UL * 1000UL; // 30분

// === 유틸 ===
int getAverageRSSI() {
  int count = rssiBufferFilled ? RSSI_BUFFER_SIZE : rssiIndex;
  if (count == 0) return -100;
  long sum = 0;
  for (int i = 0; i < count; i++) sum += rssiBuffer[i];
  return sum / count;
}

void addRSSIValue(int rssi) {
  rssiBuffer[rssiIndex++] = rssi;
  if (rssiIndex >= RSSI_BUFFER_SIZE) {
    rssiIndex = 0;
    rssiBufferFilled = true;
  }
}

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

// --- 하드리셋 헬퍼 ---
void hardReset() {
  Serial.println(">>> HARD RESET (no station adv for configured window) <<<");
  delay(20);
  #if defined(ESP32)
    ESP.restart();
  #elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_NANO_RP2040_CONNECT)
    NVIC_SystemReset();
  #else
    void(*resetFunc)(void) = 0; resetFunc();
  #endif
}

void ble_init() {
  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
      Serial.println("BLE 초기화 완료");
      BLE.scan(true);
      robotState = SCANNING;

      // 워치독 타임스탬프 초기화
      unsigned long now = millis();
      lastScanEventMs   = now;
      lastScanRestartMs = 0;
      lastStateChangeMs = now;

      lastTargetAdvMs = 0;
      noStationBaselineMs = now; // 처음엔 현재 시각 기준으로 시작

      return;
    }
    Serial.println("BLE 초기화 실패 - 재시도 중...");
    delay(200);
  }
  Serial.println("BLE 초기화 실패 (최종)");
}

void ble_reset() {
  if (peripheral && peripheral.connected()) {
    peripheral.disconnect();
    delay(100);
  }
  BLE.stopScan();
  delay(100);

  Serial.println("BLE 리셋 중...");

  authenticated = false;
  robotState = IDLE;

  peripheral = BLEDevice(); // 안전 초기화
  nonceChar = BLECharacteristic();
  authTokenChar = BLECharacteristic();
  batteryFullChar = BLECharacteristic();
  chargerOKChar = BLECharacteristic();
  jumperRelayChar = BLECharacteristic();
  robotRelayChar = BLECharacteristic();
  dockingStatusChar = BLECharacteristic();

  rssiOkStart = 0;
  rssiBadStart = 0;
  rssiIndex = 0;
  rssiBufferFilled = false;

  delay(100);
  BLE.scan(true);
  robotState = SCANNING;

  // 워치독 타임스탬프 초기화
  unsigned long now = millis();
  lastScanEventMs   = now;
  lastScanRestartMs = 0;
  lastStateChangeMs = now;

  lastTargetAdvMs = 0;
  noStationBaselineMs = now; // 스캔을 다시 시작했으니 기준 갱신
}

void ble_run() {
  unsigned long now = millis();
  unsigned long currentMillis = now;

  if (robotState == SCANNING) {
    BLEDevice device = BLE.available();
    if (device) {
      // 어떤 광고든 본 것 → 스캔이 살아있음을 표시
      lastScanEventMs = now;

      // 타깃만 처리
      if (device.hasLocalName() && device.localName() == targetLocalName) {
        lastTargetAdvMs = now;        // ★ 타깃 광고 갱신
        noStationBaselineMs = now;    // ★ "마지막으로 스테이션을 본 시각" 갱신

        int rssi = device.rssi();
        addRSSIValue(rssi);

        if (now - lastRSSILog >= 1000) {
          Serial.print("RSSI(평균): ");
          Serial.println(getAverageRSSI());
          lastRSSILog = now;
        }

        // RSSI 검증(사용자 기준 유지: -85, 10s)
        if (getAverageRSSI() >= -85) {
          if (rssiOkStart == 0) rssiOkStart = now;
          if (now - rssiOkStart >= 10000) {
            rssiOkStart = 0;
            BLE.stopScan();
            Serial.println("RSSI OK → 연결 시도 중...");
            robotState = CONNECTING;

            // 상태 전환 기록(워치독 그레이스)
            lastStateChangeMs = now;

            if (device.connect()) {
              Serial.println("연결 성공!");
              peripheral = device;

              // ===== GATT 탐색: 소량 재시도 + 짧은 안정화 지연 =====
              bool discovered = false;
              for (int i = 0; i < 3 && !discovered; ++i) {
                delay(120); // post-connect settle
                discovered = peripheral.discoverAttributes();
              }
              if (!discovered) {
                Serial.println("GATT 탐색 반복 실패");
                ble_reset();
                return;
              }

              Serial.println("GATT 속성 탐색 완료");

              nonceChar         = peripheral.characteristic("2A03");
              authTokenChar     = peripheral.characteristic("2A04");
              batteryFullChar   = peripheral.characteristic("2A01");
              chargerOKChar     = peripheral.characteristic("2A02");
              jumperRelayChar   = peripheral.characteristic("AA05");
              robotRelayChar    = peripheral.characteristic("AA10");
              dockingStatusChar = peripheral.characteristic("AA06");

              if (nonceChar && nonceChar.canRead() && authTokenChar && authTokenChar.canWrite()) {
                // ===== nonce 읽기: 소량 재시도 =====
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
                  Serial.println("nonce 읽기 반복 실패");
                  ble_reset();
                  return;
                }

                Serial.print("nonce 수신: ");
                Serial.println(nonce);

                generateHMAC_SHA256(sharedKey, nonce, tokenHex);
                Serial.print("토큰 전송: ");
                Serial.println(tokenHex);

                // ===== 토큰 쓰기: 2회 재시도 =====
                bool tokenSent = false;
                for (int i = 0; i < 2 && !tokenSent; ++i) {
                  tokenSent = authTokenChar.writeValue((const unsigned char *)tokenHex, 16);
                  if (!tokenSent) delay(60);
                }

                if (tokenSent) {
                  authenticated   = true;
                  robotState      = CONNECTED;
                  lastStatusRead  = now;
                  lastRs485Report = now;
                  rssiBadStart    = 0;

                  // 상태 전환 기록(워치독 그레이스)
                  lastStateChangeMs = now;
                } else {
                  Serial.println("토큰 전송 실패");
                  ble_reset();
                }
              } else {
                Serial.println("GATT 인증 캐릭터리스틱 유효성 실패");
                ble_reset();
              }
            } else {
              Serial.println("연결 실패");
              robotState = SCANNING;
              BLE.scan(true);

              // 스캔 재시작 기준 갱신
              lastScanEventMs   = now;
              lastScanRestartMs = 0;
              lastStateChangeMs = now;
            }
          }
        } else {
          rssiOkStart = 0;
        }
      }
    }

    // ★ 타깃 광고가 일정 시간 사라지면 RSSI 집계/홀드 완전 리셋
    if (rssiOkStart > 0 && lastTargetAdvMs > 0 && (now - lastTargetAdvMs) > TARGET_ADV_MISS_MS) {
      rssiOkStart = 0;
      rssiIndex = 0;
      rssiBufferFilled = false;
      Serial.println("타깃 광고 끊김 → RSSI 집계/타이머 리셋");
    }

    // ---- Light Scan Watchdog (SCANNING 전용) ----
    // 상태 전환 직후 그레이스 타임 동안은 건들지 않음
    if ((now - lastStateChangeMs) > POST_EVENT_GRACE_MS) {
      // 광고가 너무 오래 '완전 0'이면 스캔만 재시작
      if ((now - lastScanEventMs) > SCAN_STALL_MS &&
          (now - lastScanRestartMs) > MIN_RESTART_GAP_MS) {
        Serial.println("스캔 무응답 → 스캔 재시작(라이트)");
        BLE.stopScan();
        delay(60);
        BLE.scan(true);
        lastScanEventMs   = now;
        lastScanRestartMs = now;
        // lastStateChangeMs는 그대로(상태 변화 아님)
      }
    }

  }
  else if (robotState == CONNECTED) {
    if (!peripheral.connected()) {
      Serial.println("연결 끊김 → 재스캔");
      sendStatus("BMSBLE", 0);
      lastBLEState = 0;
      ble_reset();
      return;
    }

    // 연결 후 RSSI 감시
    if (now - lastRSSILog >= 1000) {
      int rssi = peripheral.rssi();
      Serial.print("연결 후 RSSI: ");
      Serial.println(rssi);
      lastRSSILog = now;

      if (rssi <= -120) {  // 연결 약화/에러 → 리셋(보수적 유지)
        Serial.println("RSSI 너무 약함 → 연결 해제");
        ble_reset();
        return;
      }
    }

    // 5초마다 릴레이 상태/도킹 상태 보고(하트비트)
    if (currentMillis - lastReportTime >= 5000) {
      lastReportTime = currentMillis;

      // Docking 상태 읽기 (있으면)
      if (dockingStatusChar && dockingStatusChar.canRead()) {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue)) {
          if (dockingValue != lastDockingStatus) {
            Serial.print("Docking 상태 수신: ");
            Serial.println(dockingValue);
            lastDockingStatus = dockingValue;
            sendStatus("DOCK", dockingValue);  // RS485 보고
          }
        } else {
          Serial.println("Docking 상태 읽기 실패");
        }
      }

      // 현재 릴레이 상태 전송(로봇 → 스테이션 동기)
byte relayState = getRelayState() ? 1 : 0;

      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite()) {
        if (!robotRelayChar.writeValue((uint8_t)relayState)) {
          Serial.println("릴레이 상태 전송 실패 → 재연결 시도");
          ble_reset();
          return;
        } else {
          rs485_reportRelayState(relayState);  // RS485에도 현재 상태 보고
        }
      } else {
        Serial.println("robotRelayChar 유효하지 않음 → 재연결 시도");
        ble_reset();
        return;
      }
    }
  }

  // ---- 조건부 주기 하드리셋 ----
  // 연결되어 있지 않고, 스테이션 광고를 HARD_RESET_NO_ADV_MS 동안 못 봤다면 하드리셋
  if (robotState != CONNECTED) {
    if ((now - noStationBaselineMs) > HARD_RESET_NO_ADV_MS) {
      Serial.println("조건부 하드리셋 트리거: 장시간 스테이션 광고 미감지 & 미연결");
      hardReset();
      return; // 실질적으로 돌아오지 않음
    }
  }
}

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
