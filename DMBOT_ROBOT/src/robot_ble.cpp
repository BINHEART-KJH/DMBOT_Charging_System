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

#define RELAY_PIN 4
unsigned long lastReportTime = 0;

// ====================== 스캔 워치독 추가 ======================
static unsigned long lastScanEventMs = 0;   // 어떤 광고든 마지막으로 본 시각
static unsigned long lastTargetAdvMs = 0;   // DM-STATION 광고를 마지막으로 본 시각
static uint8_t       scanRestartCount = 0;  // 스캔만 재시작한 횟수(누적)

const unsigned long ADV_MISS_RESET_MS = 1500;  // 타깃 광고를 1.5초 못 보면 RSSI 집계/타이머 리셋
const unsigned long SCAN_WATCHDOG_MS  = 8000;  // 광고 자체를 8초 못 보면 스캔 재시작
const uint8_t SCAN_RESTARTS_BEFORE_FULL_RESET = 3; // 3회 반복되면 BLE 전체 리셋
// ============================================================

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

      // 워치독 초기화
      lastScanEventMs = millis();
      lastTargetAdvMs = 0;
      scanRestartCount = 0;
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

  // 워치독 초기화
  lastScanEventMs = millis();
  lastTargetAdvMs = 0;
  scanRestartCount = 0;
}

void ble_run()
{
  unsigned long now = millis();
  unsigned long currentMillis = now;

  if (robotState == SCANNING)
  {
    BLEDevice device = BLE.available();
    if (device)
    {
      // 어떤 광고든 본 것
      lastScanEventMs = now;

      // 타깃만 처리
      if (device.hasLocalName() && device.localName() == targetLocalName)
      {
        lastTargetAdvMs = now;

        int rssi = device.rssi();
        addRSSIValue(rssi);

        if (now - lastRSSILog >= 1000)
        {
          Serial.print("RSSI(평균): ");
          Serial.println(getAverageRSSI());
          lastRSSILog = now;
        }

        if (getAverageRSSI() >= -70)
        {
          if (rssiOkStart == 0)
            rssiOkStart = now;
          if (now - rssiOkStart >= 10000)
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

                nonceChar         = peripheral.characteristic("2A03");
                authTokenChar     = peripheral.characteristic("2A04");
                batteryFullChar   = peripheral.characteristic("2A01");
                chargerOKChar     = peripheral.characteristic("2A02");
                jumperRelayChar   = peripheral.characteristic("AA05");
                robotRelayChar    = peripheral.characteristic("AA10");
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
                      lastStatusRead = now;
                      lastRs485Report = now;
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

              // 스캔 재시작 시각 갱신
              lastScanEventMs = now;
              lastTargetAdvMs = 0;
              scanRestartCount = 0;
            }
          }
        }
        else
        {
          rssiOkStart = 0;
        }
      }
    }

    // ------ 워치독: 타깃 광고 미감지(평균 RSSI 검증 중 끊긴 경우) ------
    if (rssiOkStart > 0 && (now - lastTargetAdvMs) > ADV_MISS_RESET_MS)
    {
      rssiOkStart = 0;
      rssiIndex = 0;
      rssiBufferFilled = false;
      Serial.println("타깃 광고 끊김 → RSSI 집계/타이머 리셋");
    }

    // ------ 워치독: 광고 자체 무응답 → 스캔만 재시작 ------
    if ((now - lastScanEventMs) > SCAN_WATCHDOG_MS)
    {
      Serial.println("스캔 무응답 → 스캔 재시작");
      BLE.stopScan();
      delay(80);
      BLE.scan(true);
      lastScanEventMs = now;
      lastTargetAdvMs = 0;

      scanRestartCount++;
      if (scanRestartCount >= SCAN_RESTARTS_BEFORE_FULL_RESET)
      {
        Serial.println("스캔 무응답 반복 → BLE 전체 리셋");
        ble_reset();
        return;
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
    if (now - lastRSSILog >= 1000)
    {
      int rssi = peripheral.rssi();
      Serial.print("연결 후 RSSI: ");
      Serial.println(rssi);
      lastRSSILog = now;

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

#define RELAY_PIN 4
unsigned long lastReportTime = 0;

// ====================== 스캔 워치독 추가(유지) ======================
static unsigned long lastScanEventMs = 0;   // 어떤 광고든 마지막으로 본 시각
static unsigned long lastTargetAdvMs = 0;   // DM-STATION 광고를 마지막으로 본 시각
static uint8_t       scanRestartCount = 0;  // 스캔만 재시작한 횟수(누적)

const unsigned long ADV_MISS_RESET_MS = 1500;  // 타깃 광고를 1.5초 못 보면 RSSI 집계/타이머 리셋
const unsigned long SCAN_WATCHDOG_MS  = 8000;  // 광고 자체를 8초 못 보면 스캔 재시작
const uint8_t SCAN_RESTARTS_BEFORE_FULL_RESET = 3; // 3회 반복되면 BLE 전체 리셋
// ============================================================

// ====================== ★ 링크 하트비트 보강 ======================
static unsigned long lastLinkAlive = 0;      // 마지막으로 read/write 성공한 시각
static uint8_t       dockReadFailStreak = 0; // 도킹 읽기 연속 실패
const unsigned long  LINK_IDLE_TIMEOUT_MS = 6000; // 하트비트 유휴 타임아웃
const uint8_t        DOCK_FAIL_MAX = 2;           // 도킹 읽기 연속 실패 허용
// ============================================================

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

      // 워치독 초기화
      lastScanEventMs = millis();
      lastTargetAdvMs = 0;
      scanRestartCount = 0;

      // ★ 링크 하트비트 초기화
      lastLinkAlive = millis();
      dockReadFailStreak = 0;
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

  // ★ 하트비트 상태도 리셋
  lastLinkAlive = millis();
  dockReadFailStreak = 0;

  delay(100);
  BLE.scan(true);
  robotState = SCANNING;

  // 워치독 초기화
  lastScanEventMs = millis();
  lastTargetAdvMs = 0;
  scanRestartCount = 0;
}

void ble_run()
{
  // 이벤트 폴링(연결/해제 즉시 반영)
  BLE.poll();

  unsigned long now = millis();
  unsigned long currentMillis = now;

  if (robotState == SCANNING)
  {
    BLEDevice device = BLE.available();
    if (device)
    {
      // 어떤 광고든 본 것
      lastScanEventMs = now;

      // 타깃만 처리
      if (device.hasLocalName() && device.localName() == targetLocalName)
      {
        lastTargetAdvMs = now;

        int rssi = device.rssi();
        addRSSIValue(rssi);

        if (now - lastRSSILog >= 1000)
        {
          Serial.print("RSSI(평균): ");
          Serial.println(getAverageRSSI());
          lastRSSILog = now;
        }

        // RSSI 검증
        if (getAverageRSSI() >= -85)
        {
          if (rssiOkStart == 0)
            rssiOkStart = now;
          if (now - rssiOkStart >= 10000)
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

                nonceChar         = peripheral.characteristic("2A03");
                authTokenChar     = peripheral.characteristic("2A04");
                batteryFullChar   = peripheral.characteristic("2A01");
                chargerOKChar     = peripheral.characteristic("2A02");
                jumperRelayChar   = peripheral.characteristic("AA05");
                robotRelayChar    = peripheral.characteristic("AA10");
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
                      lastStatusRead = now;
                      lastRs485Report = now;
                      rssiBadStart = 0;

                      // ★ 링크 하트비트 시작
                      lastLinkAlive = now;
                      dockReadFailStreak = 0;
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

              // 스캔 재시작 시각 갱신
              lastScanEventMs = now;
              lastTargetAdvMs = 0;
              scanRestartCount = 0;
            }
          }
        }
        else
        {
          rssiOkStart = 0;
        }
      }
    }

    // ------ 워치독: 타깃 광고 미감지(평균 RSSI 검증 중 끊긴 경우) ------
    if (rssiOkStart > 0 && (now - lastTargetAdvMs) > ADV_MISS_RESET_MS)
    {
      rssiOkStart = 0;
      rssiIndex = 0;
      rssiBufferFilled = false;
      Serial.println("타깃 광고 끊김 → RSSI 집계/타이머 리셋");
    }

    // ------ 워치독: 광고 자체 무응답 → 스캔만 재시작 ------
    if ((now - lastScanEventMs) > SCAN_WATCHDOG_MS)
    {
      Serial.println("스캔 무응답 → 스캔 재시작");
      BLE.stopScan();
      delay(80);
      BLE.scan(true);
      lastScanEventMs = now;
      lastTargetAdvMs = 0;

      scanRestartCount++;
      if (scanRestartCount >= SCAN_RESTARTS_BEFORE_FULL_RESET)
      {
        Serial.println("스캔 무응답 반복 → BLE 전체 리셋");
        ble_reset();
        return;
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

    // 연결 후 RSSI 감시(유지)
    if (now - lastRSSILog >= 1000)
    {
      int rssi = peripheral.rssi();
      Serial.print("연결 후 RSSI: ");
      Serial.println(rssi);
      lastRSSILog = now;

      if (rssi <= -120) // ble 연결 끊김 기준 유지
      {
        Serial.println("RSSI 너무 약함 → 연결 해제");
        ble_reset();
        return;
      }
    }

    // 5초마다 도킹 읽기 + 릴레이 상태 전송 (하트비트)
    if (currentMillis - lastReportTime >= 5000)
    {
      lastReportTime = currentMillis;

      // Docking 상태 읽기 (성공 시 하트비트 OK)
      if (dockingStatusChar && dockingStatusChar.canRead())
      {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue))
        {
          // ★ 하트비트 갱신 & 실패 카운터 리셋
          lastLinkAlive = now;
          dockReadFailStreak = 0;

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
          // ★ 연속 실패 누적 및 임계 시 리셋
          dockReadFailStreak++;
          Serial.println("Docking 상태 읽기 실패");
          if (dockReadFailStreak >= DOCK_FAIL_MAX)
          {
            Serial.println("도킹 상태 읽기 연속 실패 → 하트비트 타임아웃 리셋");
            ble_reset();
            return;
          }
        }
      }

      // 릴레이 상태 전송 (성공 시 하트비트 OK)
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
          // ★ 하트비트 갱신
          lastLinkAlive = now;
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

    // ★ 연결 중 유휴 타임아웃(하트비트 미발생) 체크
    if (now - lastLinkAlive > LINK_IDLE_TIMEOUT_MS)
    {
      Serial.println("링크 유휴 타임아웃 → BLE 리셋");
      ble_reset();
      return;
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
