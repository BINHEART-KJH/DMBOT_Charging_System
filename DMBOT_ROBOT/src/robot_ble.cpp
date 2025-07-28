/*#include <Arduino.h>
#include <ArduinoBLE.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "hmac.h"
#include "sha256.h"

// 필요한 상수 정의
const char *targetLocalName = "DM-STATION";
const char *sharedKey = "DM--010225";

// BLEDevice 객체와 Characteristic 정의
BLEDevice peripheral;
BLECharacteristic nonceChar;
BLECharacteristic authTokenChar;
BLECharacteristic batteryFullChar;
BLECharacteristic chargerOKChar;
BLECharacteristic jumperRelayChar;
BLECharacteristic robotRelayChar;  // ✅ Robot 릴레이 상태 전달용

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

// RELAY_PIN 정의 (예시로 D4 핀을 사용)
#define RELAY_PIN 4  // RELAY_PIN을 사용하는 핀 번호로 정의

// Relay 상태 전송을 위한 전역 변수 선언
unsigned long lastReportTime = 0; // 5초마다 전송할 때 사용

// RSSI 관련 함수들
int getAverageRSSI() {
  int count = rssiBufferFilled ? RSSI_BUFFER_SIZE : rssiIndex;
  if (count == 0) return -100;
  long sum = 0;
  for (int i = 0; i < count; i++) {
    sum += rssiBuffer[i];
  }
  return sum / count;
}

void addRSSIValue(int rssi) {
  rssiBuffer[rssiIndex++] = rssi;
  if (rssiIndex >= RSSI_BUFFER_SIZE) {
    rssiIndex = 0;
    rssiBufferFilled = true;
  }
}

// HMAC-SHA256 토큰 생성 함수
void generateHMAC_SHA256(const char *key, const char *message, char *outputHex) {
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i) {
    sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  }
  outputHex[16] = '\0';
}

// sendStatus 함수 정의
void sendStatus(const char* label, byte value) {
  Serial1.print("ST,0,");
  Serial1.print(label);
  Serial1.print(",");
  Serial1.print(value);
  Serial1.println(",ED");
}

// RS485를 통해 Relay 상태 전송
void rs485_reportRelayState(byte relayState) {
    Serial1.print("ST,0,BMS_STATION_BAT_ON,");
    Serial1.print(relayState == 1 ? "1" : "0");
    Serial1.println(",ED");
}

// BLE 초기화 함수
void ble_init() {
  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
      Serial.println("✅ BLE 초기화 완료");
      BLE.scan(true);
      robotState = SCANNING;
      return;
    }
    Serial.println("⚠️ BLE 초기화 실패 - 재시도 중...");
    delay(200);
  }
  Serial.println("❌ BLE 초기화 실패 (최종)");
}

// BLE 연결 및 리셋 함수
void ble_reset() {
  if (peripheral && peripheral.connected()) {
    peripheral.disconnect();
    delay(100);
  }
  BLE.stopScan();
  authenticated = false;
  robotState = IDLE;

  BLE.scan(true);
  robotState = SCANNING;
  rssiOkStart = 0;
  rssiBadStart = 0;
  rssiIndex = 0;
  rssiBufferFilled = false;
}

void ble_run() {
    unsigned long currentMillis = millis();

    if (robotState == SCANNING) {
        BLEDevice device = BLE.available();
        if (device && device.hasLocalName() && device.localName() == targetLocalName) {
            int rssi = device.rssi();
            addRSSIValue(rssi);

            if (millis() - lastRSSILog >= 1000) {
                Serial.print("📶 RSSI(평균): ");
                Serial.println(getAverageRSSI());
                lastRSSILog = millis();
            }

            if (getAverageRSSI() >= -90) {
                if (rssiOkStart == 0) rssiOkStart = millis();
                if (millis() - rssiOkStart >= 10000) {
                    rssiOkStart = 0;
                    BLE.stopScan();
                    Serial.println("📶 RSSI OK → 연결 시도 중...");
                    robotState = CONNECTING;

                    if (device.connect()) {
                        Serial.println("✅ 연결 성공!");
                        peripheral = device;

                        if (peripheral.discoverAttributes()) {
                            Serial.println("🔍 GATT 속성 탐색 완료");

                            nonceChar       = peripheral.characteristic("2A03");
                            authTokenChar   = peripheral.characteristic("2A04");
                            batteryFullChar = peripheral.characteristic("2A01");
                            chargerOKChar   = peripheral.characteristic("2A02");
                            jumperRelayChar = peripheral.characteristic("AA05");
                            robotRelayChar  = peripheral.characteristic("AA10");

                            if (nonceChar && nonceChar.canRead() && authTokenChar && authTokenChar.canWrite()) {
                                byte buf[20];
                                int len = nonceChar.readValue(buf, sizeof(buf));
                                if (len > 0 && len < sizeof(nonce)) {
                                    memcpy(nonce, buf, len);
                                    nonce[len] = '\0';

                                    Serial.print("📩 nonce 수신: ");
                                    Serial.println(nonce);

                                    generateHMAC_SHA256(sharedKey, nonce, tokenHex);
                                    Serial.print("➡️ 토큰 전송: ");
                                    Serial.println(tokenHex);

                                    authTokenChar.writeValue((const unsigned char *)tokenHex, 16);
                                    authenticated = true;
                                    robotState = CONNECTED;
                                    lastStatusRead = millis();
                                    lastRs485Report = millis();
                                    rssiBadStart = 0;
                                }
                            }
                        } else {
                            Serial.println("❌ GATT 탐색 실패");
                            ble_reset();
                        }
                    } else {
                        Serial.println("❌ 연결 실패");
                        robotState = SCANNING;
                        BLE.scan(true);
                    }
                }
            } else {
                rssiOkStart = 0;
            }
        }
    } else if (robotState == CONNECTED) {
        if (!peripheral.connected()) {
            Serial.println("🔌 연결 끊김 → 재스캔");
            sendStatus("BMSBLE", 0);
            lastBLEState = 0;
            ble_reset();
            return;
        }

        // 5초마다 Relay 상태 전송
        if (currentMillis - lastReportTime >= 5000) {
            lastReportTime = currentMillis;

            // Relay 상태를 BLE로 전송
            byte relayState = digitalRead(RELAY_PIN);  // 현재 Relay 상태를 읽어서 전송
            robotRelayChar.writeValue((uint8_t)relayState);  // Relay 상태를 Robot → Station으로 전송

            // Relay 상태 RS485로 전송
            rs485_reportRelayState(relayState);  // Relay 상태 RS485로 전송
        }
    }
}

// Getter functions
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

      if (getAverageRSSI() >= -70)
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

      if (rssi <= -80)
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