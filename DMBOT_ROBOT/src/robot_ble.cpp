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

// === BLE 연결 후 3초 지연 ON용 타이머 ===
bool relayTimerArmed = false;
unsigned long relayTimerStart = 0;

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
  for (int i = 0; i < 8; ++i)
    sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
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

static inline void setRelay(byte on) {
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

void ble_init() {
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false); // 시작은 무조건 OFF

  for (int i = 0; i < 5; i++) {
    if (BLE.begin()) {
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

void ble_reset() {
  // 연결 끊기 전/후 무조건 릴레이 OFF
  setRelay(false);
  relayTimerArmed = false;
  relayTimerStart = 0;

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
}

void ble_run() {
  unsigned long currentMillis = millis();

  if (robotState == SCANNING) {
    BLEDevice device = BLE.available();
    if (device && device.hasLocalName() && device.localName() == targetLocalName) {
      int rssi = device.rssi();
      addRSSIValue(rssi);

      if (millis() - lastRSSILog >= 1000) {
        Serial.print("RSSI(평균): ");
        Serial.println(getAverageRSSI());
        lastRSSILog = millis();
      }

      if (getAverageRSSI() >= -70) {
        if (rssiOkStart == 0) rssiOkStart = millis();
        if (millis() - rssiOkStart >= 10000) {
          rssiOkStart = 0;
          BLE.stopScan();
          Serial.println("RSSI OK → 연결 시도 중...");
          robotState = CONNECTING;

          if (device.connect()) {
            Serial.println("연결 성공!");
            peripheral = device;

            if (peripheral.discoverAttributes()) {
              Serial.println("GATT 속성 탐색 완료");

              nonceChar        = peripheral.characteristic("2A03");
              authTokenChar    = peripheral.characteristic("2A04");
              batteryFullChar  = peripheral.characteristic("2A01");
              chargerOKChar    = peripheral.characteristic("2A02");
              jumperRelayChar  = peripheral.characteristic("AA05");
              robotRelayChar   = peripheral.characteristic("AA10");
              dockingStatusChar= peripheral.characteristic("AA06");

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
                    lastStatusRead = millis();
                    lastRs485Report = millis();
                    rssiBadStart = 0;

                    // === 연결(인증) 직후: 릴레이 3초 지연 ON 타이머 무장 ===
                    setRelay(false);               // 안전 OFF
                    relayTimerArmed = true;
                    relayTimerStart = millis();
                    Serial.println("CONNECTED: 3초 후 릴레이 ON 예정");
                  } else {
                    Serial.println("토큰 전송 실패");
                    ble_reset();
                  }
                } else {
                  Serial.println("nonce 읽기 실패");
                  ble_reset();
                }
              } else {
                Serial.println("GATT 인증 캐릭터리스틱 유효성 실패");
                ble_reset();
              }
            } else {
              Serial.println("GATT 탐색 실패");
              ble_reset();
            }
          } else {
            Serial.println("연결 실패");
            robotState = SCANNING;
            BLE.scan(true);
          }
        }
      } else {
        rssiOkStart = 0;
      }
    }
  }
  else if (robotState == CONNECTED) {
    if (!peripheral.connected()) {
      Serial.println("연결 끊김 → 재스캔");
      sendStatus("BMSBLE", 0);
      lastBLEState = 0;

      // 끊기면 즉시 릴레이 OFF
      setRelay(false);
      relayTimerArmed = false;
      relayTimerStart = 0;

      ble_reset();
      return;
    }

    // 연결 후 RSSI 감시
    if (millis() - lastRSSILog >= 1000) {
      int rssi = peripheral.rssi();
      Serial.print("연결 후 RSSI: ");
      Serial.println(rssi);
      lastRSSILog = millis();

      if (rssi <= -120) {
        Serial.println("RSSI 너무 약함 → 연결 해제");
        // 해제 직전에도 OFF 보장
        setRelay(false);
        relayTimerArmed = false;
        relayTimerStart = 0;

        ble_reset();
        return;
      }
    }

    // === 3초 타이머 경과 시 릴레이 ON ===
    if (relayTimerArmed && (millis() - relayTimerStart >= 3000)) {
      setRelay(true);                // ON
      relayTimerArmed = false;
      Serial.println("3초 경과 → 릴레이 ON");

      // 가능한 경우 Station에 즉시 상태 전송(선택, 없으면 5초 주기 보고에 맡김)
      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite()) {
        robotRelayChar.writeValue((uint8_t)1);
      }
    }

    // 5초마다 상태 보고/동기(필요 시 유지)
    if (currentMillis - lastReportTime >= 5000) {
      lastReportTime = currentMillis;

      // Docking 상태 읽기(있으면)
      if (dockingStatusChar && dockingStatusChar.canRead()) {
        byte dockingValue;
        if (dockingStatusChar.readValue(dockingValue)) {
          if (dockingValue != lastDockingStatus) {
            Serial.print("Docking 상태 수신: ");
            Serial.println(dockingValue);
            lastDockingStatus = dockingValue;
            sendStatus("DOCK", dockingValue);
          }
        } else {
          Serial.println("Docking 상태 읽기 실패");
        }
      }

      // 현재 릴레이 상태 전송(주기 보고)
      byte relayState = digitalRead(RELAY_PIN);

      if (peripheral.connected() && robotRelayChar && robotRelayChar.canWrite()) {
        if (!robotRelayChar.writeValue((uint8_t)relayState)) {
          Serial.println("릴레이 상태 전송 실패 → 재연결 시도");
          ble_reset();
          return;
        } else {
          rs485_reportRelayState(relayState);
        }
      } else {
        Serial.println("robotRelayChar 유효하지 않음 → 재연결 시도");
        ble_reset();
        return;
      }
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
*/