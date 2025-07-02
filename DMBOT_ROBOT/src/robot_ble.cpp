// robot_ble.cpp - Station 구조 기반 BLE Central 구현

#include "robot_ble.h"
#include "robot_gpio.h"
#include "hmac.h"
#include <ArduinoBLE.h>

// === UUIDs ===
#define SERVICE_UUID              "dmb0t001-5e42-4f8e-a2d3-d3b0t5t4710n"
#define NONCE_CHAR_UUID           "dmb0t002-6e42-4f8e-a2d3-d3b0t5t4710n"
#define TOKEN_CHAR_UUID           "dmb0t003-7e42-4f8e-a2d3-d3b0t5t4710n"
#define CHARGERSTATE_CHAR_UUID    "dmb0t004-8e42-4f8e-a2d3-d3b0t5t4710n"

#define LOCAL_NAME                "DM-STATION"
#define AUTH_TIMEOUT_MS           1500
#define RSSI_THRESHOLD            -65

enum BLEState {
  BLE_IDLE,
  BLE_SCANNING,
  BLE_CONNECTING,
  BLE_CONNECTED
};

BLEState bleState = BLE_IDLE;

BLEDevice peripheral;
bool authSuccess = false;
bool chargerState = false;

char nonceBuffer[32];
const char* secretKey = "DMBOT-SECRET-KEY";

BLECharacteristic nonceChar;
BLECharacteristic tokenChar;
BLECharacteristic chargerStateChar;

unsigned long authStartTime = 0;

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("[BLE] Init failed");
    return;
  }
  BLE.setLocalName("ROBOT-CENTRAL");
  BLE.setDeviceName("ROBOT-CENTRAL");
  Serial.println("[BLE] Initialized");
}

void startBLEScan() {
  BLE.scan();  // Start scanning
  Serial.println("[BLE] Scanning started");
}

void updateBLEState() {
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    Serial.print("[BLE] Found device: ");
    Serial.println(peripheral.address());

    // TODO: 연결, 인증, GATT 읽기 로직 등 추가 예정
  }

  // 연결 상태 및 특성 상태 업데이트 로직 추가 가능
}

bool isBLEConnected() {
  return BLE.connected();
}

bool isChargerStateOn() {
  return chargerState;
}