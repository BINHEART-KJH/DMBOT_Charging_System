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

BLEDevice peripheral;
BLEState bleState = BLE_IDLE;
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

bool isBLEConnected() {
  return bleState == BLE_CONNECTED && authSuccess;
}

bool isChargerStateOn() {
  return chargerState;
}

void updateBLE() {
  switch (bleState) {
    case BLE_IDLE:
      Serial.println("[BLE] Scanning...");
      BLE.scan();
      bleState = BLE_SCANNING;
      break;

    case BLE_SCANNING: {
      BLEDevice dev = BLE.available();
      if (dev && dev.hasLocalName() && dev.localName() == LOCAL_NAME && dev.rssi() > RSSI_THRESHOLD) {
        BLE.stopScan();
        if (dev.connect()) {
          peripheral = dev;
          bleState = BLE_CONNECTED;
          Serial.println("[BLE] Connected to Station");
        } else {
          Serial.println("[BLE] Connect failed");
          bleState = BLE_IDLE;
        }
      }
      break;
    }

    case BLE_CONNECTED:
      if (!peripheral.connected()) {
        Serial.println("[BLE] Disconnected");
        BLE.disconnect();
        bleState = BLE_IDLE;
        authSuccess = false;
        chargerState = false;
        break;
      }

      if (!peripheral.discoverAttributes()) {
        Serial.println("[BLE] GATT Discovery Failed");
        BLE.disconnect();
        bleState = BLE_IDLE;
        break;
      }

      nonceChar = peripheral.characteristic(NONCE_CHAR_UUID);
      tokenChar = peripheral.characteristic(TOKEN_CHAR_UUID);
      chargerStateChar = peripheral.characteristic(CHARGERSTATE_CHAR_UUID);

      if (!nonceChar || !tokenChar || !chargerStateChar) {
        Serial.println("[BLE] GATT characteristic missing");
        BLE.disconnect();
        bleState = BLE_IDLE;
        break;
      }

      if (!nonceChar.canRead() || !tokenChar.canWrite() || !chargerStateChar.canRead()) {
        Serial.println("[BLE] Invalid permissions");
        BLE.disconnect();
        bleState = BLE_IDLE;
        break;
      }

      // === 인증 ===
      nonceChar.readValue((uint8_t*)nonceBuffer, sizeof(nonceBuffer) - 1);
      nonceBuffer[sizeof(nonceBuffer) - 1] = '\0';

      char tokenOut[65] = {0};
      generateHMAC_SHA256(secretKey, nonceBuffer, tokenOut);
      tokenChar.writeValue((uint8_t*)tokenOut, 64);

      authStartTime = millis();
      while (millis() - authStartTime < AUTH_TIMEOUT_MS) {
        delay(50);
        if (!peripheral.connected()) break;

        uint8_t stateVal[2] = {0};
        if (chargerStateChar.readValue(stateVal, 1)) {
          if (stateVal[0] == '1') {
            authSuccess = true;
            break;
          }
        }
      }

      if (!authSuccess) {
        Serial.println("[BLE] Auth timeout or fail");
        BLE.disconnect();
        bleState = BLE_IDLE;
        break;
      }

      Serial.println("[BLE] Authenticated");
      bleState = BLE_CONNECTED;
      break;
  }

  if (authSuccess && chargerStateChar) {
    uint8_t val[2] = {0};
    if (chargerStateChar.readValue(val, 1)) {
      chargerState = (val[0] == '1');
    }
  }
}