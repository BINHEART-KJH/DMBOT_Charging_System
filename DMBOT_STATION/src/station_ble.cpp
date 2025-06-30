#include "station_ble.h"
#include "station_gpio.h"
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

BLEService dmService(SERVICE_UUID);
BLEStringCharacteristic nonceChar(NONCE_CHAR_UUID, BLERead, 32);
BLEStringCharacteristic tokenChar(TOKEN_CHAR_UUID, BLEWrite, 64);
BLEStringCharacteristic chargerStateChar(CHARGERSTATE_CHAR_UUID, BLERead, 1);

BLEDevice central;
BLEState bleState = IDLE;

char nonceBuffer[32];
bool authenticated = false;
bool chargerOn = false;
unsigned long authStartTime = 0;

const char* secretKey = "DMBOT-SECRET-KEY";

// 상태 외부 접근용
BLEState getBLEState() {
  return bleState;
}

bool isAdvertising() {
  return bleState == ADVERTISING;
}

void setChargerState(bool on) {
  chargerOn = on;
  chargerStateChar.writeValue(on ? "1" : "0");
}

String generateNonce() {
  String nonce = "";
  for (int i = 0; i < 16; ++i) {
    nonce += (char)random(48, 90);
  }
  return nonce;
}

void startAdvertising() {
  if (bleState == ADVERTISING) return;

  BLE.stopAdvertise();
  BLE.setLocalName(LOCAL_NAME);
  BLE.setAdvertisedService(dmService);
  BLE.advertise();

  bleState = ADVERTISING;
  authenticated = false;
}

void disconnectAndReset() {
  BLE.disconnect();
  bleState = IDLE;
  authenticated = false;
  setChargerState(false);
  BLE.stopAdvertise();
}

void setupBLE() {
  if (!BLE.begin()) {
    while (1); // 실패 시 멈춤 (LED 진단으로 대체 가능)
  }

  BLE.setDeviceName(LOCAL_NAME);
  BLE.setLocalName(LOCAL_NAME);

  dmService.addCharacteristic(nonceChar);
  dmService.addCharacteristic(tokenChar);
  dmService.addCharacteristic(chargerStateChar);

  BLE.setAdvertisedService(dmService);
  BLE.addService(dmService);

  setChargerState(false);
}

void updateBLE() {
  BLEDevice newCentral = BLE.central();

  // 새 연결 요청
  if (bleState == ADVERTISING && newCentral) {
    if (newCentral.rssi() < RSSI_THRESHOLD) {
      newCentral.disconnect();
      return;
    }

    central = newCentral;
    bleState = WAIT_AUTH;

    String nonce = generateNonce();
    nonceChar.writeValue(nonce);
    strncpy(nonceBuffer, nonce.c_str(), sizeof(nonceBuffer) - 1);
    nonceBuffer[sizeof(nonceBuffer) - 1] = '\0';

    authStartTime = millis();
    return;
  }

  if (bleState == WAIT_AUTH) {
    if (!central.connected()) {
      disconnectAndReset();
      return;
    }

    if (tokenChar.written()) {
      char tokenBuf[65] = {0};
      tokenChar.readValue(tokenBuf, sizeof(tokenBuf) - 1);

      char expectedHMAC[65];
      generateHMAC_SHA256(nonceBuffer, secretKey, expectedHMAC);

      if (strncmp(expectedHMAC, tokenBuf, 64) == 0) {
        authenticated = true;
        bleState = CONNECTED;
      } else {
        disconnectAndReset();
        return;
      }
    }

    if (millis() - authStartTime > AUTH_TIMEOUT_MS) {
      disconnectAndReset();
      return;
    }
  }

  if (bleState == CONNECTED) {
    if (!central.connected()) {
      disconnectAndReset();
      return;
    }
    // GATT 상태는 GPIO에서 업데이트됨
  }
}