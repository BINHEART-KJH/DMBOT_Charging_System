#include "robot_ble.h"
#include "robot_auth.h"
#include "robot_gpio.h"
#include <Arduino.h>

static BLEDevice central;
static unsigned long lastGATTReadTime = 0;
static bool connected = false;

const char* targetLocalName = "DMBOT-STATION";
const int GATT_READ_INTERVAL = 5000;

void robotBLE_init() {
  if (!BLE.begin()) {
    Serial.println("[BLE] BLE 초기화 실패");
    return;
  }

  BLE.scan();
  Serial.println("[BLE] 스캔 시작");
}

void robotBLE_update() {
  if (!connected) {
    BLEDevice peripheral = BLE.available();
    if (peripheral && peripheral.hasLocalName() && peripheral.localName() == targetLocalName) {
      Serial.println("[BLE] Station 발견, 연결 시도 중...");
      BLE.stopScan();

      if (peripheral.connect()) {
        Serial.println("[BLE] 연결 성공");
        if (peripheral.discoverAttributes()) {
          Serial.println("[BLE] GATT 발견 성공");

          central = peripheral;
          robotAuth_reset();
          robotAuth_update(central);
          connected = true;
          lastGATTReadTime = millis();
        } else {
          Serial.println("[BLE] GATT 탐색 실패, 연결 해제");
          peripheral.disconnect();
          BLE.scan();
        }
      } else {
        Serial.println("[BLE] 연결 실패, 재스캔");
        BLE.scan();
      }
    }
  } else {
    // 연결 상태 유지 중
    if (!central.connected()) {
      Serial.println("[BLE] 연결 끊김, 재스캔");
      connected = false;
      robotAuth_reset();
      BLE.scan();
      return;
    }

    // 인증 시도
    robotAuth_update(central);

    // 인증 성공 후 주기적 GATT 상태 체크
    if (robotBLE_isAuthenticated()) {
      if (millis() - lastGATTReadTime > GATT_READ_INTERVAL) {
        Serial.println("[BLE] GATT 상태 주기적 체크...");

        // 예: Battery 상태 characteristic 읽기 시도
        BLECharacteristic batteryChar = central.characteristic("battery_state_uuid");
        if (batteryChar && batteryChar.canRead()) {
          if (batteryChar.read()) {
            int value = batteryChar.value()[0];
            Serial.print("[BLE] Battery 상태: ");
            Serial.println(value);
          } else {
            Serial.println("[BLE] GATT 읽기 실패, 연결 해제");
            robotBLE_disconnect();
            BLE.scan();
          }
        }

        lastGATTReadTime = millis();
      }
    }
  }
}

bool robotBLE_isConnected() {
  return connected && central.connected();
}

bool robotBLE_isAuthenticated() {
  return isRobotAuthenticated();
}

void robotBLE_disconnect() {
  if (central && central.connected()) {
    central.disconnect();
  }
  connected = false;
  robotAuth_reset();
}
