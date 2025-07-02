// #include <ArduinoBLE.h>
// #include "robot_fsm.h"
// #include "robot_ble.h"
// #include "hmac.h"
// #include "sha256.h"

// const char *targetLocalName = "DM-STATION";
// const char *sharedKey = "DM--010225";

// BLEDevice peripheral;
// BLECharacteristic nonceChar;
// BLECharacteristic authTokenChar;
// BLECharacteristic batteryFullChar;
// BLECharacteristic chargerOKChar;
// BLECharacteristic jumperRelayChar;

// char nonce[9];     // 8 + null
// char tokenHex[17]; // 16 + null

// bool authenticated = false;
// unsigned long lastStatusRead = 0;
// unsigned long lastRs485Report = 0;

// byte lastBLEState = 0xFF;
// byte lastBatteryFull = 0xFF;
// byte lastChargerOK = 0xFF;
// byte lastJumperRelay = 0xFF;

// void sendStatus(const char* label, byte value) {
//   Serial1.print("ST,0,");
//   Serial1.print(label);
//   Serial1.print(",");
//   Serial1.print(value);
//   Serial1.println(",ED");
// }

// void generateHMAC_SHA256(const char *key, const char *message, char *outputHex) {
//   uint8_t hmacResult[32];
//   HMAC hmac;
//   hmac.init((const uint8_t *)key, strlen(key));
//   hmac.update((const uint8_t *)message, strlen(message));
//   hmac.finalize(hmacResult, sizeof(hmacResult));
//   for (int i = 0; i < 8; ++i) {
//     sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
//   }
//   outputHex[16] = '\0';
// }

// void ble_init() {
//   if (!BLE.begin()) {
//     Serial.println("❌ BLE 초기화 실패!");
//     return;
//   }
//   Serial.println("✅ BLE 초기화 완료");
//   BLE.scan(true);
//   robotState = SCANNING;
// }

// void ble_reset() {
//   if (peripheral) peripheral.disconnect();
//   BLE.stopScan();
//   authenticated = false;
//   robotState = IDLE;
//   BLE.scan(true);
//   robotState = SCANNING;
// }

// void ble_run() {
//   if (robotState == SCANNING) {
//     BLEDevice device = BLE.available();
//     if (device && device.hasLocalName() && device.localName() == targetLocalName) {
//       Serial.print("📡 발견된 장치: ");
//       Serial.println(device.localName());

//       BLE.stopScan();
//       Serial.println("📶 연결 시도 중...");
//       robotState = CONNECTING;

//       if (device.connect()) {
//         Serial.println("✅ 연결 성공!");
//         peripheral = device;

//         if (peripheral.discoverAttributes()) {
//           Serial.println("🔍 GATT 속성 탐색 완료");

//           nonceChar       = peripheral.characteristic("2A03");
//           authTokenChar   = peripheral.characteristic("2A04");
//           batteryFullChar = peripheral.characteristic("2A01");
//           chargerOKChar   = peripheral.characteristic("2A02");
//           jumperRelayChar = peripheral.characteristic("AA05");

//           if (nonceChar && nonceChar.canRead() && authTokenChar && authTokenChar.canWrite()) {
//             byte buf[20];
//             int len = nonceChar.readValue(buf, sizeof(buf));
//             if (len > 0 && len < sizeof(nonce)) {
//               memcpy(nonce, buf, len);
//               nonce[len] = '\0';

//               Serial.print("📩 nonce 수신: ");
//               Serial.println(nonce);

//               generateHMAC_SHA256(sharedKey, nonce, tokenHex);
//               Serial.print("➡️ 토큰 전송: ");
//               Serial.println(tokenHex);

//               authTokenChar.writeValue((const unsigned char *)tokenHex, 16);
//               authenticated = true;
//               robotState = CONNECTED;
//               lastStatusRead = millis();
//               lastRs485Report = millis();
//             }
//           }
//         } else {
//           Serial.println("❌ GATT 탐색 실패");
//           ble_reset();
//         }
//       } else {
//         Serial.println("❌ 연결 실패");
//         robotState = SCANNING;
//         BLE.scan(true);
//       }
//     }
//   }

//   else if (robotState == CONNECTED) {
//     if (!peripheral.connected()) {
//       Serial.println("🔌 연결 끊김 → 재스캔");
//       sendStatus("BMSBLE", 0);
//       lastBLEState = 0;
//       ble_reset();
//       return;
//     }

//     // 3초마다 BLE GATT 읽기
//     if (millis() - lastStatusRead >= 3000) {
//       lastStatusRead = millis();

//       byte val;

//       if (batteryFullChar.readValue(&val, 1)) {
//         Serial.print("🔋 Battery Full: ");
//         Serial.println(val ? "YES" : "NO");
//         if (val != lastBatteryFull) {
//           sendStatus("BMSBATFULL", val);
//           lastBatteryFull = val;
//         }
//       } else {
//         Serial.println("⚠️ batteryFullChar read 실패");
//       }

//       if (chargerOKChar.readValue(&val, 1)) {
//         Serial.print("🔌 Charger OK: ");
//         Serial.println(val ? "YES" : "NO");
//         if (val != lastChargerOK) {
//           sendStatus("BMSCHARGER", val);
//           lastChargerOK = val;
//         }
//       } else {
//         Serial.println("⚠️ chargerOKChar read 실패");
//       }

//       if (jumperRelayChar.readValue(&val, 1)) {
//         Serial.print("🔗 Jumper Relay: ");
//         Serial.println(val ? "ON" : "OFF");
//         if (val != lastJumperRelay) {
//           sendStatus("BMSCHARGERRELAY", val);
//           lastJumperRelay = val;
//         }
//       } else {
//         Serial.println("⚠️ jumperRelayChar read 실패");
//       }

//       Serial.println("---------------------------------");
//     }

//     // 5초마다 BLE 연결 상태 주기 보고
//     if (millis() - lastRs485Report >= 5000) {
//       lastRs485Report = millis();
//       if (lastBLEState != 1) {
//         sendStatus("BMSBLE", 1);
//         lastBLEState = 1;
//       }
//     }
//   }
// }

// // 상태 Getter 함수 정의
// bool getBleConnectionState() {
//   return (robotState == CONNECTED && peripheral.connected());
// }

// bool getBatteryFullStatus() {
//   return lastBatteryFull;
// }

// bool getChargerOkStatus() {
//   return lastChargerOK;
// }

// bool getChargerRelayStatus() {
//   return lastJumperRelay;
// }

#include <ArduinoBLE.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"  // BATTERY_RELAY_PIN 정의 포함
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

char nonce[9];     // 8 + null
char tokenHex[17]; // 16 + null

bool authenticated = false;
unsigned long lastStatusRead = 0;
unsigned long lastRs485Report = 0;

byte lastBLEState = 0xFF;
byte lastBatteryFull = 0xFF;
byte lastChargerOK = 0xFF;
byte lastJumperRelay = 0xFF;

void sendStatus(const char* label, byte value) {
  Serial1.print("ST,0,");
  Serial1.print(label);
  Serial1.print(",");
  Serial1.print(value);
  Serial1.println(",ED");
}

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

void ble_init() {
  if (!BLE.begin()) {
    Serial.println("❌ BLE 초기화 실패!");
    return;
  }
  Serial.println("✅ BLE 초기화 완료");
  BLE.scan(true);
  robotState = SCANNING;
}

void ble_reset() {
  if (peripheral) peripheral.disconnect();
  BLE.stopScan();
  authenticated = false;
  robotState = IDLE;
  BLE.scan(true);
  robotState = SCANNING;
}

void ble_run() {
  if (robotState == SCANNING) {
    BLEDevice device = BLE.available();
    if (device && device.hasLocalName() && device.localName() == targetLocalName) {
      Serial.print("📡 발견된 장치: ");
      Serial.println(device.localName());

      BLE.stopScan();
      Serial.println("📶 연결 시도 중...");
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
  }

  else if (robotState == CONNECTED) {
    if (!peripheral.connected()) {
      Serial.println("🔌 연결 끊김 → 재스캔");
      sendStatus("BMSBLE", 0);
      lastBLEState = 0;
      ble_reset();
      return;
    }

    // 3초마다 BLE GATT 읽기
    if (millis() - lastStatusRead >= 3000) {
      lastStatusRead = millis();

      byte val;

      if (batteryFullChar.readValue(&val, 1)) {
        Serial.print("🔋 Battery Full: ");
        Serial.println(val ? "YES" : "NO");
        if (val != lastBatteryFull) {
          sendStatus("BMSBATFULL", val);
          lastBatteryFull = val;
        }
      } else {
        Serial.println("⚠️ batteryFullChar read 실패");
      }

      if (chargerOKChar.readValue(&val, 1)) {
        Serial.print("🔌 Charger OK: ");
        Serial.println(val ? "YES" : "NO");
        if (val != lastChargerOK) {
          sendStatus("BMSCHARGER", val);
          lastChargerOK = val;
        }
      } else {
        Serial.println("⚠️ chargerOKChar read 실패");
      }

      if (jumperRelayChar.readValue(&val, 1)) {
        Serial.print("🔗 Jumper Relay: ");
        Serial.println(val ? "ON" : "OFF");
        if (val != lastJumperRelay) {
          sendStatus("BMSCHARGERRELAY", val);
          lastJumperRelay = val;
        }
      } else {
        Serial.println("⚠️ jumperRelayChar read 실패");
      }

      Serial.println("---------------------------------");
    }

    // 5초마다 BLE 및 릴레이 상태 RS485 전송
    if (millis() - lastRs485Report >= 5000) {
      lastRs485Report = millis();

      // BLE 연결 상태
      if (lastBLEState != 1) {
        sendStatus("BMSBLE", 1);
        lastBLEState = 1;
      }

      // Battery_on 릴레이 상태 전송
      byte relayState = digitalRead(BATTERY_RELAY_PIN);
      sendStatus("BMSROBOT", relayState);
    }
  }
}

// 상태 Getter 함수 정의
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
