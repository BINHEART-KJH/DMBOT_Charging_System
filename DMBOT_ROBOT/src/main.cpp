/*#include <Arduino.h>
#include <ArduinoBLE.h>
#include "modbus_slave.h"
#include "ble_scanner.h"

#define RELAY_PIN 4  // D4 대신 직접 지정

void setup() {
  Serial.begin(9600);
  setupModbus();
  setupBLEScanner();
  pinMode(RELAY_PIN, OUTPUT);  // BATTERY_READY 릴레이
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[MAIN] Robot 시작됨");
}

void loop() {
  updateModbus();         // RS485 명령 수신
  updateBLEScanLoop();    // BLE 처리

  static bool lastBLE = false;
  bool nowBLE = modbusGetBLECmd();

  if (nowBLE != lastBLE) {
    lastBLE = nowBLE;
    if (nowBLE) {
      Serial.println("[MAIN] BLE 스캔 시작");
      startScan();
    } else {
      Serial.println("[MAIN] BLE 스캔 중지");
      stopScan();
      disconnectFromStation();       // 연결 종료
      digitalWrite(RELAY_PIN, LOW);  // 릴레이 OFF
    }
  }
}
*/


#include <Arduino.h>
#include <ArduinoBLE.h>
#include "ble_scanner.h"

#define RELAY_PIN 4  // BATTERY_READY 릴레이 D4

void setup() {
  Serial.begin(9600);
  setupBLEScanner();

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  Serial.println("[DEBUG] Modbus 없이 Robot 시작됨");
  startScan();  // 부팅 시 자동 BLE 스캔 시작
}

void loop() {
  updateBLEScanLoop();  // BLE 연결, 인증, 릴레이 제어 등 처리
}