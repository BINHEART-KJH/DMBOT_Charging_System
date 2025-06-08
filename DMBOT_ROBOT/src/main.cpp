#include <Arduino.h>
#include <ArduinoBLE.h>
#include "rs485_control.h"
#include "ble_scanner.h"

#define RELAY_PIN 4  // D4 대신 직접 지정

void setup() {
  Serial.begin(9600);
  setupRS485();
  setupBLEScanner();
  pinMode(RELAY_PIN, OUTPUT);  // BATTERY_READY 릴레이
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[MAIN] Robot 시작됨");
}

void loop() {
  processRS485Command();

  if (isBLEScanRequested()) {
    startScan();
    clearBLECommandFlags();
  }

   if (isBLEStopRequested()) {
    stopScan();                    // 스캔 중지
    disconnectFromStation();       // ✅ 실제 연결 해제
    digitalWrite(RELAY_PIN, LOW);  // 릴레이 OFF
    clearBLECommandFlags();
  }

  updateBLEScanLoop();
}