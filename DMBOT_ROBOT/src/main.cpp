#include <Arduino.h>
#include "ble_scanner.h"

String serialBuffer;

void setup() {
  pinMode(4, OUTPUT);  // RELAY_PIN

  Serial.begin(9600);
  while (!Serial);  // USB 연결 대기 (전원만으로 동작 시 제거 가능)

  setupBLEScanner();  // BLE 초기화
  startScan();        // BLE 스캔 시작

  Serial.println("[MAIN] 시스템 시작됨");
}

void loop() {
  updateBLEScanLoop();  // BLE 상태 갱신

  // Serial 입력 수신 처리
  while (Serial.available()) {
    char c = Serial.read();
    serialBuffer += c;
    if (c == '\n') {
      serialBuffer.trim();
      processSerialCommand(serialBuffer.c_str());  // 🔧 String → const char*
      serialBuffer = "";  // 입력 버퍼 초기화
    }
  }
}