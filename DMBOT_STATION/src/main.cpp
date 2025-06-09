/*#include <Arduino.h>
#include "ble_auth.h"

void setup() {
  Serial.begin(9600);
  delay(1000);  // Serial 안정화
  Serial.println("[MAIN] Station 초기화 시작");

  setupBLE();  // BLE 초기화 (advertise 시작)
}

void loop() {
  updateBLEStateMachine();  // BLE 연결 및 인증 상태 처리
}*/


#include <Arduino.h>
#include "ble_auth.h"
#include "led_status.h"

void setup() {
  Serial.begin(9600);
  delay(1000);  // Serial 안정화
  Serial.println("[MAIN] Station 초기화 시작");

  setupLED();    // RGB LED 초기화
  setupBLE();     // BLE 초기화 및 광고 시작
}

void loop() {
  updateBLEStateMachine();  // BLE 연결 및 인증 상태 처리
  updateLEDEffect();        // LED 상태 및 애니메이션 처리
}