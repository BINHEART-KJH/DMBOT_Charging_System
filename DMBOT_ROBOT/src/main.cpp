#include <Arduino.h>
#include "robot_gpio.h"
#include "robot_rs485.h"
#include "robot_ble.h"

void setup() {
  Serial.begin(9600);       // 디버깅용 Serial
  setupGPIO();              // 릴레이 초기화 (D4)
  setupRS485();             // RS485 통신 초기화 (Serial1)
  setupBLE();               // BLE 초기화 및 준비
}

void loop() {
  updateRS485();            // RS485 명령 수신 처리
  updateBLE();              // BLE 스캔 및 인증 상태머신 처리
  updateGPIO();             // BLE 또는 수동 명령 기반 릴레이 상태 유지
}