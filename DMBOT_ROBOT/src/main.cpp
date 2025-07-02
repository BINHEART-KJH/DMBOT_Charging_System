#include <Arduino.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "robot_rs485.h"

void setup() {
  Serial.begin(9600);            // 디버깅용 Serial
  robotGPIO_init();              // 릴레이 제어 핀 초기화
  robotBLE_init();               // BLE 초기화
  robotRS485_init();            // RS485 통신 초기화
  robotFSM_init();              // 상태머신 초기화
}

void loop() {
  robotFSM_update();            // FSM 상태 업데이트
  robotBLE_update();           // BLE 상태 관리 (연결, 인증, GATT 읽기 등)
  robotRS485_update();         // RS485 명령 수신 및 응답 송신 처리
}