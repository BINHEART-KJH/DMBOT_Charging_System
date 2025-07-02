#include <Arduino.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "robot_rs485.h"

void setup() {
  Serial.begin(9600);
  gpio_init();
  rs485_init();
  ble_init();
}

void loop() {
  rs485_run();  // RS485 수신
  ble_run();    // BLE 상태 FSM
  rs485_report();  // 주기적 BLE + 상태 보고
}