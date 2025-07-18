#include <Arduino.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "robot_rs485.h"

#include "mbed.h"  // 🛡️ Watchdog 타이머를 위해 필요 (Nano RP2040용)

mbed::Watchdog &wdt = mbed::Watchdog::get_instance();

void setup() {
  delay(2000);
  Serial.begin(9600);

  gpio_init();
  delay(100);

  rs485_init();
  delay(100);

  ble_init();
  delay(100);

  // 🛡️ Watchdog 타이머 시작 (예: 5초)
  wdt.start(5000);
}

void loop() {
  wdt.kick();         // 🟢 Watchdog 리셋 → 시스템이 살아있다는 신호

  rs485_run();        // RS485 수신 처리
  ble_run();          // BLE FSM 실행
  rs485_report();     // 주기적 BLE + 상태 보고

  delay(10);          // 너무 길게 하지 말 것 (5초 안에는 최소 한 번 loop 돌아야 함)
}
