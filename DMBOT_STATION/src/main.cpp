#include <Arduino.h>
#include "station_fsm.h"
#include "station_gpio.h"
#include "station_led.h"
#include "station_ble.h"

StationState lastState = IDLE;

void ble_rs485_run() {}  // 사용하지 않음

void setup() {
  Serial.begin(9600);
  delay(500);

  gpio_init();
  led_init();
  ble_init();

  Serial.println("Station Setup Complete");
}

void loop() {
  gpio_run();
  ble_run();
  state_update(isAdvertising);
  led_run();

  StationState current = get_current_state();
  if (current != lastState) {
    lastState = current;
    if (current == IDLE) {
      ble_reset();
    }
  }

  delay(10);
}
