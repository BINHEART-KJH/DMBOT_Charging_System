#include <Arduino.h>
#include "station_fsm.h"
#include "station_gpio.h"
#include "station_led.h"
#include "station_ble.h"
#include "config.h"

StationState lastState = IDLE;

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("\n========================================");
  Serial.println("DMBOT Station - Charging System (Improved)");
  Serial.println("========================================");

  gpio_init();
  delay(100);
  
  led_init();
  delay(100);
  
  ble_init();
  delay(100);

  LOG_INFO("Setup complete - entering main loop");
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
