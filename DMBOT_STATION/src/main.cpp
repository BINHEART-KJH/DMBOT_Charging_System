#include <Arduino.h>
#include "station_ble.h"
#include "station_gpio.h"
#include "station_led.h"

void setup() {
  Serial.begin(9600);

  setupGPIO();
  setupLED();
  setupBLE();
}

void loop() {
  updateGPIO();
  updateBLE();
  updateLED();
}