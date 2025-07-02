#include <Arduino.h>
#include "station_fsm.h"
#include "station_gpio.h"
#include "station_auth.h"
#include "station_led.h"

void setup() {
  Serial.begin(9600);
  stationGPIO_init();
  ledStatus_init();
  stationFSM_init();
}

void loop() {
  stationGPIO_update();
  stationFSM_update();
  ledStatus_update();
}