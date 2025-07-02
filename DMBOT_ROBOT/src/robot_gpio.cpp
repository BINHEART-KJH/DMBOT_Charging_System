#include <Arduino.h>
#include "robot_gpio.h"

#define BATTERY_RELAY_PIN 4

bool relayState = false;

void gpio_init() {
  pinMode(BATTERY_RELAY_PIN, OUTPUT);
  setRelay(false);  // 초기 OFF
}

void setRelay(bool on) {
  relayState = on;
  digitalWrite(BATTERY_RELAY_PIN, on ? HIGH : LOW);
}

bool getRelayState() {
  return relayState;
}