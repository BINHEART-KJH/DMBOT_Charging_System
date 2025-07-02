#include "robot_gpio.h"
#include <Arduino.h>

#define BATTERY_RELAY_PIN 4  // D4 → GPIO 4

void robotGPIO_init() {
  pinMode(BATTERY_RELAY_PIN, OUTPUT);
  digitalWrite(BATTERY_RELAY_PIN, LOW);  // 기본 OFF
}

void robotGPIO_setRelay(bool on) {
  digitalWrite(BATTERY_RELAY_PIN, on ? HIGH : LOW);
}