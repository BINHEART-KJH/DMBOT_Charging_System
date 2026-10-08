#include <Arduino.h>
#include "robot_gpio.h"
#include "config.h"

#define BATTERY_RELAY_PIN 4

bool relayState = false;

/* ===== ORIGINAL CODE (기존 코드 주석 처리) =====
//bool relayState = false;
//
//void gpio_init() {
//  pinMode(BATTERY_RELAY_PIN, OUTPUT);
//  setRelay(false);  // 초기 OFF
//}
//
//void setRelay(bool on) {
//  relayState = on;
//  digitalWrite(BATTERY_RELAY_PIN, on ? HIGH : LOW);
//}
//
//bool getRelayState() {
//  return relayState;
//}
===== END ORIGINAL CODE ===== */

// ===== IMPROVED GPIO LOGIC =====
void gpio_init() {
  pinMode(BATTERY_RELAY_PIN, OUTPUT);
  digitalWrite(BATTERY_RELAY_PIN, LOW);
  relayState = false;
  LOG_INFO("Robot GPIO initialized");
}

void setRelay(bool on) {
  relayState = on;
  digitalWrite(BATTERY_RELAY_PIN, on ? HIGH : LOW);
  LOG_DEBUG("Robot relay set: %s", on ? "ON" : "OFF");
}

bool getRelayState() {
  return relayState;
}
