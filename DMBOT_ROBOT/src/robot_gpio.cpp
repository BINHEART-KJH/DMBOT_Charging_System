#include "robot_gpio.h"
#include "robot_ble.h"   // ✅ 추가

#define RELAY_PIN 4       // D4

static bool relayOn = false;  // ✅ 전역 선언

void setupGPIO() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  relayOn = false;
}

void updateGPIO() {
  if (isBLEConnected() && isChargerStateOn()) {
    if (!relayOn) {
      digitalWrite(RELAY_PIN, HIGH);
      relayOn = true;
    }
  } else {
    if (relayOn) {
      digitalWrite(RELAY_PIN, LOW);
      relayOn = false;
    }
  }
}

bool isRelayOn() {
  return relayOn;
}

void forceRelayState(bool state) {
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
  relayOn = state;
}