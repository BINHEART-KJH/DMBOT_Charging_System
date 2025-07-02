#include "station_gpio.h"

static bool dockingPinState = false;
static bool lastReadState = false;
static bool lastDockingPin = false;
static unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 30;  // ms


void stationGPIO_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

void stationGPIO_update() {
  bool reading = digitalRead(DOCKING_PIN);
  unsigned long currentTime = millis();

  if (reading != lastDockingPin) {
    lastDebounceTime = currentTime;
  }

  if ((currentTime - lastDebounceTime) > debounceDelay) {
    if (reading != dockingPinState) {
      dockingPinState = reading;
      if (dockingPinState) {
        Serial.println("[GPIO] 도킹 감지됨 (HIGH)");
      } else {
        Serial.println("[GPIO] 도킹 해제됨 (LOW)");
      }
    }
  }

  lastDockingPin = reading;
}

bool isDockingPinHigh() {
  return dockingPinState;
}

void setRelayState(bool on) {
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

void setLED(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
}

void relay_set(bool on) {
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}