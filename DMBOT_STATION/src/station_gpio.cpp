#include "station_gpio.h"
#include "station_ble.h"

#define DOCKING_PIN        2
#define CHARGER_OK_PIN     6
#define BATTERY_FULL_PIN   5
#define RELAY_PIN          7

// 내부 상태 변수
bool docked = false;
bool relayOn = false;

unsigned long lastDockChangeTime = 0;
bool lastDockRead = false;

unsigned long relayStartTime = 0;
const unsigned long RELAY_TRIGGER_DELAY_MS = 10000;

void setupGPIO() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(CHARGER_OK_PIN, INPUT);
  pinMode(BATTERY_FULL_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
}

bool readDockingState() {
  return digitalRead(DOCKING_PIN) == HIGH;
}

void updateGPIO() {
  // 도킹 핀 디바운스 (HIGH 유지 100ms → 광고 시작)
  bool currentDock = readDockingState();
  unsigned long now = millis();

  if (currentDock != lastDockRead) {
    lastDockChangeTime = now;
    lastDockRead = currentDock;
  }

  if (!docked && currentDock && (now - lastDockChangeTime >= 100)) {
    docked = true;
    startAdvertising();
  }

  if (docked && !currentDock) {
    docked = false;
    disconnectAndReset();
  }

  // 릴레이 제어 로직
  if (getBLEState() == CONNECTED && isDocked()) {
    if (isChargerOK() && !isBatteryFull()) {
      if (relayStartTime == 0) relayStartTime = now;

      if ((now - relayStartTime >= RELAY_TRIGGER_DELAY_MS) && !relayOn) {
        relayOn = true;
        digitalWrite(RELAY_PIN, HIGH);
        setChargerState(true); // BLE GATT 업데이트
      }
    } else {
      relayStartTime = 0;
      if (relayOn) {
        relayOn = false;
        digitalWrite(RELAY_PIN, LOW);
        setChargerState(false);
      }
    }
  } else {
    relayStartTime = 0;
    if (relayOn) {
      relayOn = false;
      digitalWrite(RELAY_PIN, LOW);
      setChargerState(false);
    }
  }
}

bool isDocked() {
  return docked;
}

bool isChargerOK() {
  return digitalRead(CHARGER_OK_PIN) == HIGH;
}

bool isBatteryFull() {
  return digitalRead(BATTERY_FULL_PIN) == HIGH;
}

bool isRelayOn() {
  return relayOn;
}