#include "station_fsm.h"
#include "station_gpio.h"
#include "station_ble.h"  // authSuccess를 사용하기 위해

StationState currentState = IDLE;

unsigned long lastStateCheckTime = 0;
const unsigned long stateCheckInterval = 500;

void state_update(bool isAdvertising) {
  unsigned long currentMillis = millis();

  if (currentMillis - lastStateCheckTime >= stateCheckInterval) {
    lastStateCheckTime = currentMillis;

    int docking = digitalRead(DOCKING_PIN);
    StationState newState;

    if (docking == LOW) {
      newState = IDLE;
    } else {
      BLEDevice central = BLE.central();
      if (central && central.connected()) {
        if (authSuccess) {
          newState = CONNECTED;
        } else {
          newState = CONNECTING;
        }
      } else if (isAdvertising) {
        newState = ADVERTISING;
      } else {
        newState = DOCKING_OK;
      }
    }

    if (newState != currentState) {
      currentState = newState;
      Serial.print("FSM changed → ");
      switch (currentState) {
        case IDLE: Serial.println("IDLE"); break;
        case DOCKING_OK: Serial.println("DOCKING_OK"); break;
        case ADVERTISING: Serial.println("ADVERTISING"); break;
        case CONNECTING: Serial.println("CONNECTING"); break;
        case CONNECTED: Serial.println("CONNECTED"); break;
      }
    }
  }
}

StationState get_current_state() {
  return currentState;
}
