#ifndef STATION_BLE_H
#define STATION_BLE_H

#include <ArduinoBLE.h>

void setupBLE();
void updateBLE();
void startAdvertising();
void disconnectAndReset();
bool isAdvertising();

// BLE 상태 정의
enum BLEState {
  IDLE,
  ADVERTISING,
  WAIT_AUTH,
  CONNECTED
};

// BLE 상태 접근 함수
BLEState getBLEState();

// chargerStateChar 제어용
void setChargerState(bool on);

#endif