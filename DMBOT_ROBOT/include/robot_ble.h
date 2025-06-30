// robot_ble.h - BLE Central (Robot) side header

#ifndef ROBOT_BLE_H
#define ROBOT_BLE_H

#include <ArduinoBLE.h>

// BLE 연결 상태 enum
enum BLEState {
  BLE_IDLE,
  BLE_SCANNING,
  BLE_CONNECTED
};

extern BLEState bleState;
extern bool authSuccess;

// BLE 초기화 및 상태 갱신
void setupBLE();
void updateBLE();

// 인증 여부 및 충전기 상태
bool isBLEConnected();
bool isChargerStateOn();

#endif