#ifndef STATION_FSM_H
#define STATION_FSM_H

#include <Arduino.h>

// FSM 상태 정의
enum StationState {
  IDLE,
  DOCKING_OK,
  ADVERTISING,
  CONNECTED,
  CONNECTING,
  ERROR
};

extern StationState currentState;

// BLE 광고 상태를 인자로 전달
void state_update(bool isAdvertising);
StationState get_current_state();

#endif