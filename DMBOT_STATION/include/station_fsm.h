#ifndef STATION_FSM_H
#define STATION_FSM_H

enum StationState {
  ST_IDLE,
  ST_DOCKED,
  ST_ADVERTISING,
  ST_CONNECTING,
  ST_AUTHENTICATING,
  ST_CONNECTED,
  ST_ERROR
};

void stationFSM_init();
void stationFSM_update();

StationState getStationState();
void setStationState(StationState newState);

#endif