#ifndef STATION_LED_H
#define STATION_LED_H

#include "station_fsm.h"

void ledStatus_init();
void ledStatus_update();

void setLedState(StationState state);

#endif