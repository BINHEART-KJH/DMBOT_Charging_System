#ifndef STATION_AUTH_H
#define STATION_AUTH_H

#include <ArduinoBLE.h>

void stationAuth_init();
void stationAuth_reset();
void stationAuth_update(BLEDevice central);
bool isStationAuthenticated();

extern BLEService authService;
extern BLECharacteristic nonceChar;
extern BLECharacteristic tokenChar;

#endif