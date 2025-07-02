#ifndef ROBOT_AUTH_H
#define ROBOT_AUTH_H

#include <ArduinoBLE.h>

void robotAuth_init();
void robotAuth_reset();
void robotAuth_update(BLEDevice central);
bool isRobotAuthenticated();

extern BLEService authService;
extern BLECharacteristic nonceChar;
extern BLECharacteristic tokenChar;

#endif