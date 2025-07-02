#ifndef ROBOT_BLE_H
#define ROBOT_BLE_H

#include <ArduinoBLE.h>

void robotBLE_init();
void robotBLE_update();
bool robotBLE_isConnected();
bool robotBLE_isAuthenticated();
void robotBLE_disconnect();

#endif