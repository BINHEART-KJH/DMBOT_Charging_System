// robot_ble.h
#ifndef ROBOT_BLE_H
#define ROBOT_BLE_H

#include <ArduinoBLE.h>

bool robotBLE_begin();
void robotBLE_startScan();
bool robotBLE_connectToStation();
void robotBLE_loop();

#endif