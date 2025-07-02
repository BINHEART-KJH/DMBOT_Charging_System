#ifndef ROBOT_BLE_H
#define ROBOT_BLE_H

void setupBLE();
void startBLEScan();
void updateBLEState();
bool isBLEConnected();
bool isChargerStateOn();

#endif