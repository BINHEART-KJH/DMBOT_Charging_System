#ifndef STATION_BLE_H
#define STATION_BLE_H

#include <ArduinoBLE.h>

extern bool isAdvertising;
extern char nonce[9];         // 인증에 사용되는 nonce
extern bool authSuccess;      // 인증 성공 여부

void ble_init();
void ble_run();
void ble_reset();

#endif