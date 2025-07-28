#ifndef ROBOT_BLE_H
#define ROBOT_BLE_H

void ble_init();
void ble_run();
void ble_reset();

// RS485 보고용 상태 Getter 함수
bool getBleConnectionState();
bool getBatteryFullStatus();
bool getChargerOkStatus();
bool getChargerRelayStatus();
bool getDockingStatus();
#endif