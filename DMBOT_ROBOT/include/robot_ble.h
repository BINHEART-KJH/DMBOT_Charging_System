#ifndef ROBOT_BLE_H
#define ROBOT_BLE_H

void ble_init();
void ble_run();
void ble_reset();

// RS485 보고용 상태 Getter 함수
// (IMPROVED) volatile로 선언된 동기화 변수를 통해 Race Condition 방지
bool getBleConnectionState();       // CONNECTED 상태 && peripheral.connected()
bool getBatteryFullStatus();
bool getChargerOkStatus();
bool getChargerRelayStatus();
bool getDockingStatus();

// (NEW) BLE 상태 진단용
void ble_print_status();

#endif
