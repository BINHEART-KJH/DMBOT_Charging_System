#ifndef STATION_GPIO_H
#define STATION_GPIO_H

void setupGPIO();
void updateGPIO();

// 외부에서 상태 조회
bool isDocked();
bool isChargerOK();
bool isBatteryFull();
bool isRelayOn();

#endif