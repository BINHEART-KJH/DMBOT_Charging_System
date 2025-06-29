#ifndef ROBOT_GPIO_H
#define ROBOT_GPIO_H

#include <Arduino.h>

void setupGPIO();
void updateGPIO();              // ✅ 이 줄이 있어야 합니다
bool isRelayOn();
void forceRelayState(bool state);  // RS485 강제 제어용

#endif