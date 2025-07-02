#ifndef STATION_GPIO_H
#define STATION_GPIO_H

#include <Arduino.h>

// 핀 정의
#define DOCKING_PIN   8   // D8 - 입력
#define RELAY_PIN     7   // D7 - 출력
#define LED_PIN       LED_BUILTIN  // 내장 LED

void relay_set(bool on);

// 초기화
void stationGPIO_init();

// 업데이트 (디바운싱 처리 등)
void stationGPIO_update();

// 상태 확인
bool isDockingPinHigh();

// 릴레이 제어
void setRelayState(bool on);

// LED 제어
void setLED(bool on);

#endif