#ifndef STATION_GPIO_H
#define STATION_GPIO_H

#include <Arduino.h>

// GPIO 핀 정의
#define DOCKING_PIN      8
#define BATTERY_FULL_PIN 5
#define CHARGER_OK_PIN   6
#define RELAY_PIN        7
#define BUILTIN_LED      LED_BUILTIN

// GPIO 관련 함수
void gpio_init();
void gpio_run();

#endif