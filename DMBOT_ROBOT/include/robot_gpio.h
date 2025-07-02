#ifndef ROBOT_GPIO_H
#define ROBOT_GPIO_H

#define BATTERY_RELAY_PIN 4  // 릴레이 연결된 D4 핀

void gpio_init();
void setRelay(bool on);
bool getRelayState();

#endif