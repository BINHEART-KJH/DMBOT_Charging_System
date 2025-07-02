#ifndef ROBOT_RS485_H
#define ROBOT_RS485_H

void robotRS485_init();
void robotRS485_update();
void robotRS485_sendBLEStatus(bool connected);

#endif