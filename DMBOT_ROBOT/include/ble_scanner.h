#pragma once

#include <Arduino.h>

void setupBLEScanner();
void startScan();
void stopScan();
void updateBLEScanLoop();  // loop()에서 호출
void disconnectFromStation();
bool isConnectedToStation();

void setupRelayControl();
void processSerialCommand(const char* cmd);