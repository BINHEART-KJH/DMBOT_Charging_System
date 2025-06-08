#pragma once

void setupBLEScanner();
void startScan();
void stopScan();
void updateBLEScanLoop();  // loop()에서 호출
void disconnectFromStation();
bool isConnectedToStation();
void disconnectFromStation();