#pragma once
#include <ArduinoBLE.h>

void setupBLE();
void resetBLEState();
void generateNonce(char* outNonce);
bool validateToken(const char* mac, const char* token);
bool isConnected();
const char* getConnectedMac();
void updateBLEStateMachine();  // 상태머신에 대응한 BLE 처리