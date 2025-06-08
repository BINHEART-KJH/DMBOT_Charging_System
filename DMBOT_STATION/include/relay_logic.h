#pragma once

void setupRelayPins();
void updateRelayState();  // 10초 조건 확인 및 ON/OFF
bool isRelayOn();         // 현재 릴레이 상태 반환