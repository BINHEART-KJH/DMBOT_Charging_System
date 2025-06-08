#pragma once

enum LedMode {
  LED_OFF,
  LED_BREATHING,
  LED_BLINKING,
  LED_SOLID
};

void setupLED();
void setLEDMode(LedMode mode);
void updateLED();  // 매 loop()에서 호출하여 상태 반영