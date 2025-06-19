// ===================== led_control.cpp =====================
#include "led_control.h"

CRGB leds[NUM_LEDS];
LEDState currentLEDState = LED_IDLE;
unsigned long lastEffectUpdate = 0;
uint8_t breathBrightness = 0;
bool breathDirectionUp = true;

void setupLED() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  fill_solid(leds, NUM_LEDS, CRGB::White);
  FastLED.show();
}

void updateLEDState(LEDState state) {
  if (state == currentLEDState) return;
  currentLEDState = state;

  switch (state) {
    case LED_IDLE:
      FastLED.setBrightness(BRIGHTNESS / 2);
      fill_solid(leds, NUM_LEDS, CRGB::White);
      break;
    case LED_DOCKED:
      FastLED.setBrightness(BRIGHTNESS);
      fill_solid(leds, NUM_LEDS, CRGB::Yellow);
      break;
    case LED_ADVERTISING:
    case LED_AUTH_WAITING_RELAY:
      // 숨쉬기 효과 처리용. 색상만 지정해둠
      fill_solid(leds, NUM_LEDS, state == LED_ADVERTISING ? CRGB::Blue : CRGB::Green);
      break;
    case LED_CONNECTING:
      FastLED.setBrightness(BRIGHTNESS);
      fill_solid(leds, NUM_LEDS, CRGB::Blue);
      break;
    case LED_AUTH_FAIL:
    case LED_BLACKLIST:
      FastLED.setBrightness(BRIGHTNESS);
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      break;
    case LED_AUTH_SUCCESS:
    case LED_RELAY_ON:
      FastLED.setBrightness(BRIGHTNESS);
      fill_solid(leds, NUM_LEDS, CRGB::Green);
      break;
  }
  FastLED.show();
}

void updateLEDEffect() {
  if (currentLEDState != LED_ADVERTISING && currentLEDState != LED_AUTH_WAITING_RELAY)
    return;

  unsigned long now = millis();
  if (now - lastEffectUpdate < 30) return; // 약 30ms 간격으로 업데이트
  lastEffectUpdate = now;

  // 숨쉬기 효과
  if (breathDirectionUp) {
    breathBrightness += 5;
    if (breathBrightness >= BRIGHTNESS) breathDirectionUp = false;
  } else {
    if (breathBrightness >= 5) breathBrightness -= 5;
    else breathDirectionUp = true;
  }
  FastLED.setBrightness(breathBrightness);
  FastLED.show();
}