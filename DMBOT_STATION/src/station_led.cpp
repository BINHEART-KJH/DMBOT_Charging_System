#include "station_led.h"
#include <FastLED.h>

// 설정
#define LED_PIN     21      // D21
#define NUM_LEDS    10
#define BRIGHTNESS  100

CRGB leds[NUM_LEDS];
StationState currentLedState = ST_IDLE;
unsigned long ledTimer = 0;
bool ledBlinkState = false;

void ledStatus_init() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  setLedState(ST_IDLE);
}

void setAllLeds(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void setLedState(StationState state) {
  currentLedState = state;
  ledTimer = millis(); // 리셋 타이머

  switch (state) {
    case ST_IDLE:
      setAllLeds(CRGB::White); // 대기 상태
      break;

    case ST_ADVERTISING:
      // 숨쉬기 효과를 위해 업데이트에서 처리
      break;

    case ST_CONNECTED:
      setAllLeds(CRGB::Green); // 인증 성공
      break;

    case ST_DOCKED:
      setAllLeds(CRGB::Yellow);
      break;

    case ST_ERROR:
      // 깜빡임 효과를 위해 업데이트에서 처리
      break;

    default:
      setAllLeds(CRGB::Black);
      break;
  }
}

void ledStatus_update() {
  unsigned long now = millis();

  if (currentLedState == ST_ADVERTISING) {
    // 숨쉬기 블루
    uint8_t brightness = (sin(now / 750.0) + 1.0) * 127; // 0~255
    fill_solid(leds, NUM_LEDS, CHSV(160, 255, brightness));
    FastLED.show();
  }

  else if (currentLedState == ST_ERROR) {
    // 500ms 간격으로 빨간색 깜빡임
    if (now - ledTimer >= 500) {
      ledBlinkState = !ledBlinkState;
      ledTimer = now;
      setAllLeds(ledBlinkState ? CRGB::Red : CRGB::Black);
    }
  }
}
