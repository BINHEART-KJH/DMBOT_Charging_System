#include <FastLED.h>
#include "station_led.h"
#include "station_fsm.h"

#define RGB_PIN     21
#define NUM_LEDS    10

CRGB leds[NUM_LEDS];
bool blinkState = false;
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 500;  // 0.5초 간격 깜빡임

void led_init() {
  FastLED.addLeds<NEOPIXEL, RGB_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(25); // 초기 10% 밝기
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void led_run() {
  CRGB color;
  StationState state = get_current_state();

  switch (state) {
    case IDLE:
      color = CRGB::White;
      break;

    case DOCKING_OK:
      color = CRGB::Yellow;
      break;

    case ADVERTISING:
      color = CRGB::Blue;
      break;

   case CONNECTING:
  // 초록색 깜빡임
  {
    uint8_t brightness = (sin8(millis() >> 3)); // 빠른 깜빡임
    fill_solid(leds, NUM_LEDS, CRGB::Green);
    FastLED.setBrightness(brightness);
    FastLED.show();
    return;
  }

    case CONNECTED:
      color = CRGB::Green;
      break;

    default:
      color = CRGB::Red;
      break;
  }

  FastLED.setBrightness(25);
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
}
