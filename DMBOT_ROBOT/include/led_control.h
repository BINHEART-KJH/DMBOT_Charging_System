// ===================== led_coltrol.h =====================
#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <FastLED.h>

#define LED_PIN     21
#define NUM_LEDS    10
#define BRIGHTNESS  128
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

enum LEDState {
  LED_IDLE,
  LED_DOCKED,
  LED_ADVERTISING,
  LED_CONNECTING,
  LED_AUTH_FAIL,
  LED_BLACKLIST,
  LED_AUTH_SUCCESS,
  LED_AUTH_WAITING_RELAY,
  LED_RELAY_ON
};

void setupLED();
void updateLEDState(LEDState state);
void updateLEDEffect();

#endif