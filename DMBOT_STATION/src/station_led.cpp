#include "station_led.h"
#include "station_ble.h"
#include "station_gpio.h"
#include <FastLED.h>

#define LED_PIN       21
#define NUM_LEDS      10
#define BRIGHTNESS    128

CRGB leds[NUM_LEDS];

unsigned long lastUpdate = 0;
uint8_t breathPhase = 0;
bool breathUp = true;

void setAll(CRGB color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void breathingColor(CRGB baseColor) {
  unsigned long now = millis();
  if (now - lastUpdate > 30) {
    lastUpdate = now;

    if (breathUp) {
      breathPhase += 5;
      if (breathPhase >= 255) breathUp = false;
    } else {
      if (breathPhase > 5) breathPhase -= 5;
      else breathUp = true;
    }

    CRGB faded = baseColor;
    faded.nscale8_video(breathPhase);
    setAll(faded);
  }
}

void setupLED() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  setAll(CRGB::Black);
}

void updateLED() {
  BLEState state = getBLEState();

  if (!isDocked()) {
    setAll(CRGB(128, 128, 128)); // standby (white dimmed)
  } else if (state == ADVERTISING) {
    breathingColor(CRGB::Blue); // advertising (blue breathing)
  } else if (state == WAIT_AUTH) {
    setAll(CRGB::Blue); // waiting auth (solid blue)
  } else if (state == CONNECTED) {
    if (isRelayOn()) {
      setAll(CRGB::Green); // relay ON
    } else {
      breathingColor(CRGB::Green); // relay 대기 중
    }
  } else {
    setAll(CRGB::Red); // error 또는 fallback
  }
}