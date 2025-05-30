#include <ArduinoBLE.h>
#include <FastLED.h>

enum BLEState { IDLE, ADVERTISING, WAIT_AUTH, CONNECTED };
BLEState state = IDLE;

const int DOCK_PIN = 8;
const int RELAY_PIN = 7;
const int BATTERY_FULL_PIN = 5;

const int RSSI_THRESHOLD = -100;
const int AUTH_TIMEOUT_MS = 5000;
const unsigned long RELAY_HOLD_MS = 10000;
const char *SECRET_KEY = "DM_System_key";

BLEService authService("180A");
BLECharacteristic nonceChar("2A29", BLERead, 20);
BLECharacteristic tokenChar("2A2A", BLEWrite, 40);
BLECharacteristic chargerStateChar("2A2C", BLERead, 10);

BLEDevice central;
bool gotAuth = false;
String currentNonce = "";
unsigned long connectedTime = 0;
unsigned long relayStartTime = 0;
bool relayOn = false;

#define RGB_PIN 21
#define NUM_LEDS 10
CRGB leds[NUM_LEDS];

#define LED_PIN LED_BUILTIN
bool ledState = false;
unsigned long lastLedBlink = 0;

uint8_t breathBrightness = 0;
int breathDirection = 1;
unsigned long lastBreathUpdate = 0;
const int breathInterval = 8;

bool lastDockState = LOW;

// ===== CRC32 토큰 생성 =====
uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }
  }
  return ~crc;
}

String generateNonce() {
  char buffer[7];
  for (int i = 0; i < 6; i++) buffer[i] = random(33, 126);
  buffer[6] = '\0';
  return String(buffer);
}

String generateToken(const String &nonce, const char *key) {
  String input = nonce + key;
  uint32_t token = crc32((const uint8_t *)input.c_str(), input.length());
  char buf[9];
  sprintf(buf, "%08X", token);
  return String(buf);
}

bool isValidToken(const String &token) {
  return token.equalsIgnoreCase(generateToken(currentNonce, SECRET_KEY));
}

// ===== RGB 제어 =====
void updateBreathingRGB(CRGB baseColor) {
  unsigned long now = millis();
  if (now - lastBreathUpdate >= breathInterval) {
    breathBrightness += breathDirection;
    if (breathBrightness == 0 || breathBrightness == 255) breathDirection *= -1;
    fill_solid(leds, NUM_LEDS, baseColor);
    FastLED.setBrightness(breathBrightness);
    FastLED.show();
    lastBreathUpdate = now;
  }
}

void updateFixedRGB(CRGB color, uint8_t brightness = 255) {
  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void turnOffRGB() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void toggleLED() {
  if (millis() - lastLedBlink >= 1000) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastLedBlink = millis();
  }
}

// ===== BLE 제어 =====
void startAdvertising() {
  BLE.setLocalName("DMBOT-STATION");
  BLE.setAdvertisedService(authService);
  BLE.addService(authService);
  nonceChar.setValue("WAIT");
  BLE.advertise();
  Serial.println("BLE Advertising Start");
}

void stopAdvertising() {
  BLE.stopAdvertise();
  Serial.println("BLE Advertising STOP");
}

void setup() {
  Serial.begin(9600);
  pinMode(DOCK_PIN, INPUT);
  pinMode(BATTERY_FULL_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  FastLED.addLeds<WS2812, RGB_PIN, GRB>(leds, NUM_LEDS);
  turnOffRGB();

  if (!BLE.begin()) {
    Serial.println("BLE init Failed!");
    while (1);
  }

  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  authService.addCharacteristic(chargerStateChar);

  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  }
}

void loop() {
  BLE.poll();

  bool currentDockState = digitalRead(DOCK_PIN);
  if (currentDockState != lastDockState) {
    Serial.print("Docking Check : ");
    Serial.println(currentDockState ? "ON" : "OFF");
    lastDockState = currentDockState;
  }

  switch (state) {
    case IDLE:
      if (currentDockState == HIGH) {
        startAdvertising();
        state = ADVERTISING;
      } else {
        digitalWrite(LED_PIN, LOW);
        updateBreathingRGB(CRGB::Yellow);
      }
      break;

    case ADVERTISING:
      toggleLED();
      updateBreathingRGB(CRGB::Blue);

      if (currentDockState == LOW) {
        stopAdvertising();
        state = IDLE;
      } else {
        central = BLE.central();
        if (central) {
          Serial.print("BLE Connecting: ");
          Serial.println(central.address());
          int rssi = central.rssi();
          Serial.print("RSSI: ");
          Serial.println(rssi);
          if (rssi < RSSI_THRESHOLD) {
            Serial.println("RSSI LOW");
            central.disconnect();
            break;
          }

          currentNonce = generateNonce();
          nonceChar.setValue(currentNonce.c_str());
          gotAuth = false;
          connectedTime = millis();
          state = WAIT_AUTH;
        }
      }
      break;

    case WAIT_AUTH:
      toggleLED();
      updateBreathingRGB(CRGB::Green);

      if (currentDockState == LOW) {
        Serial.println("Docking Check failed, disconnected");
        if (central.connected()) central.disconnect();
        stopAdvertising();
        state = IDLE;
        break;
      }

      if (!central.connected()) {
        Serial.println("disconnected before Auth");
        stopAdvertising();
        state = IDLE;
        break;
      }

      if (tokenChar.written()) {
        String token = String((const char *)tokenChar.value(), tokenChar.valueLength());
        if (isValidToken(token)) {
          Serial.println("Auth Successe, Connected");
          gotAuth = true;
          digitalWrite(LED_PIN, HIGH);
          state = CONNECTED;
        } else {
          Serial.println("Auth failed, disconnected");
          central.disconnect();
          stopAdvertising();
          state = IDLE;
        }
      } else if (millis() - connectedTime > AUTH_TIMEOUT_MS) {
        Serial.println("Auth timeout, disconnected");
        central.disconnect();
        stopAdvertising();
        state = IDLE;
      }
      break;

    case CONNECTED:
      updateFixedRGB(CRGB::Green);
      digitalWrite(LED_PIN, HIGH);

      if (!central.connected() || currentDockState == LOW) {
        Serial.println("BLE Disconnected");
        digitalWrite(RELAY_PIN, LOW);
        chargerStateChar.setValue("0");
        relayOn = false;
        relayStartTime = 0;
        stopAdvertising();
        state = IDLE;
        break;
      }

      bool batteryFull = digitalRead(BATTERY_FULL_PIN);
      if (batteryFull) {
        // Battery Full일 땐 무조건 릴레이 OFF
        if (relayOn) {
          Serial.println("Battery_Full - Relay OFF");
          digitalWrite(RELAY_PIN, LOW);
          chargerStateChar.setValue("0");
          relayOn = false;
          relayStartTime = 0;
        }
      } else {
        if (!relayOn) {
          if (relayStartTime == 0) {
            relayStartTime = millis();
          } else if (millis() - relayStartTime >= RELAY_HOLD_MS) {
            Serial.println("Relay ON");
            digitalWrite(RELAY_PIN, HIGH);
            chargerStateChar.setValue("1");
            relayOn = true;
          }
        }
      }
      break;
  }
}