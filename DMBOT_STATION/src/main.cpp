#include <ArduinoBLE.h>
#include <FastLED.h>

enum BLEState
{
  IDLE,
  ADVERTISING,
  WAIT_AUTH,
  CONNECTED
};
BLEState state = IDLE;

const int DOCK_PIN = 8;
const int RELAY_PIN = 7;

const int RSSI_THRESHOLD = -500;
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

// ===== CRC32 토큰 생성 =====
uint32_t crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
    {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }
  }
  return ~crc;
}

String generateNonce()
{
  char buffer[7];
  for (int i = 0; i < 6; i++)
    buffer[i] = random(33, 126);
  buffer[6] = '\0';
  return String(buffer);
}

String generateToken(const String &nonce, const char *key)
{
  String input = nonce + key;
  uint32_t token = crc32((const uint8_t *)input.c_str(), input.length());
  char buf[9];
  sprintf(buf, "%08X", token);
  return String(buf);
}

bool isValidToken(const String &token)
{
  return token.equalsIgnoreCase(generateToken(currentNonce, SECRET_KEY));
}

// ===== RGB LED 제어 =====
void updateBreathingRGB(CRGB baseColor)
{
  unsigned long now = millis();
  if (now - lastBreathUpdate >= breathInterval)
  {
    breathBrightness += breathDirection;
    if (breathBrightness == 0 || breathBrightness == 255)
      breathDirection *= -1;
    fill_solid(leds, NUM_LEDS, baseColor);
    FastLED.setBrightness(breathBrightness);
    FastLED.show();
    lastBreathUpdate = now;
  }
}

void updateFixedRGB(CRGB color, uint8_t brightness = 255)
{
  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void turnOffRGB()
{
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void toggleLED()
{
  if (millis() - lastLedBlink >= 1000)
  {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastLedBlink = millis();
  }
}

// ===== BLE 제어 =====
void startAdvertising()
{
  BLE.setLocalName("DMBOT-STATION");
  BLE.setAdvertisedService(authService);
  BLE.addService(authService);
  nonceChar.setValue("WAIT");
  BLE.advertise();
  Serial.println("📡 Advertising started");
}

void stopAdvertising()
{
  BLE.stopAdvertise();
  Serial.println("🛑 Advertising stopped");
}

// ===== SETUP =====
void setup()
{
  Serial.begin(9600);
  pinMode(DOCK_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  FastLED.addLeds<WS2812, RGB_PIN, GRB>(leds, NUM_LEDS);
  turnOffRGB();

  if (!BLE.begin())
  {
    Serial.println("BLE init failed");
    while (1)
      ;
  }

  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  authService.addCharacteristic(chargerStateChar);

  if (digitalRead(DOCK_PIN) == HIGH)
  {
    startAdvertising();
    state = ADVERTISING;
  }
}

// ===== LOOP =====
void loop()
{
  BLE.poll();

  switch (state)
  {
  case IDLE:
    if (digitalRead(DOCK_PIN) == HIGH)
    {
      startAdvertising();
      state = ADVERTISING;
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
      updateBreathingRGB(CRGB::Yellow);
    }
    break;

  case ADVERTISING:
    toggleLED();
    updateBreathingRGB(CRGB::Blue);

    if (digitalRead(DOCK_PIN) == LOW)
    {
      stopAdvertising();
      state = IDLE;
    }
    else
    {
      central = BLE.central();
      if (central)
      {
        Serial.print("🔌 Connected to: ");
        Serial.println(central.address());
        int rssi = central.rssi();
        Serial.print("📶 RSSI: ");
        Serial.println(rssi);
        if (rssi < RSSI_THRESHOLD)
        {
          Serial.println("❌ RSSI too low, disconnecting");
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

    if (digitalRead(DOCK_PIN) == LOW)
    {
      Serial.println("⚠️ Docking OFF → Disconnect");
      if (central.connected())
        central.disconnect();
      stopAdvertising();
      state = IDLE;
      break;
    }

    if (!central.connected())
    {
      Serial.println("❌ Disconnected before auth");
      stopAdvertising();
      state = IDLE;
      break;
    }

    if (tokenChar.written())
    {
      String token = String((const char *)tokenChar.value(), tokenChar.valueLength());
      if (isValidToken(token))
      {
        Serial.println("✅ Auth Success");
        gotAuth = true;
        digitalWrite(LED_PIN, HIGH);
        state = CONNECTED;
      }
      else
      {
        Serial.println("❌ Auth Failed → Disconnect");
        central.disconnect();
        stopAdvertising();
        state = IDLE;
      }
    }
    else if (millis() - connectedTime > AUTH_TIMEOUT_MS)
    {
      Serial.println("⏳ Auth timeout → Disconnect");
      central.disconnect();
      stopAdvertising();
      state = IDLE;
    }
    break;

  case CONNECTED:
    updateFixedRGB(CRGB::Green);
    digitalWrite(LED_PIN, HIGH);

    if (!central.connected() || digitalRead(DOCK_PIN) == LOW)
    {
      Serial.println("🔌 Disconnected");
      digitalWrite(RELAY_PIN, LOW);
      chargerStateChar.setValue("0");
      relayOn = false;
      relayStartTime = 0;
      stopAdvertising();
      state = IDLE;
      break;
    }

    // ✅ Docking 유지되기만 하면 10초 후 릴레이 ON
    if (!relayOn)
    {
      if (relayStartTime == 0)
      {
        relayStartTime = millis();
      }
      else if (millis() - relayStartTime >= RELAY_HOLD_MS)
      {
        digitalWrite(RELAY_PIN, HIGH);
        chargerStateChar.setValue("1");
        relayOn = true;
      }
    }
    break;
  }
}
  
/*
  #include <ArduinoBLE.h>
#include <FastLED.h>

enum BLEState { IDLE, ADVERTISING, WAIT_AUTH, CONNECTED };
BLEState state = IDLE;

const int DOCK_PIN = 8;
const int BATTERY_FULL_PIN = 5;
const int RELAY_PIN = 7;

const int RSSI_THRESHOLD = -500;
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

uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
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

void updateBreathingRGB(CRGB baseColor) {
  unsigned long now = millis();
  if (now - lastBreathUpdate >= breathInterval) {
    breathBrightness += breathDirection;
    if (breathBrightness == 0 || breathBrightness == 255)
      breathDirection *= -1;
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

void startAdvertising() {
  BLE.setLocalName("DMBOT-STATION");
  BLE.setAdvertisedService(authService);
  BLE.addService(authService);
  nonceChar.setValue("WAIT");
  BLE.advertise();
  Serial.println("📡 Advertising started");
}

void stopAdvertising() {
  BLE.stopAdvertise();
  Serial.println("🛑 Advertising stopped");
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
    Serial.println("BLE init failed");
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

  switch (state) {
    case IDLE:
      if (digitalRead(DOCK_PIN) == HIGH) {
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

      if (digitalRead(DOCK_PIN) == LOW) {
        stopAdvertising();
        state = IDLE;
      } else {
        central = BLE.central();
        if (central) {
          Serial.print("🔌 Connected to: ");
          Serial.println(central.address());
          int rssi = central.rssi();
          Serial.print("📶 RSSI: ");
          Serial.println(rssi);
          if (rssi < RSSI_THRESHOLD) {
            Serial.println("❌ RSSI too low, disconnecting");
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

      if (digitalRead(DOCK_PIN) == LOW) {
        Serial.println("⚠️ Docking OFF → Disconnect");
        if (central.connected()) central.disconnect();
        stopAdvertising();
        state = IDLE;
        break;
      }

      if (!central.connected()) {
        Serial.println("❌ Disconnected before auth");
        stopAdvertising();
        state = IDLE;
        break;
      }

      if (tokenChar.written()) {
        String token = String((const char *)tokenChar.value(), tokenChar.valueLength());
        if (isValidToken(token)) {
          Serial.println("✅ Auth Success");
          gotAuth = true;
          digitalWrite(LED_PIN, HIGH);
          state = CONNECTED;
        } else {
          Serial.println("❌ Auth Failed → Disconnect");
          central.disconnect();
          stopAdvertising();
          state = IDLE;
        }
      } else if (millis() - connectedTime > AUTH_TIMEOUT_MS) {
        Serial.println("⏳ Auth timeout → Disconnect");
        central.disconnect();
        stopAdvertising();
        state = IDLE;
      }
      break;

    case CONNECTED:
      updateFixedRGB(CRGB::Green);
      digitalWrite(LED_PIN, HIGH);

      if (!central.connected() || digitalRead(DOCK_PIN) == LOW) {
        Serial.println("🔌 Disconnected");
        digitalWrite(RELAY_PIN, LOW);
        chargerStateChar.setValue("0");
        relayOn = false;
        relayStartTime = 0;
        stopAdvertising();
        state = IDLE;
        break;
      }

      bool batteryFull = digitalRead(BATTERY_FULL_PIN) == HIGH;

      if (!batteryFull) {
        if (!relayOn) {
          if (relayStartTime == 0) {
            relayStartTime = millis();
          } else if (millis() - relayStartTime >= RELAY_HOLD_MS) {
            digitalWrite(RELAY_PIN, HIGH);
            chargerStateChar.setValue("1");
            relayOn = true;
          }
        }
      } else {
        relayStartTime = 0;
        if (relayOn) {
          digitalWrite(RELAY_PIN, LOW);
          chargerStateChar.setValue("0");
          relayOn = false;
        }
      }
      break;
  }
}*/