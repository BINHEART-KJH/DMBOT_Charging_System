#include <ArduinoBLE.h>
#include <FastLED.h>
#include <string>

enum BLEState { IDLE, ADVERTISING, WAIT_AUTH, CONNECTED };
BLEState state = IDLE;

const int DOCK_PIN = 8;
const int RSSI_THRESHOLD = -50;
const int AUTH_TIMEOUT_MS = 5000;
const char *SECRET_KEY = "DM_System_key";

BLEService authService("180A");
BLECharacteristic nonceChar("2A29", BLERead, 20);
BLECharacteristic tokenChar("2A2A", BLEWrite, 40);
String currentNonce = "";

BLEDevice central;
bool gotAuth = false;
unsigned long connectedTime = 0;

// ===== LED =====
#define LED_PIN    LED_BUILTIN
#define RGB_PIN    21
#define NUM_LEDS   10
CRGB leds[NUM_LEDS];

// 숨쉬기 애니메이션 변수
uint8_t breathBrightness = 0;
int breathDirection = 1;
unsigned long lastBreathUpdate = 0;
const int breathDelay = 8;  // 숨쉬기 속도 더 빠르게 조정

// ===== 유틸 =====
uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
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
  String input = nonce + String(key);
  uint32_t token = crc32((const uint8_t *)input.c_str(), input.length());
  char buf[9];
  sprintf(buf, "%08X", token);
  return String(buf);
}

bool isValidToken(const String &token) {
  return token.equalsIgnoreCase(generateToken(currentNonce, SECRET_KEY));
}

void updateBreathEffect(CRGB color) {
  unsigned long now = millis();
  if (now - lastBreathUpdate >= breathDelay) {
    breathBrightness += breathDirection;
    if (breathBrightness == 0 || breathBrightness == 255)
      breathDirection *= -1;
    fill_solid(leds, NUM_LEDS, color);
    FastLED.setBrightness(breathBrightness);
    FastLED.show();
    lastBreathUpdate = now;
  }
}

void turnRGB(CRGB color, uint8_t brightness) {
  fill_solid(leds, NUM_LEDS, color);
  FastLED.setBrightness(brightness);
  FastLED.show();
}

void turnOffRGB() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

// ===== BLE 흐름 =====
void startAdvertising() {
  BLE.setLocalName("DMBOT-STATION");
  BLE.setAdvertisedService(authService);
  BLE.addService(authService);
  nonceChar.setValue("WAIT");
  BLE.advertise();
  Serial.println("Station advertising...");
}

void stopAdvertising() {
  BLE.stopAdvertise();
  Serial.println("Advertising stopped");
}

void handleIdleState() {
  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  } else {
    // 광고 OFF 상태에서 노란색 숨쉬기 효과
    updateBreathEffect(CRGB::Yellow);
  }
}

void handleAdvertisingState() {
  updateBreathEffect(CRGB::Blue);
  if (digitalRead(DOCK_PIN) == LOW) {
    stopAdvertising();
    state = IDLE;
    return;
  }
  BLEDevice incoming = BLE.central();
  if (incoming) {
    Serial.print("Connected to: ");
    Serial.println(incoming.address());

    if (incoming.rssi() < RSSI_THRESHOLD) {
      Serial.println("RSSI too low, disconnecting");
      incoming.disconnect();
      return;
    }

    central = incoming;
    currentNonce = generateNonce();
    nonceChar.setValue(currentNonce.c_str());

    gotAuth = false;
    connectedTime = millis();
    state = WAIT_AUTH;
  }
}

void handleWaitAuthState() {
  updateBreathEffect(CRGB::Green);

  if (!central.connected()) {
    Serial.println("Disconnected before auth");
    stopAdvertising();
    state = IDLE;
    return;
  }

  if (tokenChar.written()) {
    String token = String((const char *)tokenChar.value(), tokenChar.valueLength());
    if (isValidToken(token)) {
      Serial.println("Auth Success");
      gotAuth = true;
      state = CONNECTED;
    } else {
      Serial.println("Auth Failed → Disconnect");
      central.disconnect();
      stopAdvertising();
      state = IDLE;
    }
    return;
  }

  if (millis() - connectedTime > AUTH_TIMEOUT_MS) {
    Serial.println("Auth timeout → Disconnect");
    central.disconnect();
    stopAdvertising();
    state = IDLE;
  }
}

void handleConnectedState() {
  turnRGB(CRGB::Green, 255); // 초록색 100% ON
  if (!central.connected()) {
    Serial.println("Disconnected");
    stopAdvertising();
    state = IDLE;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(DOCK_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  FastLED.addLeds<WS2812, RGB_PIN, GRB>(leds, NUM_LEDS);
  turnOffRGB();

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }

  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);

  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  }
}

void loop() {
  switch (state) {
    case IDLE:
      handleIdleState();
      break;
    case ADVERTISING:
      handleAdvertisingState();
      break;
    case WAIT_AUTH:
      handleWaitAuthState();
      break;
    case CONNECTED:
      handleConnectedState();
      break;
  }
}