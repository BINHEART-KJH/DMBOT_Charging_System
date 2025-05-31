// ================================
// ✅ Station (Peripheral) + FastLED 구조 병합 (LED 제어는 아직 미사용)
// ================================
#include <ArduinoBLE.h>
#include <FastLED.h>

enum BLEState {
  IDLE,
  ADVERTISING,
  WAIT_AUTH,
  CONNECTED
};
BLEState state = IDLE;

// -------- FastLED 설정 --------
#define DATA_PIN    21
#define NUM_LEDS    10
CRGB leds[NUM_LEDS];

const CRGB colorMap[16] = {
  CRGB::Black,
  CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue,
  CRGB(75,0,130), CRGB(148,0,211), CRGB::Cyan, CRGB::Magenta,
  CRGB::Pink, CRGB::White, CRGB::Lime, CRGB::Teal, CRGB(255,191,0),
  CRGB::Black
};

const char* colorNames[16] = {
  "Off","Red","Orange","Yellow","Green","Blue",
  "Indigo","Violet","Cyan","Magenta",
  "Pink","White","Lime","Teal","Amber","Rainbow"
};

uint8_t currentColorIdx   = 1;   // 기본 Red
uint8_t brightnessPercent = 50;  // 기본 50%

// ---------------------------------

const int DOCKING_CHECK_PIN = 8;
const int RELAY_PIN = 7;
const int LED_PIN = LED_BUILTIN;

const char* SECRET_KEY = "DM_System_key";
const unsigned long AUTH_TIMEOUT_MS = 5000;
const unsigned long RELAY_HOLD_DELAY_MS = 10000;

BLEDevice central;
String currentNonce = "";
bool gotAuth = false;
unsigned long authStartTime = 0;
unsigned long connectedTime = 0;
bool relayOn = false;

unsigned long lastBlinkTime = 0;
bool ledBlinkState = false;

BLEService authService("180A");
BLECharacteristic nonceChar("2A29", BLERead, 20);
BLECharacteristic tokenChar("2A2A", BLEWrite, 40);
BLECharacteristic chargerStateChar("2A2C", BLERead, 10);

// 함수: 색상 및 밝기 설정 (아직 호출 X)
void setColorBrightness(uint8_t idx, uint8_t briPct) {
  uint8_t b = map(briPct, 0, 100, 0, 255);
  FastLED.setBrightness(b);
  if (idx >= 1 && idx <= 14) {
    fill_solid(leds, NUM_LEDS, colorMap[idx]);
  } else if (idx == 15) {
    fill_rainbow(leds, NUM_LEDS, 0, 255 / NUM_LEDS);
  } else {
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  }
  FastLED.show();
}

void breathingEffect(uint8_t colorIdx, unsigned long intervalMs = 2500) {
  static unsigned long breathStart = 0;
  float progress = (millis() - breathStart) / (float)intervalMs * 2.0 * PI;  // 0~2π
  uint8_t b = (sin(progress) + 1.0) * 127.5;  // 0~255

  FastLED.setBrightness(b);
  if (colorIdx >= 1 && colorIdx <= 14)
    fill_solid(leds, NUM_LEDS, colorMap[colorIdx]);
  else if (colorIdx == 15)
    fill_rainbow(leds, NUM_LEDS, 0, 255 / NUM_LEDS);
  else
    fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void updateLEDStatus(BLEState state) {
  static unsigned long prevMillis = 0;
  static bool blinkState = false;

  switch (state) {
    case IDLE:
      breathingEffect(3);  // Yellow
      break;

    case ADVERTISING:
      breathingEffect(5);  // Blue
      break;

    case WAIT_AUTH:
      if (millis() - prevMillis >= 300) {
        blinkState = !blinkState;
        FastLED.setBrightness(blinkState ? 255 : 0);
        fill_solid(leds, NUM_LEDS, colorMap[5]);  // Blue
        FastLED.show();
        prevMillis = millis();
      }
      break;

    case CONNECTED:
      FastLED.setBrightness(255);
      fill_solid(leds, NUM_LEDS, colorMap[4]);  // Green
      FastLED.show();
      break;
  }
}

void generateNonce();
uint32_t calculateCRC32(const String& data);

void resetBLE() {
  BLE.stopAdvertise();
  if (central && central.connected()) {
    central.disconnect();
    Serial.println("🔌 Central disconnected due to docking off.");
  }
  BLE.end();
  delay(100);
  if (!BLE.begin()) {
    Serial.println("❌ BLE re-init failed!");
    while (1);
  }
  BLE.setLocalName("DM-STATION");
  BLE.setDeviceName("DM-STATION");
  BLE.setAdvertisedService(authService);
  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  authService.addCharacteristic(chargerStateChar);
  BLE.addService(authService);
  nonceChar.writeValue("waiting");
  chargerStateChar.writeValue("OFF");
  state = IDLE;
  gotAuth = false;
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  relayOn = false;
  Serial.println("🔄 BLE reset complete.");
}

void setup() {
  pinMode(DOCKING_CHECK_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);

  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
    while (1);
  }

  BLE.setLocalName("DM-STATION");
  BLE.setDeviceName("DM-STATION");
  BLE.setAdvertisedService(authService);
  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  authService.addCharacteristic(chargerStateChar);
  BLE.addService(authService);
  nonceChar.writeValue("waiting");
  chargerStateChar.writeValue("OFF");

  tokenChar.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic characteristic) {
    int len = characteristic.valueLength();
    String receivedToken = "";
    for (int i = 0; i < len; i++) {
      receivedToken += (char)characteristic.value()[i];
    }
    String data = SECRET_KEY + currentNonce;
    uint32_t expectedCRC = calculateCRC32(data);
    uint32_t receivedCRC = strtoul(receivedToken.c_str(), NULL, 16);

    if (receivedCRC == expectedCRC) {
      gotAuth = true;
      Serial.println("✅ Auth success");
    } else {
      Serial.println("❌ Auth failed. Disconnecting...");
      central.disconnect();
    }
  });
}

void loop() {
  BLE.poll();
  bool docking = digitalRead(DOCKING_CHECK_PIN);

  if (docking == LOW && state != IDLE) {
    Serial.println("⚠️ Docking check LOW. Forcing BLE reset.");
    resetBLE();
    return;
  }

    updateLEDStatus(state);  // ✅ 상태에 따라 LED 처리


  if (state == ADVERTISING && (!central || !central.connected())) {
    if (millis() - lastBlinkTime >= 500) {
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
      lastBlinkTime = millis();
    }
  }

  switch (state) {
    case IDLE:
        breathingEffect(currentColorIdx);  // 🌬️ 숨쉬기 효과 한 줄로 표현
      if (docking == HIGH) {
        BLE.advertise();
        state = ADVERTISING;
        Serial.println("📡 Advertising started");
      }
      break;

    case ADVERTISING:
      central = BLE.central();
      if (central) {
        int rssi = central.rssi();
        Serial.print("🔗 Central connected | RSSI: ");
        Serial.println(rssi);

        if (rssi < -70) {
          Serial.println("📉 RSSI too weak. Disconnecting...");
          central.disconnect();
          return;
        }

        Serial.println("🔐 Waiting for auth...");
        generateNonce();
        nonceChar.writeValue(currentNonce.c_str());
        chargerStateChar.writeValue("OFF");
        gotAuth = false;
        authStartTime = millis();
        digitalWrite(LED_PIN, LOW);
        state = WAIT_AUTH;
      }
      break;

    case WAIT_AUTH:
      if (!central.connected()) {
        Serial.println("❌ Disconnected during auth");
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        chargerStateChar.writeValue("OFF");
        relayOn = false;
        state = ADVERTISING;
        break;
      }

      if (gotAuth) {
        Serial.println("🔒 Authenticated. Connection secured.");
        connectedTime = millis();
        digitalWrite(LED_PIN, HIGH);
        state = CONNECTED;
      } else if (millis() - authStartTime > AUTH_TIMEOUT_MS) {
        Serial.println("⏱ Auth timeout. Disconnecting.");
        central.disconnect();
        state = ADVERTISING;
      }
      break;

    case CONNECTED:
      if (!central.connected()) {
        Serial.println("🔌 Disconnected.");
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        chargerStateChar.writeValue("OFF");
        relayOn = false;
        state = ADVERTISING;
      } else {
        if (!relayOn && (millis() - connectedTime >= RELAY_HOLD_DELAY_MS)) {
          digitalWrite(RELAY_PIN, HIGH);
          chargerStateChar.writeValue("ON");
          relayOn = true;
          Serial.println("⚡ Relay ON after 10s hold");
        }
      }
      break;
  }
}

void generateNonce() {
  currentNonce = "";
  for (int i = 0; i < 8; i++) {
    char c = random(0, 16);
    currentNonce += String(c, HEX);
  }
  Serial.print("🔑 Nonce generated: ");
  Serial.println(currentNonce);
}

uint32_t calculateCRC32(const String& data) {
  const uint32_t polynomial = 0xEDB88320;
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < data.length(); ++i) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (int j = 0; j < 8; ++j) {
      if (crc & 1) crc = (crc >> 1) ^ polynomial;
      else crc >>= 1;
    }
  }
  return ~crc;
}