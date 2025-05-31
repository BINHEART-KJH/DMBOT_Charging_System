// ================================
// ✅ Station (Peripheral) with RSSI Check + GATT Auth + Docking Reset + LED Blink + Charger State Characteristic + Relay Hold Delay
// ================================
#include <ArduinoBLE.h>

enum BLEState {
  IDLE,
  ADVERTISING,
  WAIT_AUTH,
  CONNECTED
};
BLEState state = IDLE;

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

  if (state == ADVERTISING && (!central || !central.connected())) {
    if (millis() - lastBlinkTime >= 500) {
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
      lastBlinkTime = millis();
    }
  }

  switch (state) {
    case IDLE:
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