// ================================
// ✅ Robot (Central) with GATT Discovery Wait + Battery_ready Relay Control
// ================================
#include <ArduinoBLE.h>

const char* targetLocalName = "DM-STATION";
const int RSSI_THRESHOLD = -70;
const int LED_PIN = LED_BUILTIN;
const int BATTERY_READY_PIN = 4;  // D4
const char* SECRET_KEY = "DM_System_key";

bool isConnected = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;
BLEDevice connectedPeripheral;

uint32_t calculateCRC32(const String& data);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY_READY_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BATTERY_READY_PIN, LOW);

  Serial.begin(115200);
  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
    while (1);
  }
  BLE.scan();
  Serial.println("🔍 Scanning started...");
}

void loop() {
  if (!isConnected) {
    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
    }

    BLEDevice peripheral = BLE.available();
    if (peripheral) {
      int rssi = peripheral.rssi();
      String name = peripheral.localName();

      Serial.print("📡 Found device - Name: ");
      Serial.print(name);
      Serial.print(" | RSSI: ");
      Serial.println(rssi);

      if (name != targetLocalName || rssi < RSSI_THRESHOLD) {
        Serial.println("⛔ Not target or weak RSSI. Ignored.");
        return;
      }

      BLE.stopScan();
      Serial.println("🔗 Connecting to Station...");

      if (peripheral.connect()) {
        Serial.println("✅ Connected to Station!");
        connectedPeripheral = peripheral;
        isConnected = true;
        digitalWrite(LED_PIN, HIGH);

        // GATT 서비스 탐색 안정화
        Serial.println("🔍 Discovering attributes...");
        bool found = false;
        for (int i = 0; i < 20; i++) {
          if (peripheral.discoverAttributes()) {
            found = true;
            break;
          }
          BLE.poll();
          delay(50);
        }
        if (!found) {
          Serial.println("❌ Failed to discover attributes. Disconnecting.");
          peripheral.disconnect();
          isConnected = false;
          BLE.scan();
          return;
        }

        BLECharacteristic nonceChar = peripheral.characteristic("2A29");
        BLECharacteristic tokenChar = peripheral.characteristic("2A2A");

        if (!nonceChar || !nonceChar.canRead() || !nonceChar.read()) {
          Serial.println("❌ Failed to read nonce.");
          peripheral.disconnect();
          isConnected = false;
          BLE.scan();
          return;
        }

        String nonce = "";
        int len = nonceChar.valueLength();
        for (int i = 0; i < len; i++) {
          nonce += (char)nonceChar.value()[i];
        }

        Serial.print("🔑 Nonce received: ");
        Serial.println(nonce);

        String data = String(SECRET_KEY) + nonce;
        uint32_t crc = calculateCRC32(data);
        char token[12];
        sprintf(token, "%08lX", crc);

        Serial.print("🧾 Sending token: ");
        Serial.println(token);
        tokenChar.writeValue((const uint8_t*)token, strlen(token));
      } else {
        Serial.println("❌ Connection failed.");
        BLE.scan();
      }
    }
  } else {
    if (!connectedPeripheral.connected()) {
      Serial.println("🔌 Disconnected.");
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BATTERY_READY_PIN, LOW);
      isConnected = false;
      BLE.scan();
    } else {
      // 연결 유지 시 Battery_jumper 상태 체크
      BLECharacteristic chargerChar = connectedPeripheral.characteristic("2A2C");
      if (chargerChar && chargerChar.canRead()) {
        if (chargerChar.read()) {
          String value = "";
          int len = chargerChar.valueLength();
          for (int i = 0; i < len; i++) {
            value += (char)chargerChar.value()[i];
          }
          value.trim();
          if (value == "1" || value == "ON") {
            digitalWrite(BATTERY_READY_PIN, HIGH);
          } else {
            digitalWrite(BATTERY_READY_PIN, LOW);
          }
        }
      }
    }
  }
  BLE.poll();
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
