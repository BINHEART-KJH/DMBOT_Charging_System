#include <ArduinoBLE.h>

const char* STATION_NAME = "DM-STATION";
const char* SECRET_KEY = "DM_System_key";
const int RELAY_PIN = 4;
const int LED_PIN = LED_BUILTIN;

BLEDevice station;
BLECharacteristic nonceChar;
BLECharacteristic tokenChar;
BLECharacteristic chargerStateChar;

bool connected = false;
unsigned long lastCheckTime = 0;
const unsigned long CHECK_INTERVAL = 1000;

unsigned long lastBlink = 0;
bool ledState = false;

uint32_t calculateCRC32(const String& data);

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
    while (1);
  }

  BLE.scan();
  Serial.println("🔍 Scanning for Station...");
}

void loop() {
  BLE.poll();

  if (!connected) {
    // 점멸 효과
    if (millis() - lastBlink >= 500) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }

    BLEDevice peripheral = BLE.available();
    if (peripheral && peripheral.hasLocalName()) {
      String name = peripheral.localName();
      int rssi = peripheral.rssi();

      Serial.print("🔍 Found device: ");
      Serial.print(name);
      Serial.print(" | RSSI: ");
      Serial.println(rssi);

      if (name == STATION_NAME && rssi >= -60) {
        Serial.println("✅ Target matched. Connecting...");

        BLE.stopScan();  // ✅ 반드시 scan 중지

        if (peripheral.connect()) {
          delay(200);  // ✅ BLE stack 안정화 대기
          Serial.println("🔗 Connected to Station");
          station = peripheral;

          if (station.discoverService("180A")) {
            nonceChar = station.characteristic("2A29");
            tokenChar = station.characteristic("2A2A");
            chargerStateChar = station.characteristic("2A2C");

            if (nonceChar && tokenChar && chargerStateChar && nonceChar.canRead() && tokenChar.canWrite()) {
              nonceChar.read();
              String nonce = "";
              int len = nonceChar.valueLength();
              for (int i = 0; i < len; i++) {
                nonce += (char)nonceChar.value()[i];
              }

              String data = SECRET_KEY + nonce;
              uint32_t crc = calculateCRC32(data);
              char token[9];
              sprintf(token, "%08lX", crc);

              Serial.print("🔐 Sending token: ");
              Serial.println(token);

              tokenChar.writeValue(token);
              connected = true;
              digitalWrite(LED_PIN, HIGH);
              lastCheckTime = millis();
            } else {
              Serial.println("❌ GATT characteristics not ready");
              station.disconnect();
              delay(500);
              BLE.scan();
            }
          } else {
            Serial.println("❌ Service discovery failed");
            station.disconnect();
            delay(500);
            BLE.scan();
          }
        } else {
          Serial.println("❌ Connection failed");
          peripheral.disconnect();  // 🔁 명시적으로 해제
          delay(1000);
          BLE.scan();
        }
      }
    }
  } else {
    if (!station.connected()) {
      Serial.println("🔌 Disconnected from Station");
      connected = false;
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      BLE.scan();
      return;
    }

    if (millis() - lastCheckTime >= CHECK_INTERVAL) {
      chargerStateChar.read();
      int len = chargerStateChar.valueLength();
      String state = "";
      for (int i = 0; i < len; i++) {
        state += (char)chargerStateChar.value()[i];
      }

      Serial.print("⚡ Charger State: ");
      Serial.println(state);

      if (state == "ON") {
        digitalWrite(RELAY_PIN, HIGH);
      } else {
        digitalWrite(RELAY_PIN, LOW);
      }

      lastCheckTime = millis();
    }
  }
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