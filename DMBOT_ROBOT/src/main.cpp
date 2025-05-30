// ================= Robot (Central) =================
#include <ArduinoBLE.h>

const char *TARGET_NAME = "DMBOT-STATION";
const char *SECRET_KEY = "DM_System_key";
const char *SERVICE_UUID = "180A";
const char *NONCE_UUID = "2A29";
const char *TOKEN_UUID = "2A2A";
const char *CHARGER_STATE_UUID = "2A2C";

BLEDevice peripheral;
BLECharacteristic nonceChar;
BLECharacteristic tokenChar;
BLECharacteristic chargerStateChar;

const int BATTERY_READY_PIN = 4;
const int LED_PIN = LED_BUILTIN;

bool connected = false;
bool ledState = false;
unsigned long lastBlink = 0;
String currentNonce = "";

// ================= Utility =================
uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
    }
  }
  return ~crc;
}

String generateToken(const String &nonce, const char *key) {
  String input = nonce + key;
  uint32_t token = crc32((const uint8_t *)input.c_str(), input.length());
  char buf[9];
  sprintf(buf, "%08X", token);
  return String(buf);
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY_READY_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BATTERY_READY_PIN, LOW);

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed");
    while (1);
  }
  Serial.println("✅ Robot BLE Central Ready");
}

void loop() {
  BLE.poll();

  if (!connected) {
    // LED blink every 1s in scanning mode
    if (millis() - lastBlink >= 1000) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlink = millis();
    }

    digitalWrite(BATTERY_READY_PIN, LOW);

    BLE.scan();
    BLEDevice found;
    unsigned long scanStart = millis();

    while (millis() - scanStart < 3000) {
      found = BLE.available();
      if (found && found.localName() == TARGET_NAME) {
        BLE.stopScan();
        peripheral = found;
        Serial.print("✅ Found: ");
        Serial.println(peripheral.address());

        if (peripheral.connect()) {
          Serial.println("🔗 Connected");

          if (peripheral.discoverAttributes()) {
            nonceChar = peripheral.characteristic(NONCE_UUID);
            tokenChar = peripheral.characteristic(TOKEN_UUID);
            chargerStateChar = peripheral.characteristic(CHARGER_STATE_UUID);

            if (nonceChar && tokenChar && chargerStateChar && nonceChar.canRead() && tokenChar.canWrite()) {
              char buffer[21] = {0};
              if (nonceChar.readValue((uint8_t *)buffer, sizeof(buffer) - 1)) {
                currentNonce = String(buffer);
                Serial.println("📥 Nonce: " + currentNonce);
                String token = generateToken(currentNonce, SECRET_KEY);

                if (tokenChar.writeValue(token.c_str())) {
                  Serial.println("📤 Token Sent");
                  connected = true;
                  digitalWrite(LED_PIN, HIGH);
                } else {
                  Serial.println("❌ Token Write Failed");
                  peripheral.disconnect();
                }
              } else {
                Serial.println("❌ Nonce Read Failed");
                peripheral.disconnect();
              }
            } else {
              Serial.println("❌ Characteristics Not Found");
              peripheral.disconnect();
            }
          } else {
            Serial.println("❌ Discover Failed");
            peripheral.disconnect();
          }
        } else {
          Serial.println("❌ Connect Failed");
        }
        break;
      }
    }
    BLE.stopScan();
  } else {
    // Handle disconnect
    if (!peripheral.connected()) {
      Serial.println("❌ Disconnected");
      connected = false;
      digitalWrite(LED_PIN, LOW);
      digitalWrite(BATTERY_READY_PIN, LOW);
      return;
    }

    // Maintain connection and update BATTERY_READY
    if (chargerStateChar && chargerStateChar.canRead()) {
      char buffer[5] = {0};
      chargerStateChar.readValue((uint8_t*)buffer, sizeof(buffer) - 1);
      if (strcmp(buffer, "1") == 0) {
        digitalWrite(BATTERY_READY_PIN, HIGH);
      } else {
        digitalWrite(BATTERY_READY_PIN, LOW);
      }
    } else {
      digitalWrite(BATTERY_READY_PIN, LOW);
    }
  }
}
