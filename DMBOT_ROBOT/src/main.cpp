// ================= Robot (Central) - Refactored =================
#include <ArduinoBLE.h>

const char *TARGET_NAME = "BLE-TEST";
const char *SECRET_KEY = "DM_System_key";

BLEDevice peripheral;
BLECharacteristic nonceChar;
BLECharacteristic tokenChar;

bool connected = false;
String currentNonce = "";

const int LED_PIN = LED_BUILTIN;
unsigned long lastBlink = 0;
bool ledState = false;

// === Utility ===
uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
  }
  return ~crc;
}

String generateToken(const String &nonce, const char *key) {
  String input = nonce + String(key);
  uint32_t token = crc32((const uint8_t *)input.c_str(), input.length());
  char buf[9];
  sprintf(buf, "%08X", token);
  return String(buf);
}

void blinkWhileScanning() {
  if (millis() - lastBlink >= 500) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
    lastBlink = millis();
  }
}

bool connectToPeripheral(BLEDevice found) {
  Serial.print("🔗 Connecting to: ");
  Serial.println(found.address());
  peripheral = found;

  if (!peripheral.connect()) {
    Serial.println("❌ Connection failed");
    return false;
  }

  Serial.println("✅ Connected!");

  if (!peripheral.discoverAttributes()) {
    Serial.println("❌ Attribute discovery failed");
    peripheral.disconnect();
    return false;
  }

  nonceChar = peripheral.characteristic("2A29");
  tokenChar = peripheral.characteristic("2A2A");

  if (!(nonceChar && nonceChar.canRead() && tokenChar && tokenChar.canWrite())) {
    Serial.println("❌ Characteristics invalid");
    peripheral.disconnect();
    return false;
  }

  char buffer[21] = {0};
  if (!nonceChar.readValue((uint8_t *)buffer, sizeof(buffer) - 1)) {
    Serial.println("❌ Failed to read nonce");
    peripheral.disconnect();
    return false;
  }
  currentNonce = String(buffer);
  Serial.println("📥 Nonce: " + currentNonce);

  String token = generateToken(currentNonce, SECRET_KEY);
  if (!tokenChar.writeValue(token.c_str())) {
    Serial.println("❌ Failed to write token");
    peripheral.disconnect();
    return false;
  }

  Serial.println("📤 Sent token: " + token);
  digitalWrite(LED_PIN, HIGH);
  connected = true;
  return true;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed");
    while (1);
  }
  Serial.println("✅ Robot Central initialized");
}

void loop() {
  if (!connected) {
    Serial.println("🔍 Scanning...");
    BLE.scan();

    unsigned long startScan = millis();
    while (millis() - startScan < 5000) {
      BLEDevice found = BLE.available();
      if (found && found.localName() == TARGET_NAME) {
        BLE.stopScan();
        if (connectToPeripheral(found)) return;
      }
      blinkWhileScanning();
    }
    BLE.stopScan();
  } else {
    if (!peripheral.connected()) {
      Serial.println("❎ Disconnected");
      connected = false;
      digitalWrite(LED_PIN, LOW);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
  }
}
