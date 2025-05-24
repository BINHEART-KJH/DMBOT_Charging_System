// ================= Station (Peripheral) - Refactored =================
#include <ArduinoBLE.h>
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
bool isAdvertising = false;
bool gotAuth = false;
unsigned long connectedTime = 0;

const int LED_PIN = LED_BUILTIN;

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

void startAdvertising() {
  BLE.setLocalName("BLE-TEST");
  BLE.setAdvertisedService(authService);
  BLE.addService(authService);
  nonceChar.setValue("WAIT");
  BLE.advertise();
  isAdvertising = true;
  digitalWrite(LED_PIN, HIGH);
  Serial.println("📡 Station advertising...");
}

void stopAdvertising() {
  BLE.stopAdvertise();
  isAdvertising = false;
  digitalWrite(LED_PIN, LOW);
  Serial.println("🛑 Advertising stopped");
}

void handleIdleState() {
  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  }
}

void handleAdvertisingState() {
  if (digitalRead(DOCK_PIN) == LOW) {
    stopAdvertising();
    state = IDLE;
    return;
  }
  BLEDevice incoming = BLE.central();
  if (incoming) {
    Serial.print("🔌 Connected to: ");
    Serial.println(incoming.address());

    if (incoming.rssi() < RSSI_THRESHOLD) {
      Serial.println("❌ RSSI too low, disconnecting");
      incoming.disconnect();
      return;
    }
    central = incoming;
    currentNonce = generateNonce();
    nonceChar.setValue(currentNonce.c_str());

    gotAuth = false;
    connectedTime = millis();
    digitalWrite(LED_PIN, LOW);
    state = WAIT_AUTH;
  }
}

void handleWaitAuthState() {
  if (!central.connected()) {
    Serial.println("🔌 Disconnected before auth");
    stopAdvertising();
    state = IDLE;
    return;
  }
  if (tokenChar.written()) {
    String token = String((const char *)tokenChar.value(), tokenChar.valueLength());
    if (isValidToken(token)) {
      Serial.println("✅ Auth Success");
      gotAuth = true;
      digitalWrite(LED_PIN, LOW);
      state = CONNECTED;
    } else {
      Serial.println("❌ Auth Failed → Disconnect");
      central.disconnect();
      stopAdvertising();
      state = IDLE;
    }
    return;
  }
  if (millis() - connectedTime > AUTH_TIMEOUT_MS) {
    Serial.println("⏳ Auth timeout → Disconnect");
    central.disconnect();
    stopAdvertising();
    state = IDLE;
  }
}

void handleConnectedState() {
  if (!central.connected()) {
    Serial.println("🔌 Disconnected");
    stopAdvertising();
    state = IDLE;
  }
  // 연결 유지 중 처리 영역
}

void setup() {
  Serial.begin(9600);
  pinMode(DOCK_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed");
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
