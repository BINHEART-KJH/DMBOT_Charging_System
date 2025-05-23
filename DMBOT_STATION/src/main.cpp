#include <Arduino.h>
#include <ArduinoBLE.h>
#include <unordered_map>
#include <string>

// ======================= 상태 정의 =======================
enum BLEState {
  IDLE,
  ADVERTISING,
  CONNECTED,
  WAIT_AUTH,
  DISCONNECTING
};
BLEState state = IDLE;

// ======================= 설정 상수 =======================
const int DOCK_PIN = 8;
const int RSSI_THRESHOLD = -50;
const int AUTH_TIMEOUT_MS = 5000;
const int TEMP_CACHE_SIZE = 10;
const unsigned long REJECT_COOLDOWN = 10000;
const unsigned long LOG_INTERVAL_PER_MAC = 10000;
const char* SECRET_KEY = "MY_SECRET_KEY";

// ======================= 인증 구조 =======================
String currentNonce = "";
BLEService authService("180A");
BLECharacteristic nonceChar("2A29", BLERead, 20);
BLECharacteristic tokenChar("2A2A", BLEWrite, 40);

std::unordered_map<std::string, unsigned long> lastBlockLogTime;
struct TempMAC {
  std::string mac;
  unsigned long timestamp;
};
TempMAC recentRejects[TEMP_CACHE_SIZE];
int recentIndex = 0;

bool isTemporarilyBlocked(const std::string& mac) {
  unsigned long now = millis();
  for (int i = 0; i < TEMP_CACHE_SIZE; i++) {
    if (recentRejects[i].mac == mac && now - recentRejects[i].timestamp < REJECT_COOLDOWN) {
      return true;
    }
  }
  return false;
}

void addTempBlock(const std::string& mac) {
  recentRejects[recentIndex] = {mac, millis()};
  recentIndex = (recentIndex + 1) % TEMP_CACHE_SIZE;
}

String generateNonce() {
  char buffer[7];
  for (int i = 0; i < 6; i++) buffer[i] = random(33, 126);
  buffer[6] = '\0';
  return String(buffer);
}

uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
      else crc >>= 1;
    }
  }
  return ~crc;
}

String generateToken(String nonce, const char* key) {
  String input = nonce + String(key);
  uint32_t token = crc32((const uint8_t*)input.c_str(), input.length());
  char buf[9];
  sprintf(buf, "%08X", token);
  return String(buf);
}

bool isValidToken(String token) {
  String expected = generateToken(currentNonce, SECRET_KEY);
  return token.equalsIgnoreCase(expected);
}

// ======================= BLE =======================
bool isAdvertising = false;
unsigned long connectedTime = 0;
bool gotAuth = false;

void startAdvertising() {
  if (!isAdvertising) {
    BLE.setLocalName("DMBOT-STATION");
    BLE.advertise();
    isAdvertising = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("📡 Advertising started");
  }
}

void stopAdvertising() {
  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("🛑 Advertising stopped");
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(DOCK_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed");
    while (1);
  }

  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  BLE.setAdvertisedService(authService);
  BLE.addService(authService);
  Serial.println("✅ BLE service setup complete");

  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  }
}

void loop() {
  switch (state) {
    case IDLE:
      if (digitalRead(DOCK_PIN) == HIGH) {
        Serial.println("🟢 Sensor ON → Start Advertising");
        startAdvertising();
        state = ADVERTISING;
      }
      break;

    case ADVERTISING:
      if (digitalRead(DOCK_PIN) == LOW) {
        Serial.println("⚪ Sensor OFF → Stop Advertising");
        stopAdvertising();
        state = IDLE;
      } else {
        BLEDevice central = BLE.central();
        if (central) {
          std::string mac = central.address().c_str();

          if (isTemporarilyBlocked(mac)) {
            unsigned long now = millis();
            if (now - lastBlockLogTime[mac] > LOG_INTERVAL_PER_MAC) {
              Serial.println("⛔ Temporarily blocked MAC: " + String(mac.c_str()));
              lastBlockLogTime[mac] = now;
            }
            central.disconnect();
            return;
          }

          Serial.println("🔌 Connected: " + String(mac.c_str()));
          Serial.print("📶 RSSI: ");
          Serial.println(central.rssi());

          if (central.rssi() < RSSI_THRESHOLD) {
            Serial.println("❌ RSSI too low, disconnecting");
            addTempBlock(mac);
            central.disconnect();
            return;
          }

          currentNonce = generateNonce();
          nonceChar.setValue(currentNonce.c_str());

          connectedTime = millis();
          gotAuth = false;
          digitalWrite(LED_BUILTIN, LOW);
          state = WAIT_AUTH;
        }
      }
      break;

    case WAIT_AUTH: {
      BLEDevice central = BLE.central();
      if (!central || !central.connected()) {
        Serial.println("🔌 Disconnected before auth");
        stopAdvertising();
        state = IDLE;
        break;
      }

      if (tokenChar.written()) {
        int len = tokenChar.valueLength();
        const uint8_t *val = tokenChar.value();
        String token((const char *)val, len);

        if (isValidToken(token)) {
          Serial.println("✅ Auth Success");
          gotAuth = true;
          digitalWrite(LED_BUILTIN, LOW);
          state = CONNECTED;
        } else {
          Serial.println("❌ Invalid token → disconnect");
          addTempBlock(central.address().c_str());
          central.disconnect();
          stopAdvertising();
          state = IDLE;
        }
        break;
      }

      if (!gotAuth && millis() - connectedTime >= AUTH_TIMEOUT_MS) {
        std::string mac = central.address().c_str();
        Serial.println("⏳ Auth timeout, disconnecting");
        addTempBlock(mac);
        central.disconnect();
        stopAdvertising();
        state = IDLE;
        break;
      }
    } break;

    case CONNECTED:
      break;

    case DISCONNECTING:
      Serial.println("🔄 Disconnecting");
      stopAdvertising();
      state = IDLE;
      break;
  }
}
