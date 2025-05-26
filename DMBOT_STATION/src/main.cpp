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

<<<<<<< HEAD
// ======================= 함수 =======================
void startAdvertising() {
  if (!isAdvertising) {
    BLE.setLocalName("DMBOT-STATION");
    BLE.advertise();
    isAdvertising = true;
    Serial.println("Advertising started");
=======
uint32_t crc32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : (crc >> 1);
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01
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
<<<<<<< HEAD
  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
    Serial.println("Advertising stopped");
=======
  BLE.stopAdvertise();
  isAdvertising = false;
  digitalWrite(LED_PIN, LOW);
  Serial.println("🛑 Advertising stopped");
}

void handleIdleState() {
  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01
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

<<<<<<< HEAD
void addToBlacklist(String mac) {
  if (blacklistIndex < BLACKLIST_SIZE) {
    blacklist[blacklistIndex++] = mac;
    Serial.println("Added to blacklist: " + mac);
=======
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
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01
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
<<<<<<< HEAD
  delay(1000);  // 시리얼 안정화
  Serial.println("Setup 시작");

=======
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01
  pinMode(DOCK_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  if (!BLE.begin()) {
<<<<<<< HEAD
    Serial.println("BLE 초기화 실패");
    while (1);
  }

  dummyService.addCharacteristic(dummyChar);
  BLE.setAdvertisedService(dummyService);
  BLE.addService(dummyService);
  Serial.println("BLE 서비스 등록 완료");
=======
    Serial.println("❌ BLE init failed");
    while (1);
  }
  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01

  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  }
}

void loop() {
  switch (state) {
    case IDLE:
<<<<<<< HEAD
      if (digitalRead(DOCK_PIN) == HIGH) {
        Serial.println("센서 ON → 광고 시작");
        startAdvertising();
        state = ADVERTISING;
      }
=======
      handleIdleState();
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01
      break;
    case ADVERTISING:
<<<<<<< HEAD
      if (digitalRead(DOCK_PIN) == LOW) {
        Serial.println("센서 OFF → 광고 중단");
        stopAdvertising();
        state = IDLE;
      } else {
        BLEDevice central = BLE.central();
        if (central) {
          String mac = central.address();

          if (isBlacklisted(mac)) {
            Serial.println("블랙리스트 기기 접속 시도: " + mac);
            // 부하 줄이기 위해 disconnect 생략 가능
            return;
          }

          Serial.println("🔌 연결됨: " + mac);
          Serial.print("RSSI: ");
          Serial.println(central.rssi());

          if (central.rssi() < RSSI_THRESHOLD) {
            Serial.println("RSSI 너무 낮음, 연결 종료");
            central.disconnect();
            return;
          }

          connectedTime = millis();
          gotLocalName = false;
          state = WAIT_AUTH;
        }
      }
      break;

    case WAIT_AUTH: {
      BLEDevice central = BLE.central();

      if (!central || !central.connected()) {
        Serial.println("🔌 연결 끊김 (인증 전)");
        stopAdvertising();
        state = IDLE;
        break;
      }

      if (dummyChar.written()) {
        int len = dummyChar.valueLength();
        const uint8_t *val = dummyChar.value();
        String name((const char *)val, len);
        Serial.println("localName 수신됨: " + name);
        gotLocalName = true;
        state = CONNECTED;
        break;
      }

      if (!gotLocalName && millis() - connectedTime >= AUTH_TIMEOUT_MS) {
        Serial.println("인증 타임아웃 (5초 경과)");
        addToBlacklist(central.address());
        central.disconnect();
        stopAdvertising();
        state = IDLE;
        break;
      }
    } break;

    case CONNECTED:
      Serial.println("인증 성공 상태 - CONNECTED");
      // 릴레이 제어나 다음 단계 로직 여기에 추가 가능
      delay(1000);  // 로그 과다 방지
      break;

    case DISCONNECTING:
      Serial.println("DISCONNECTING → IDLE");
      stopAdvertising();
      state = IDLE;
=======
      handleAdvertisingState();
      break;
    case WAIT_AUTH:
      handleWaitAuthState();
      break;
    case CONNECTED:
      handleConnectedState();
>>>>>>> 5ac69b07d3ec56e4ef71f9a5fea578a0fdeb1d01
      break;
  }
}
