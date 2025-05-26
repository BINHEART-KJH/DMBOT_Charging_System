#include <Arduino.h>
#include <ArduinoBLE.h>

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
const int BLACKLIST_SIZE = 5;

// ======================= 변수 =======================
unsigned long connectedTime = 0;
bool gotLocalName = false;
String blacklist[BLACKLIST_SIZE];
int blacklistIndex = 0;
bool isAdvertising = false;

// ======================= BLE 서비스/캐릭터리스틱 =======================
BLEService dummyService("180A");
BLECharacteristic dummyChar("2A29", BLERead, 20);

// ======================= 함수 =======================
void startAdvertising() {
  if (!isAdvertising) {
    BLE.setLocalName("DMBOT-STATION");
    BLE.advertise();
    isAdvertising = true;
    Serial.println("Advertising started");
  }
}

void stopAdvertising() {
  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
    Serial.println("Advertising stopped");
  }
}

bool isBlacklisted(String mac) {
  for (int i = 0; i < BLACKLIST_SIZE; i++) {
    if (blacklist[i] == mac)
      return true;
  }
  return false;
}

void addToBlacklist(String mac) {
  if (blacklistIndex < BLACKLIST_SIZE) {
    blacklist[blacklistIndex++] = mac;
    Serial.println("Added to blacklist: " + mac);
  }
}

// ======================= setup =======================
void setup() {
  Serial.begin(9600);
  delay(1000);  // 시리얼 안정화
  Serial.println("Setup 시작");

  pinMode(DOCK_PIN, INPUT);

  if (!BLE.begin()) {
    Serial.println("BLE 초기화 실패");
    while (1);
  }

  dummyService.addCharacteristic(dummyChar);
  BLE.setAdvertisedService(dummyService);
  BLE.addService(dummyService);
  Serial.println("BLE 서비스 등록 완료");

  if (digitalRead(DOCK_PIN) == HIGH) {
    startAdvertising();
    state = ADVERTISING;
  }
}

// ======================= loop =======================
void loop() {
  switch (state) {
    case IDLE:
      if (digitalRead(DOCK_PIN) == HIGH) {
        Serial.println("센서 ON → 광고 시작");
        startAdvertising();
        state = ADVERTISING;
      }
      break;

    case ADVERTISING:
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
      break;
  }
}