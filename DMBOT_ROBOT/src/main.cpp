#include <ArduinoBLE.h>

const char* AUTH_SERVICE_UUID = "12345678-1234-5678-1234-1234567890AB";
const char* AUTH_CHAR_UUID    = "ABCD1234-5678-1234-5678-ABCDEF012345";
const char* AUTH_TOKEN        = "DH-010226";

BLEService       authService(AUTH_SERVICE_UUID);
BLECharacteristic authChar(
  AUTH_CHAR_UUID,
  BLEWrite,               // Central이 쓰기 허용
  strlen(AUTH_TOKEN)      // 토큰 길이
);

const int LED_PIN = LED_BUILTIN;
unsigned long previousMillis = 0;
const unsigned long BLINK_INTERVAL = 500;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed");
  }

  // Peripheral 설정
  BLE.setLocalName("DMBOT-SERVICE");
  authService.addCharacteristic(authChar);
  BLE.addService(authService);
  BLE.setAdvertisedService(authService);
  BLE.advertise();
  Serial.println("📡 Advertising DMBOT-SERVICE with auth");
}

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("🔗 Connected: "); Serial.println(central.address());

    unsigned long start = millis();
    bool authed = false;

    // 5초간 토큰 수신 대기
    while (central.connected() && millis() - start < 5000) {
      BLE.poll();
      if (authChar.written()) {
        String recv = String((char*)authChar.value(), authChar.valueLength());
        Serial.print("✉️  Received token: "); Serial.println(recv);
        if (recv == AUTH_TOKEN) {
          authed = true;
          Serial.println("✅ Authenticated");
          break;
        } else {
          Serial.println("❌ Token mismatch");
        }
      }
    }

    if (!authed) {
      Serial.println("⏱️  Auth timeout → disconnect");
    } else {
      Serial.println("💡 Entering operational state (LED ON)");
      digitalWrite(LED_PIN, HIGH);
      // 충전 컨트롤 로직 수행...
      while (central.connected()) {
        BLE.poll();
      }
      Serial.println("🔌 Central disconnected");
    }

    // 언제나 광고 재시작
    central.disconnect();
    BLE.stopAdvertise();
    digitalWrite(LED_PIN, LOW);
    BLE.advertise();
    Serial.println("🔄 Advertising restarted");
  }
  else {
    // 광고 중 LED 깜박임
    unsigned long now = millis();
    if (now - previousMillis >= BLINK_INTERVAL) {
      previousMillis = now;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}