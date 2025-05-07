#include <ArduinoBLE.h>

const int    LED_PIN            = LED_BUILTIN;
const int    DOCK_PIN           = 8;                      // 도킹 센서 입력
const int    RSSI_THRESHOLD     = -60;
const char*  TARGET_NAME        = "DMBOT-SERVICE";

const char*  AUTH_SERVICE_UUID  = "12345678-1234-5678-1234-1234567890AB";
const char*  AUTH_CHAR_UUID     = "ABCD1234-5678-1234-5678-ABCDEF012345";
const char*  AUTH_TOKEN         = "DH-010226";

bool        scanningActive     = false;
unsigned long previousMillis   = 0;
const unsigned long blinkInterval = 500;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DOCK_PIN, INPUT);  // 외부 신호로 HIGH/LOW 구분

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
  }
  BLE.setLocalName("DMBOT-STATION");

  Serial.println("🔍 Ready for docking...");
}

void loop() {
  bool docking = (digitalRead(DOCK_PIN) == HIGH);

  // 1) 도킹 ON → 스캔 연속 모드 시작
  if (docking && !scanningActive) {
    Serial.println("⚓ Docking ON → BLE.scan() 시작");
    BLE.scan();            // scanForName 대신
    scanningActive = true;
  }
  // 2) 도킹 OFF → 스캔 중지 + BLE 모듈 리셋
  else if (!docking && scanningActive) {
    Serial.println("⛔ Docking OFF → BLE.scan() 중지 & 모듈 리셋");
    BLE.stopScan();
    BLE.end();
    delay(100);
    if (!BLE.begin()) {
      Serial.println("❌ BLE re-init failed!");
      while (1);
    }
    BLE.setLocalName("DMBOT-STATION");
    scanningActive = false;
    Serial.println("🔄 BLE reset 완료, 도킹 대기중...");
  }

  // 3) 도킹 중 스캔된 디바이스 핸들링
  if (docking && scanningActive) {
    BLEDevice peripheral = BLE.available();
    if (peripheral
        && String(peripheral.localName()) == TARGET_NAME
        && peripheral.rssi() >= RSSI_THRESHOLD) 
    {
      Serial.println("───────────────────────────");
      Serial.print("Found → "); Serial.print(peripheral.localName());
      Serial.print(" ["); Serial.print(peripheral.address());
      Serial.print("], RSSI="); Serial.println(peripheral.rssi());
      Serial.println("───────────────────────────");

      Serial.println("➡️  조건 충족, 연결 시도...");
      BLE.stopScan();
      scanningActive = false;

      if (peripheral.connect()) {
        Serial.print("✅ Connected to "); Serial.println(peripheral.address());

        // 서비스 탐색
        Serial.print("🔑 Discovering service ");
        Serial.println(AUTH_SERVICE_UUID);
        if (peripheral.discoverService(AUTH_SERVICE_UUID)) {
          Serial.println("✅ Service discovered");
          BLECharacteristic authChar = peripheral.characteristic(AUTH_CHAR_UUID);
          if (authChar) {
            Serial.println("🔑 Auth characteristic found");
            // 토큰 전송
            Serial.print("✉️  Sending token: "); Serial.println(AUTH_TOKEN);
            if (authChar.writeValue(AUTH_TOKEN)) {
              Serial.println("✅ Auth token sent, operational state 진입");
              digitalWrite(LED_PIN, HIGH);
              // 연결 유지: 도킹이 유지되는 동안
              while (peripheral.connected() && docking) {
                BLE.poll();
                docking = (digitalRead(DOCK_PIN) == HIGH);
              }
              Serial.println("🔌 Connection ended or docking lost");
            } else {
              Serial.println("❌ Failed to send auth token");
            }
          } else {
            Serial.println("❌ Auth characteristic not found");
          }
        } else {
          Serial.println("❌ Service discovery failed");
        }
      } else {
        Serial.println("❌ Connection failed");
      }

      // 연결 종료 및 재스캔 준비
      peripheral.disconnect();
      Serial.println("🔄 Disconnected, docking 유지 시 재스캔");
      if (docking) {
        BLE.scan();
        scanningActive = true;
      }
    }
  }

  // 4) LED 상태 표시
  if (BLE.connected()) {
    digitalWrite(LED_PIN, HIGH);
  } else if (scanningActive) {
    unsigned long now = millis();
    if (now - previousMillis >= blinkInterval) {
      previousMillis = now;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }

  BLE.poll();
}