#include <ArduinoBLE.h>
#include <mbedtls/md.h>

enum BLEState {
  IDLE,
  ADVERTISING,
  WAIT_AUTH,
  CONNECTED
};

BLEState state = IDLE;

const int DOCKING_CHECK_PIN = 8;
const int RELAY_PIN = 7;
const int LED_PIN = LED_BUILTIN;

const char* SECRET_KEY = "DM_System_key";
const unsigned long AUTH_TIMEOUT_MS = 5000;
const unsigned long RELAY_HOLD_DELAY_MS = 10000;

BLEDevice central;
char currentNonce[9] = "";
bool gotAuth = false;
bool relayOn = false;
unsigned long authStartTime = 0;
unsigned long connectedTime = 0;
unsigned long lastBlinkTime = 0;
bool ledBlinkState = false;

BLEService authService("180A");
BLECharacteristic nonceChar("2A29", BLERead, 20);
BLECharacteristic tokenChar("2A2A", BLEWrite, 80);  // 64자 HMAC 대응
BLECharacteristic chargerStateChar("2A2C", BLERead, 10);

void generateNonce();
String computeHMAC_SHA256(const String& key, const String& message);

void resetBLE() {
  BLE.stopAdvertise();
  if (central && central.connected()) {
    central.disconnect();
    Serial.println("🔌 Central disconnected due to docking off.");
  }

  BLE.end();
  delay(100);
  if (!BLE.begin()) {
    Serial.println("❌ BLE re-init failed!");
    while (1);
  }

  BLE.setLocalName("DM-STATION");
  BLE.setDeviceName("DM-STATION");
  BLE.setAdvertisedService(authService);

  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  authService.addCharacteristic(chargerStateChar);
  BLE.addService(authService);

  nonceChar.writeValue("waiting");
  chargerStateChar.writeValue("OFF");

  state = IDLE;
  gotAuth = false;
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  relayOn = false;
  Serial.println("🔄 BLE reset complete.");
}

void setup() {
  pinMode(DOCKING_CHECK_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  randomSeed(analogRead(A0));

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed!");
    while (1);
  }

  BLE.setLocalName("DM-STATION");
  BLE.setDeviceName("DM-STATION");
  BLE.setAdvertisedService(authService);

  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  authService.addCharacteristic(chargerStateChar);
  BLE.addService(authService);

  nonceChar.writeValue("waiting");
  chargerStateChar.writeValue("OFF");

  tokenChar.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic characteristic) {
    int len = characteristic.valueLength();
    char tokenBuffer[81] = {0};  // 64자 + 널종료
    memcpy(tokenBuffer, characteristic.value(), len);
    tokenBuffer[len] = '\0';

    String receivedToken = String(tokenBuffer);
    String nonceStr = String(currentNonce).substring(0, 8);  // 정확한 8글자 nonce 사용
    String expectedToken = computeHMAC_SHA256(SECRET_KEY, nonceStr);

    Serial.print("🤖 Received token: ");
    Serial.println(receivedToken);
    Serial.print("✅ Expected token: ");
    Serial.println(expectedToken);

    if (receivedToken == expectedToken) {
      gotAuth = true;
      Serial.println("✅ Auth success");
    } else {
      Serial.println("❌ Auth failed. Disconnecting...");
      central.disconnect();
    }
  });
}

void loop() {
  BLE.poll();
  bool docking = digitalRead(DOCKING_CHECK_PIN);

  if (docking == LOW && state != IDLE) {
    Serial.println("⚠️ Docking LOW → Reset BLE");
    resetBLE();
    return;
  }

  if (state == ADVERTISING && (!central || !central.connected())) {
    if (millis() - lastBlinkTime >= 500) {
      ledBlinkState = !ledBlinkState;
      digitalWrite(LED_PIN, ledBlinkState);
      lastBlinkTime = millis();
    }
  }

  switch (state) {
    case IDLE:
      if (docking == HIGH) {
        BLE.advertise();
        state = ADVERTISING;
        Serial.println("📡 Advertising started");
      }
      break;

    case ADVERTISING:
      central = BLE.central();
      if (central) {
        int rssi = central.rssi();
        Serial.print("🔗 Connected Central | RSSI: ");
        Serial.println(rssi);

        if (rssi < -70) {
          Serial.println("📉 RSSI too weak. Disconnecting...");
          central.disconnect();
          return;
        }

        Serial.println("🔐 Waiting for HMAC auth...");
        generateNonce();
        nonceChar.writeValue(currentNonce);
        chargerStateChar.writeValue("OFF");
        gotAuth = false;
        authStartTime = millis();
        digitalWrite(LED_PIN, LOW);
        state = WAIT_AUTH;
      }
      break;

    case WAIT_AUTH:
      if (!central.connected()) {
        Serial.println("❌ Disconnected during auth");
        digitalWrite(RELAY_PIN, LOW);
        chargerStateChar.writeValue("OFF");
        state = ADVERTISING;
        break;
      }

      if (gotAuth) {
        Serial.println("🔒 Authenticated. Connection secured.");
        connectedTime = millis();
        digitalWrite(LED_PIN, HIGH);
        state = CONNECTED;
      } else if (millis() - authStartTime > AUTH_TIMEOUT_MS) {
        Serial.println("⏱ Auth timeout. Disconnecting...");
        central.disconnect();
        state = ADVERTISING;
      }
      break;

    case CONNECTED:
      if (!central.connected()) {
        Serial.println("🔌 Disconnected.");
        digitalWrite(RELAY_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        chargerStateChar.writeValue("OFF");
        relayOn = false;
        state = ADVERTISING;
      } else {
        if (!relayOn && (millis() - connectedTime >= RELAY_HOLD_DELAY_MS)) {
          digitalWrite(RELAY_PIN, HIGH);
          chargerStateChar.writeValue("ON");
          relayOn = true;
          Serial.println("⚡ Relay ON after 10s hold");
        }
      }
      break;
  }
}

void generateNonce() {
  for (int i = 0; i < 8; i++) {
    currentNonce[i] = "0123456789ABCDEF"[random(16)];
  }
  currentNonce[8] = '\0';
  Serial.print("🔑 Nonce: ");
  Serial.println(currentNonce);
}

String computeHMAC_SHA256(const String& key, const String& message) {
  byte output[32];
  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, info, 1); // 1 = HMAC
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key.c_str(), key.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)message.c_str(), message.length());
  mbedtls_md_hmac_finish(&ctx, output);
  mbedtls_md_free(&ctx);

  char hex[65];
  for (int i = 0; i < 32; i++) {
    sprintf(&hex[i * 2], "%02x", output[i]);  // 소문자 hex
  }
  hex[64] = '\0';

  return String(hex);
}