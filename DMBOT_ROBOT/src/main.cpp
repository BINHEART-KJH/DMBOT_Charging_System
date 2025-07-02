#include <Arduino.h>
#include <ArduinoBLE.h>
#include "hmac.h"

const char* STATION_NAME    = "DM-STATION";
const char* SHARED_SECRET   = "SHARED_SECRET_KEY";
const char* NONCE_UUID      = "2A26";
const char* TOKEN_UUID      = "2A27";
const char* BATTERY_UUID    = "2A28";
const char* CHARGER_UUID    = "2A29";
const char* RELAY_UUID      = "2A2A";

bool isConnected = false;

void setup() {
  Serial.begin(9600);
  delay(1000);

  if (!BLE.begin()) {
    Serial.println("❌ BLE init failed");
    while (1);
  }

  BLE.setLocalName("DMBOT-ROBOT");
  BLE.scan();  // 비동기 스캔
  Serial.println("🔍 Scanning...");
}

void loop() {
  if (isConnected) return;

  BLEDevice peripheral = BLE.available();
  if (!peripheral) return;

  Serial.print("📡 Found: ");
  Serial.println(peripheral.localName());

  if (peripheral.localName() != STATION_NAME) return;

  BLE.stopScan();  // 연결 시도 전에는 반드시 stopScan

  if (!peripheral.connect()) {
    Serial.println("❌ Connection failed");
    BLE.scan();  // 실패하면 재스캔
    return;
  }

  Serial.print("🟢 Connected to: ");
  Serial.println(peripheral.address());

  if (!peripheral.discoverAttributes()) {
    Serial.println("❌ Discover failed");
    peripheral.disconnect();
    BLE.scan();
    return;
  }

  BLEService service = peripheral.service("180C");
  if (!service) {
    Serial.println("❌ Auth service not found");
    peripheral.disconnect();
    BLE.scan();
    return;
  }

  BLECharacteristic nonceChar   = service.characteristic(NONCE_UUID);
  BLECharacteristic tokenChar   = service.characteristic(TOKEN_UUID);
  BLECharacteristic batteryChar = service.characteristic(BATTERY_UUID);
  BLECharacteristic chargerChar = service.characteristic(CHARGER_UUID);
  BLECharacteristic relayChar   = service.characteristic(RELAY_UUID);

  if (!nonceChar || !tokenChar) {
    Serial.println("❌ Required auth characteristics not found");
    peripheral.disconnect();
    BLE.scan();
    return;
  }

  // 🔐 인증 시작
  char nonce[33] = {0};
  nonceChar.readValue((uint8_t*)nonce, 32);
  nonce[32] = '\0';

  Serial.print("📥 Nonce received: ");
  Serial.println(nonce);

  char token[65] = {0};
  generateHMAC_SHA256(nonce, SHARED_SECRET, token);

  Serial.print("📤 Sending token: ");
  Serial.println(token);

  tokenChar.writeValue((const uint8_t*)token, 64);
  Serial.println("✅ Token sent");

  delay(100);  // 인증 대기

  if (!batteryChar || !chargerChar || !relayChar) {
    Serial.println("❌ Missing GATT characteristics for polling");
    peripheral.disconnect();
    BLE.scan();
    return;
  }

  isConnected = true;

  while (peripheral.connected()) {
    uint8_t battery = 0, charger = 0, relay = 0;

    bool ok1 = batteryChar.readValue(&battery, 1);
    bool ok2 = chargerChar.readValue(&charger, 1);
    bool ok3 = relayChar.readValue(&relay, 1);

    if (!ok1 || !ok2 || !ok3) {
      Serial.println("⚠️ Failed to read GATT values");
      break;
    }

    Serial.print("📊 BatteryFull: ");
    Serial.print(battery);
    Serial.print(" | ChargerOK: ");
    Serial.print(charger);
    Serial.print(" | JumperRelay: ");
    Serial.println(relay);

    delay(5000);
  }

  Serial.println("🔌 Disconnected");
  peripheral.disconnect();
  isConnected = false;
  BLE.scan();  // 연결 끊기면 다시 스캔
}
