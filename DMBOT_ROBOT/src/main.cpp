#include <ArduinoBLE.h>
#include <SHA256.h>
#include <mbedtls/md.h>

const char *SECRET_KEY = "DM_System_key";
const char *TARGET_NAME = "DM-STATION";
const int RSSI_THRESHOLD = -60;
const int RELAY_PIN = 4;
const int LED_PIN = LED_BUILTIN;

BLECharacteristic nonceChar;
BLECharacteristic tokenChar;
BLECharacteristic chargerStateChar;

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;

void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  while (!Serial)
    ;

  if (!BLE.begin())
  {
    Serial.println("❌ Failed to initialize BLE!");
    while (1)
      ;
  }

  Serial.println("🔍 Scanning for Station...");
  BLE.scan();
}

String hmacSha256(const String &key, const String &message)
{
  unsigned char output[32];
  mbedtls_md_context_t ctx;
  const mbedtls_md_info_t *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, info, 1); // 1 = HMAC
  mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key.c_str(), key.length());
  mbedtls_md_hmac_update(&ctx, (const unsigned char *)message.c_str(), message.length());
  mbedtls_md_hmac_finish(&ctx, output);
  mbedtls_md_free(&ctx);

  char hexStr[65];
  for (int i = 0; i < 32; i++)
  {
    sprintf(hexStr + i * 2, "%02x", output[i]); // ✅ 소문자 hex 출력
  }
  return String(hexStr);
}

void loop()
{
  BLEDevice dev = BLE.available();

  if (dev)
  {
    if (!dev.hasLocalName())
    {
      Serial.println("⚠️ Device has no localName");
      return;
    }

    String localName = dev.localName();
    int rssi = dev.rssi();
    Serial.print("🔍 Found device: ");
    Serial.print(localName);
    Serial.print(" | RSSI: ");
    Serial.println(rssi);

    if (localName != TARGET_NAME || rssi < RSSI_THRESHOLD)
    {
      Serial.println("⛔ Not target device or weak signal");
      return;
    }

    Serial.println("✅ Target matched. Connecting...");
    BLE.stopScan();

    if (!dev.connect())
    {
      Serial.println("❌ Connection failed");
      BLE.scan();
      return;
    }

    Serial.println("🔗 Connected to Station");

    if (!dev.discoverAttributes())
    {
      Serial.println("❌ Attribute discovery failed");
      dev.disconnect();
      BLE.scan();
      return;
    }

    BLEService authService = dev.service("180A");
    if (!authService)
    {
      Serial.println("❌ Auth service not found");
      dev.disconnect();
      BLE.scan();
      return;
    }

    nonceChar = authService.characteristic("2A29");
    tokenChar = authService.characteristic("2A2A");
    chargerStateChar = authService.characteristic("2A2C");

    if (!nonceChar || !tokenChar || !chargerStateChar)
    {
      Serial.println("❌ GATT characteristics not found");
      dev.disconnect();
      BLE.scan();
      return;
    }

    if (!nonceChar.canRead() || !tokenChar.canWrite() || !chargerStateChar.canRead())
    {
      Serial.println("❌ GATT characteristic permissions incorrect");
      dev.disconnect();
      BLE.scan();
      return;
    }

    String nonce = "";
    byte buffer[32] = {0};
    int len = nonceChar.readValue(buffer, sizeof(buffer));
    if (len <= 0)
    {
      Serial.println("❌ Failed to read nonce");
      dev.disconnect();
      BLE.scan();
      return;
    }

    for (int i = 0; i < len; i++)
    {
      nonce += (char)buffer[i];
    }

    Serial.print("🔐 Received nonce: ");
    Serial.println(nonce);

    String token = hmacSha256(SECRET_KEY, nonce);
    Serial.print("🔐 Sending token: ");
    Serial.println(token);
    tokenChar.writeValue(token.c_str());

    digitalWrite(LED_PIN, HIGH); // 연결 표시

    while (dev.connected())
    {
      if (millis() - lastReadTime >= READ_INTERVAL)
      {
        lastReadTime = millis();

        len = chargerStateChar.valueLength();
        const uint8_t *stateData = chargerStateChar.value();
        String state = "";
        for (int i = 0; i < len; i++)
        {
          state += (char)stateData[i];
        }

        Serial.print("⚡ Charger State: ");
        Serial.println(state);

        if (state == "ON")
        {
          digitalWrite(RELAY_PIN, HIGH);
        }
        else
        {
          digitalWrite(RELAY_PIN, LOW);
        }
      }
    }

    Serial.println("🔌 Disconnected from Station");
    digitalWrite(LED_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW);
    BLE.scan();
  }

  // Blink LED while scanning
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (millis() - lastBlink > 500)
  {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastBlink = millis();
  }
}