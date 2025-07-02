#include "station_auth.h"
#include "hmac.h"

BLEService authService("12345678-1234-5678-1234-56789DMBOTA1");
BLECharacteristic nonceChar("12345678-1234-5678-1234-56789DMBOTA2", BLERead | BLEWrite, 32);
BLECharacteristic tokenChar("12345678-1234-5678-1234-56789DMBOTA3", BLERead | BLEWrite, 64);

static bool authenticated = false;
const char* secretKey = "DMBOT_SHARED_SECRET";

void stationAuth_init() {
  BLE.setAdvertisedService(authService);
  authService.addCharacteristic(nonceChar);
  authService.addCharacteristic(tokenChar);
  BLE.addService(authService);
}

void stationAuth_reset() {
  authenticated = false;
  nonceChar.writeValue((const uint8_t*)"", 1);
  tokenChar.writeValue((const uint8_t*)"", 1);
}

bool isStationAuthenticated() {
  return authenticated;
}

void stationAuth_update(BLEDevice central) {
  if (authenticated) return;
  if (!nonceChar.written() || !tokenChar.written()) return;

  String nonce = String((const char*)nonceChar.value());
  String token = String((const char*)tokenChar.value());

  if (nonce.length() == 0 || token.length() == 0) return;

  // ✅ HMAC 함수 호출
  char expectedToken[65];
  generateHMAC_SHA256(nonce.c_str(), secretKey, expectedToken);

  if (token.equalsIgnoreCase(expectedToken)) {
    Serial.println("[AUTH] 인증 성공");
    authenticated = true;
  } else {
    Serial.println("[AUTH] 인증 실패");
  }
}