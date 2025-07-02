// #include <ArduinoBLE.h>
// #include "station_auth.h"
// #include "hmac.h"
// #include "sha256.h"

// const char* SECRET_KEY = "DMBOT_SHARED_SECRET";

// BLEService authService("180A");
// BLECharacteristic nonceChar("2A26", BLERead, 32);
// BLECharacteristic tokenChar("2A27", BLEWrite, 64);

// char currentNonce[33] = {0};
// bool isAuthSuccess = false;

// void generateNonce(char* buffer) {
//   const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
//   for (int i = 0; i < 32; i++) {
//     buffer[i] = charset[random(0, sizeof(charset) - 1)];
//   }
//   buffer[32] = '\0';
// }

// void setupAuthService() {
//   authService.addCharacteristic(nonceChar);
//   authService.addCharacteristic(tokenChar);
//   BLE.addService(authService);
// }

// void resetAuth() {
//   isAuthSuccess = false;
//   generateNonce(currentNonce);
//   nonceChar.writeValue(currentNonce);
// }

// bool processAuthToken() {
//   if (!tokenChar.written()) return false;

//   char receivedToken[65] = {0};
//   tokenChar.readValue((uint8_t*)receivedToken, sizeof(receivedToken) - 1);

//   char expectedToken[65] = {0};
//   generateHMAC_SHA256(currentNonce, SECRET_KEY, expectedToken);

//   if (strcmp(receivedToken, expectedToken) == 0) {
//     isAuthSuccess = true;
//     return true;
//   }
//   return false;
// }

// bool isAuthenticated() {
//   return isAuthSuccess;
// }
