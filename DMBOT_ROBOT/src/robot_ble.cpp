// // robot_ble.cpp
// #include <ArduinoBLE.h>
// #include "robot_ble.h"
// #include "robot_auth.h"

// BLEDevice peripheral;
// BLECharacteristic nonceChar;
// BLECharacteristic tokenChar;

// bool isConnected = false;

// bool robotBLE_begin() {
//   if (!BLE.begin()) {
//     Serial.println("❌ BLE init failed");
//     return false;
//   }
//   Serial.println("✅ BLE initialized");
//   return true;
// }

// void robotBLE_startScan() {
//   Serial.println("🔍 Start scanning for peripherals...");
//   BLE.scan();
// }

// bool robotBLE_connectToStation() {
//   peripheral = BLE.available();

//   if (!peripheral) return false;

//   Serial.print("📡 Found: ");
//   Serial.println(peripheral.address());

//   if (peripheral.localName() != "DM-STATION") {
//     Serial.println("⛔ Not DM-STATION, skipping");
//     return false;
//   }

//   Serial.println("🔗 Connecting...");
//   if (!peripheral.connect()) {
//     Serial.println("❌ Connection failed");
//     return false;
//   }

//   Serial.println("✅ Connected!");
//   if (!peripheral.discoverAttributes()) {
//     Serial.println("❌ Discover failed");
//     peripheral.disconnect();
//     return false;
//   }

//   nonceChar = peripheral.characteristic("2A26");
//   tokenChar = peripheral.characteristic("2A27");

//   if (!nonceChar || !tokenChar) {
//     Serial.println("❌ Required characteristics not found");
//     peripheral.disconnect();
//     return false;
//   }

//   char nonce[33] = {0};
//   if (!nonceChar.readValue((uint8_t*)nonce, sizeof(nonce) - 1)) {
//     Serial.println("❌ Failed to read nonce");
//     peripheral.disconnect();
//     return false;
//   }

//   Serial.print("📥 Nonce received: ");
//   Serial.println(nonce);

//   char token[65] = {0};
//   generateHMAC_SHA256(nonce, "SHARED_SECRET_KEY", token);

//   Serial.print("📤 Sending token: ");
//   Serial.println(token);

//   if (!tokenChar.writeValue((const uint8_t*)token, strlen(token))) {
//     Serial.println("❌ Failed to write token");
//     peripheral.disconnect();
//     return false;
//   }

//   Serial.println("✅ Token sent");
//   isConnected = true;
//   return true;
// }

// void robotBLE_loop() {
//   if (isConnected && !peripheral.connected()) {
//     Serial.println("🔌 Disconnected");
//     isConnected = false;
//     BLE.scan();  // restart scan
//   }

//   if (!isConnected) {
//     if (robotBLE_connectToStation()) {
//       Serial.println("🔄 Still connected...");
//     }
//   }
// }