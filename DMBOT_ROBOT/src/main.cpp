#include <ArduinoBLE.h>

void setup() {
  Serial.begin(9600);

  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (1);
  }

  Serial.println("Robot BLE scanning...");
  BLE.scan();
}

void loop() {
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    Serial.print("Found device: ");
    Serial.print(peripheral.address());
    Serial.print(" (");
    Serial.print(peripheral.localName());
    Serial.println(")");

    if (peripheral.localName() == "DM-STATION") {
      Serial.println("Target station found. Connecting...");
      BLE.stopScan();

      if (peripheral.connect()) {
        Serial.println("Connected to DM-STATION!");

        while (peripheral.connected()) {
          delay(100);
        }

        Serial.println("Disconnected from station");
      } else {
        Serial.println("Connection failed");
      }

      BLE.scan();  // Restart scanning after disconnect
    }
  }
}