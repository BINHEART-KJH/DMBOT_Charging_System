#include <Arduino.h>
#include "robot_gpio.h"
#include "robot_ble.h"

unsigned long lastReportTime = 0;

String inputBuffer;

void rs485_init() {
  Serial1.begin(9600);
}

void rs485_report() {
  if (millis() - lastReportTime < 5000) return;
  lastReportTime = millis();

  Serial1.print("ST,0,BMSBLE,");
  Serial1.print(getBleConnectionState() ? "1" : "0");
  Serial1.println(",ED");

  Serial1.print("ST,0,BMSBATFULL,");
  Serial1.print(getBatteryFullStatus() ? "1" : "0");
  Serial1.println(",ED");

  Serial1.print("ST,0,BMSCHARGER,");
  Serial1.print(getChargerOkStatus() ? "1" : "0");
  Serial1.println(",ED");

  Serial1.print("ST,0,BMSCHARGERRELAY,");
  Serial1.print(getChargerRelayStatus() ? "1" : "0");
  Serial1.println(",ED");
}


void rs485_run() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.startsWith("ST,0,BMSROBOT,")) {
        if (inputBuffer.endsWith(",1,ED")) {
          setRelay(true);
          Serial.println("🟢 RS485: 릴레이 ON");
        } else if (inputBuffer.endsWith(",0,ED")) {
          setRelay(false);
          Serial.println("🔴 RS485: 릴레이 OFF");
        } else {
          Serial.println("⚠️ RS485: 잘못된 릴레이 명령");
        }
      }
      inputBuffer = "";  // Clear buffer
    } else {
      inputBuffer += c;
    }
  }
}