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

  // 🔗 BLE 연결 상태
  Serial1.print("ST,0,BMS_STATION_CONNECTED,");
  Serial1.print(getBleConnectionState() ? "1" : "0");
  Serial1.println(",ED");

  // 🔋 배터리 Full 여부
  Serial1.print("ST,0,BMS_STATION_BAT_FULL,");
  Serial1.print(getBatteryFullStatus() ? "1" : "0");
  Serial1.println(",ED");

  // 🔌 충전 중 여부 (Station과 Robot이 통전 중)
  Serial1.print("ST,0,BMS_STATION_BAT_CHARGING,");
  Serial1.print(getChargerOkStatus() ? "1" : "0");
  Serial1.println(",ED");

  // ⚡ 충전 준비 완료 (Station의 Jumper 릴레이 상태)
  Serial1.print("ST,0,BMS_STATION_BAT_ON,");
  Serial1.print(getChargerRelayStatus() ? "1" : "0");
  Serial1.println(",ED");
}

void rs485_run() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      inputBuffer.trim();

      if (inputBuffer.startsWith("ST,0,BMS_ROBOT_CTRL_BAT,")) {
        if (inputBuffer.endsWith(",1,ED")) {
          setRelay(true);
          Serial.println("RS485: 로봇 릴레이 ON (충전 시작)");
        } else if (inputBuffer.endsWith(",0,ED")) {
          setRelay(false);
          Serial.println("RS485: 로봇 릴레이 OFF (충전 중단)");
        } else {
          Serial.println("RS485: 잘못된 릴레이 명령 수신");
        }
      }

      inputBuffer = "";  // Clear buffer after processing
    } else {
      inputBuffer += c;
    }
  }
}
