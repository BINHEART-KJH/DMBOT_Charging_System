#include "robot_rs485.h"
#include "robot_gpio.h"
#include <Arduino.h>

#define RS485 Serial1

static String rxBuffer;

void robotRS485_init() {
  RS485.begin(9600);
}

void robotRS485_update() {
  while (RS485.available()) {
    char c = RS485.read();

    if (c == '\n') {
      rxBuffer.trim();
      if (rxBuffer.startsWith("ST,0,BMSROBOT,")) {
        if (rxBuffer.endsWith(",1,ED")) {
          Serial.println("[RS485] 릴레이 ON 명령 수신");
          robotGPIO_setRelay(true);
        } else if (rxBuffer.endsWith(",0,ED")) {
          Serial.println("[RS485] 릴레이 OFF 명령 수신");
          robotGPIO_setRelay(false);
        }
      }
      rxBuffer = "";
    } else {
      rxBuffer += c;
    }
  }
}

void robotRS485_sendBLEStatus(bool connected) {
  if (connected) {
    RS485.println("ST,0,BMSBLE,1,ED");
    Serial.println("[RS485] BLE 연결됨 전송");
  } else {
    RS485.println("ST,0,BMSBLE,0,ED");
    Serial.println("[RS485] BLE 연결 해제 전송");
  }
}