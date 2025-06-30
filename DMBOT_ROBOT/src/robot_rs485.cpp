#include <Arduino.h>
#include "robot_rs485.h"
#include "robot_gpio.h"

#define RS485_SERIAL Serial1

#define INPUT_BUFFER_SIZE 64
char inputBuffer[INPUT_BUFFER_SIZE];
size_t inputPos = 0;

void setupRS485() {
  RS485_SERIAL.begin(9600);
  Serial.println("[RS485] Ready.");
}

void updateRS485() {
  while (RS485_SERIAL.available()) {
    char c = RS485_SERIAL.read();

    if (c == '\n' || c == '\r') {
      inputBuffer[inputPos] = '\0'; // 문자열 종료
      Serial.print("[RS485] Received: ");
      Serial.println(inputBuffer);

      if (strcmp(inputBuffer, "ST,0,BMSVCC,1,ED") == 0) {
        forceRelayState(true);
        Serial.println("→ Relay ON");
      } else if (strcmp(inputBuffer, "ST,0,BMSVCC,0,ED") == 0) {
        forceRelayState(false);
        Serial.println("→ Relay OFF");
      } else {
        Serial.println("→ Unknown command");
      }

      inputPos = 0; // 버퍼 초기화
    } else {
      if (inputPos < INPUT_BUFFER_SIZE - 1) {
        inputBuffer[inputPos++] = c;
      }
    }
  }
}