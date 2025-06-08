#include "rs485_control.h"
#include <Arduino.h>

#define RS485_SERIAL      Serial1
#define CMD_BUFFER_SIZE   32

char commandBuffer[CMD_BUFFER_SIZE];
int bufferIndex = 0;

bool bleScanRequested = false;
bool bleStopRequested = false;

void setupRS485() {
  RS485_SERIAL.begin(9600);  // 8N1
  bufferIndex = 0;
  memset(commandBuffer, 0, sizeof(commandBuffer));
}

void processRS485Command() {
  while (Serial1.available()) {
    char ch = Serial1.read();

    // 명령 끝 구분자: CRLF or LF만 감지
    if (ch == '\n') {
      commandBuffer[bufferIndex] = '\0';  // Null-terminate

      String cmd = String(commandBuffer);
      cmd.trim();  // "\r\n" 같이 들어온 경우 잘라냄

      if (cmd == "BLE ON") {
        bleScanRequested = true;
        bleStopRequested = false;
        Serial1.println("BLE SCAN STARTED");
      } else if (cmd == "BLE OFF") {
        bleScanRequested = false;
        bleStopRequested = true;
        Serial1.println("BLE SCAN STOPPED");
      }

      bufferIndex = 0;
      memset(commandBuffer, 0, sizeof(commandBuffer));
    } else {
      if (bufferIndex < CMD_BUFFER_SIZE - 1) {
        commandBuffer[bufferIndex++] = ch;
      }
    }
  }
}

bool isBLEScanRequested() {
  return bleScanRequested;
}

bool isBLEStopRequested() {
  return bleStopRequested;
}

void clearBLECommandFlags() {
  bleScanRequested = false;
  bleStopRequested = false;
}