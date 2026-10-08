#include <Arduino.h>
#include "robot_gpio.h"
#include "robot_ble.h"
#include "config.h"

// ====== 옵션: 시리얼 모니터에서 리셋 명령 허용 ======
// ENABLE_SERIAL_MONITOR_RESET 0 (USB 없이 운용)
#define ENABLE_SERIAL_MONITOR_RESET 1
// ================================================

unsigned long lastReportTime_1 = 0;

String inputBuffer;
#if ENABLE_SERIAL_MONITOR_RESET
String monitorInputBuffer;
#endif

// ===== ORIGINAL CODE (기존 코드 주석 처리) =====
// String inputBuffer;
// unsigned long lastReportTime_1 = 0;
//
// static inline void hardResetNow() {
//   setRelay(false);
//   Serial1.println("ST,0,BMS_ROBOT_RESETTING,1,ED");
//   Serial1.flush();
//   Serial.println(">>> HARD RESET TRIGGERED <<<");
//   Serial.flush();
//   delay(30);
//   NVIC_SystemReset();
// }
// ===== END ORIGINAL CODE ===== */

// --- 하드리셋 헬퍼 ---
static inline void hardResetNow() {
  setRelay(false);

  Serial1.println("ST,0,BMS_ROBOT_RESETTING,1,ED");
  Serial1.flush();
  Serial.println(">>> HARD RESET TRIGGERED <<<");
  Serial.flush();
  delay(30);

#if defined(ESP32)
  ESP.restart();
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_NANO_RP2040_CONNECT)
  NVIC_SystemReset();
#else
  void(*resetFunc)(void) = 0; resetFunc();
#endif
}

// 공통 프레임 처리 함수
static void processFrameLine(const String& line) {
  if (line.startsWith("ST,0,BMS_ROBOT_CTRL_BAT_ON,")) {
    if (line.endsWith(",1,ED")) {
      setRelay(true);
      Serial.println("RS485: robot relay ON");
    } else if (line.endsWith(",0,ED")) {
      setRelay(false);
      Serial.println("RS485: robot relay OFF");
    } else {
      Serial.println("RS485: Wrong relay command value");
    }
    return;
  }

  if (line.startsWith("ST,0,BMS_ROBOT_RESET,")) {
    if (line.endsWith(",1,ED")) {
      Serial.println("RS485: hard reset command received");
      hardResetNow();
    } else {
      Serial.println("RS485: reset command ignored (value not 1)");
    }
    return;
  }
}

void rs485_init() {
  Serial1.begin(9600);
  delay(30);
  while (Serial1.available()) {
    (void)Serial1.read();
  }

#if ENABLE_SERIAL_MONITOR_RESET
  delay(10);
  while (Serial.available()) {
    (void)Serial.read();
  }
#endif
}

void rs485_report() {
  if (millis() - lastReportTime_1 < 5000) {
    return;
  }
  lastReportTime_1 = millis();

  bool bleConnected = getBleConnectionState();
  Serial1.print("ST,0,BMS_STATION_CONNECTED,");
  Serial1.print(bleConnected ? "1" : "0");
  Serial1.println(",ED");

  if (!bleConnected) {
    setRelay(false);
    Serial.println("RS485: robot relay OFF");
  }
}

void rs485_run() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        processFrameLine(inputBuffer);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
      if (inputBuffer.length() > 128) {
        inputBuffer = "";
      }
    }
  }

#if ENABLE_SERIAL_MONITOR_RESET
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      monitorInputBuffer.trim();
      if (monitorInputBuffer.length() > 0) {
        processFrameLine(monitorInputBuffer);
      }
      monitorInputBuffer = "";
    } else {
      monitorInputBuffer += c;
      if (monitorInputBuffer.length() > 128) {
        monitorInputBuffer = "";
      }
    }
  }
#endif
}
