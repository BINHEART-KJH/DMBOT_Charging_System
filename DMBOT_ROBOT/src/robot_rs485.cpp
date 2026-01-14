/*#include <Arduino.h>
#include "robot_gpio.h"
#include "robot_ble.h"

// ====== 옵션: 시리얼 모니터에서 리셋 명령 허용 ======
// ENABLE_SERIAL_MONITOR_RESET 0 (USB 없이 운용)
#define ENABLE_SERIAL_MONITOR_RESET 1
// ================================================

unsigned long lastReportTime_1 = 0;

String inputBuffer;        // RS485(Serial1) 수신 버퍼
#if ENABLE_SERIAL_MONITOR_RESET
String monitorInputBuffer; // Serial(USB) 수신 버퍼 - 테스트 용
#endif

// --- 하드리셋 헬퍼 ---
static inline void hardResetNow() {
  // 안전을 위해 릴레이 OFF 후 리셋
  setRelay(false);

  // (선택) 호스트/상위에 ACK 한번 통지
  Serial1.println("ST,0,BMS_ROBOT_RESETTING,1,ED");
  Serial1.flush();
  Serial.println(">>> HARD RESET TRIGGERED <<<");
  Serial.flush();
  delay(30);

  // 실제 리셋
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
  // 형식 예:
  // ST,0,BMS_ROBOT_CTRL_BAT_ON,1,ED
  // ST,0,BMS_ROBOT_RESET,1,ED

  if (line.startsWith("ST,0,BMS_ROBOT_CTRL_BAT_ON,")) {
    if (line.endsWith(",1,ED")) {
      setRelay(true); // Relay ON
      Serial.println("RS485: robot relay ON");
    } else if (line.endsWith(",0,ED")) {
      setRelay(false); // Relay OFF
      Serial.println("RS485: robot relay OFF");
    } else {
      Serial.println("RS485: Wrong relay command value");
    }
    return;
  }

  // === 하드리셋 명령 ===
  // ST,0,BMS_ROBOT_RESET,1,ED  -> 즉시 하드리셋
  if (line.startsWith("ST,0,BMS_ROBOT_RESET,")) {
    if (line.endsWith(",1,ED")) {
      Serial.println("RS485: 하드리셋 명령 수신 → 리부트");
      hardResetNow();
      // 보통 여기로 돌아오지 않음
    } else {
      Serial.println("RS485: 하드리셋 명령 무시(값이 1이 아님)");
    }
    return;
  }

  // 여기에 다른 명령들이 있다면 이어서 추가...
}

void rs485_init()
{
  Serial1.begin(9600);
  // 부팅 직후 RX 버퍼에 남아있을 수 있는 쓰레기 바이트 제거
  delay(30);
  while (Serial1.available()) { (void)Serial1.read(); }

  // (디버깅) USB CDC도 깨끗이
  #if ENABLE_SERIAL_MONITOR_RESET
  delay(10);
  while (Serial.available()) { (void)Serial.read(); }
  #endif
}

void rs485_report()
{
  if (millis() - lastReportTime_1 < 5000)
    return;
  lastReportTime_1 = millis();

  // BLE 연결 상태
  bool bleConnected = getBleConnectionState();
  Serial1.print("ST,0,BMS_STATION_CONNECTED,");
  Serial1.print(bleConnected ? "1" : "0");
  Serial1.println(",ED");

  if(bleConnected == 0){
     setRelay(false);
     Serial.println("RS485: robot relay OFF");
  }
  // //배터리 Full 여부
  // Serial1.print("ST,0,BMS_STATION_BAT_FULL,");
  // Serial1.print(getBatteryFullStatus() ? "1" : "0");
  // Serial1.println(",ED");

  // //충전 중 여부 (Station과 Robot이 통전 중)
  // Serial1.print("ST,0,BMS_STATION_BAT_CHARGING,");
  // Serial1.print(getChargerOkStatus() ? "1" : "0");
  // Serial1.println(",ED");

  // //충전 준비 완료 (Station의 Jumper 릴레이 상태)
  // Serial1.print("ST,0,BMS_STATION_BAT_ON,");
  // Serial1.print(getChargerRelayStatus() ? "1" : "0");
  // Serial1.println(",ED");

  // //도킹 상태 (Station의 도킹 센서 상태)
  // Serial1.print("ST,0,BMS_STATION_DOCKED,");
  // Serial1.print(getDockingStatus() ? "1" : "0");
  // Serial1.println(",ED");

  if (!bleConnected)
    return; // 연결 안됐으면 여기서 종료
}

void rs485_run()
{
  // ===== RS485 (Serial1) 수신 =====
  while (Serial1.available())
  {
    char c = Serial1.read();

    // CR은 무시, LF에서 한 줄 처리
    if (c == '\r') continue;

    if (c == '\n')
    {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        processFrameLine(inputBuffer);
      }
      inputBuffer = ""; // Clear buffer after processing
    }
    else
    {
      inputBuffer += c;
      // (보호) 비정상적으로 너무 길어지면 초기화
      if (inputBuffer.length() > 128) inputBuffer = "";
    }
  }

#if ENABLE_SERIAL_MONITOR_RESET
  // ===== Serial Monitor(USB Serial) 수신 - 테스트 용 =====
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      monitorInputBuffer.trim();
      if (monitorInputBuffer.length() > 0) {
        processFrameLine(monitorInputBuffer);
      }
      monitorInputBuffer = ""; // Clear buffer after processing
    }
    else
    {
      monitorInputBuffer += c;
      if (monitorInputBuffer.length() > 128) monitorInputBuffer = "";
    }
  }
#endif
}
*/

#include <Arduino.h>
#include "robot_gpio.h"
#include "robot_ble.h"

// ====== 옵션: 시리얼 모니터에서 리셋 명령 허용 ======
#define ENABLE_SERIAL_MONITOR_RESET 1
// ================================================

unsigned long lastReportTime_1 = 0;

String inputBuffer;        // RS485(Serial1) 수신 버퍼
#if ENABLE_SERIAL_MONITOR_RESET
String monitorInputBuffer; // Serial(USB) 수신 버퍼 - 테스트 용
#endif

// BLE 연결 전환 감지(Connected -> Disconnected)
static bool prevBleConnected = false;

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

      // BLE 연결이 없으면 ON 명령은 무조건 무시 + 강제 OFF
      if (!getBleConnectionState()) {
        setRelay(false);
        Serial.println("RS485: BLE not connected -> ignore ON, force OFF");

        // 상위에도 0 보고(필요하면)
        Serial1.println("ST,0,BMS_STATION_BAT_ON,0,ED");
        Serial1.flush();
        return;
      }

      setRelay(true);
      Serial.println("RS485: robot relay ON");
      return;

    } else if (line.endsWith(",0,ED")) {

      setRelay(false);
      Serial.println("RS485: robot relay OFF");

      // OFF는 상위에 0 보고(원하면 유지)
      Serial1.println("ST,0,BMS_STATION_BAT_ON,0,ED");
      Serial1.flush();
      return;

    } else {
      Serial.println("RS485: Wrong relay command value");
      return;
    }
  }

  // === 하드리셋 명령 ===
  if (line.startsWith("ST,0,BMS_ROBOT_RESET,")) {
    if (line.endsWith(",1,ED")) {
      Serial.println("RS485: 하드리셋 명령 수신 → 리부트");
      hardResetNow();
    } else {
      Serial.println("RS485: 하드리셋 명령 무시(값이 1이 아님)");
    }
    return;
  }
}

void rs485_init()
{
  Serial1.begin(9600);
  delay(30);
  while (Serial1.available()) { (void)Serial1.read(); }

  #if ENABLE_SERIAL_MONITOR_RESET
  delay(10);
  while (Serial.available()) { (void)Serial.read(); }
  #endif

  prevBleConnected = getBleConnectionState();
}

void rs485_report()
{
  if (millis() - lastReportTime_1 < 5000) return;
  lastReportTime_1 = millis();

  bool bleConnected = getBleConnectionState();

  Serial1.print("ST,0,BMS_STATION_CONNECTED,");
  Serial1.print(bleConnected ? "1" : "0");
  Serial1.println(",ED");

  // Connected -> Disconnected 전환 순간: 즉시 OFF + 0 프레임 1회
  if (prevBleConnected && !bleConnected) {
    setRelay(false);
    Serial.println("RS485: BLE disconnected -> robot relay OFF");

    Serial1.println("ST,0,BMS_STATION_BAT_ON,0,ED");
    Serial1.flush();
  }

  prevBleConnected = bleConnected;

  if (!bleConnected) return;
}

void rs485_run()
{
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      inputBuffer.trim();
      if (inputBuffer.length() > 0) {
        processFrameLine(inputBuffer);
      }
      inputBuffer = "";
    }
    else
    {
      inputBuffer += c;
      if (inputBuffer.length() > 128) inputBuffer = "";
    }
  }

#if ENABLE_SERIAL_MONITOR_RESET
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      monitorInputBuffer.trim();
      if (monitorInputBuffer.length() > 0) {
        processFrameLine(monitorInputBuffer);
      }
      monitorInputBuffer = "";
    }
    else
    {
      monitorInputBuffer += c;
      if (monitorInputBuffer.length() > 128) monitorInputBuffer = "";
    }
  }
#endif
}
