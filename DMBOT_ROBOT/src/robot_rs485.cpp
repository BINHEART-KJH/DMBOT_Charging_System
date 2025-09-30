#include <Arduino.h>
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
      Serial.println("RS485: 로봇 릴레이 ON (충전 시작)");
    } else if (line.endsWith(",0,ED")) {
      setRelay(false); // Relay OFF
      Serial.println("RS485: 로봇 릴레이 OFF (충전 중단)");
    } else {
      Serial.println("RS485: 잘못된 릴레이 명령 수신");
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

  // 🔗 BLE 연결 상태
  bool bleConnected = getBleConnectionState();
  Serial1.print("ST,0,BMS_STATION_CONNECTED,");
  Serial1.print(bleConnected ? "1" : "0");
  Serial1.println(",ED");

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
    return; // ❌ 연결 안됐으면 여기서 종료
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
 

/*  #include <Arduino.h>
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

// === BLE 상태 상승엣지 감지를 위한 이전 상태 저장 ===
static bool prevBleConnected = false;

// === RS485 TX 로그 헬퍼 ===
static void rs485_tx_kv(const char* key, int v) {
  // 프레임 생성
  String s = "ST,0,";
  s += key; s += ",";
  s += String(v);
  s += ",ED";

  // 실제 RS485 송신
  Serial1.println(s);

  // USB 시리얼로 미러 로그 (타임스탬프 포함)
  Serial.print("[RS485 TX "); Serial.print(millis()); Serial.print("] ");
  Serial.println(s);
}

// --- 하드리셋 헬퍼 ---
static inline void hardResetNow() {
  // 안전을 위해 릴레이 OFF 후 리셋
  setRelay(false);

  // (선택) 호스트/상위에 ACK 한번 통지 (TX 로그 포함)
  rs485_tx_kv("BMS_ROBOT_RESETTING", 1);
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
      Serial.print("[ACT "); Serial.print(millis()); Serial.println("] RS485: 로봇 릴레이 ON (충전 시작)");
    } else if (line.endsWith(",0,ED")) {
      setRelay(false); // Relay OFF
      Serial.print("[ACT "); Serial.print(millis()); Serial.println("] RS485: 로봇 릴레이 OFF (충전 중단)");
    } else {
      Serial.print("[WARN "); Serial.print(millis()); Serial.println("] RS485: 잘못된 릴레이 명령 수신");
    }
    return;
  }

  // === 하드리셋 명령 ===
  // ST,0,BMS_ROBOT_RESET,1,ED  -> 즉시 하드리셋
  if (line.startsWith("ST,0,BMS_ROBOT_RESET,")) {
    if (line.endsWith(",1,ED")) {
      Serial.print("[ACT "); Serial.print(millis()); Serial.println("] RS485: 하드리셋 명령 수신 → 리부트");
      hardResetNow();
      // 보통 여기로 돌아오지 않음
    } else {
      Serial.print("[WARN "); Serial.print(millis()); Serial.println("] RS485: 하드리셋 명령 무시(값이 1이 아님)");
    }
    return;
  }

  // === 기타 프레임 로깅 ===
  Serial.print("[INFO "); Serial.print(millis()); Serial.print("] RS485: unknown frame -> ");
  Serial.println(line);
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

  // 🔗 BLE 연결 상태
  bool bleConnected = getBleConnectionState();

  // ★ TX도 헬퍼로 보내서 콘솔에 [RS485 TX …] 로 찍히게
  rs485_tx_kv("BMS_STATION_CONNECTED", bleConnected ? 1 : 0);

  // --- (진단용) 필요 시 아래 상태도 임시 활성화해서 관측 ---
  // rs485_tx_kv("BMS_STATION_BAT_ON",   getChargerRelayStatus() ? 1 : 0);
  // rs485_tx_kv("BMS_STATION_DOCKED",   getDockingStatus() ? 1 : 0);
  // rs485_tx_kv("BMS_STATION_CHARGING", getChargerOkStatus() ? 1 : 0);
  // rs485_tx_kv("BMS_STATION_BAT_FULL", getBatteryFullStatus() ? 1 : 0);

  // ★ 핵심: BLE 0→1 상승엣지에서 DOCK 재주장 (엣지 없이도 CM4가 다시 판단하도록)
  if (bleConnected && !prevBleConnected) {
    rs485_tx_kv("BMS_STATION_DOCKED", getDockingStatus() ? 1 : 0);
    // 필요하면 헬로 프레임로 재동기화 트리거
    // rs485_tx_kv("HELLO", 1);
  }
  prevBleConnected = bleConnected;

  if (!bleConnected)
    return; // ❌ 연결 안됐으면 여기서 종료
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
        // ★ 원문 로그 (언제 무엇을 받았는지 항상 보인다)
        Serial.print("[RS485 RX "); Serial.print(millis()); Serial.print("] ");
        Serial.println(inputBuffer);

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
        // 모니터로도 동일하게 라인 처리 + 로그
        Serial.print("[MON RX "); Serial.print(millis()); Serial.print("] ");
        Serial.println(monitorInputBuffer);

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