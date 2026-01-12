/*#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = false;      // RELAY_PIN2(D4)의 현재 상태
bool isDisconnected = false;   // 단선 상태 플래그
unsigned long lastPrintTime = 0;

// === 전역 변수 (station_gpio.cpp 상단에 위치) ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03; // 더 작을수록 반응이 느리지만 안정적

// === 전압 조건 ===
const float DISCONNECT_V          = 0.600;   // 단선 판단 기준
const float CHARGE_START_MIN_V    = 0.850;   // 충전 시작 하한
const float CHARGE_START_MAX_V    = 1.275;   // 충전 시작 상한
const float CHARGE_STOP_V         = 1.325;   // 과충전 판단 기준

// === 부팅 부스팅(전원 ON 직후 10초 강제 ON) ===
const unsigned long BOOT_ASSIST_MS = 10000;  // 부스팅 지속시간 (10s)
bool bootAssistActive = false;
unsigned long bootAssistStart = 0;

// === 필터링된 전압 읽기 함수 ===
float getFilteredVoltage()
{
  int raw = analogRead(ADC_PIN);

  // 이상치 제거 (하드웨어 노이즈 등)
  if (raw < 100 || raw > 1023)
    return smoothedVoltage;

  float voltage = (raw / 1023.0) * 3.3;

  if (firstSample)
  {
    smoothedVoltage = voltage;
    firstSample = false;
  }
  else
  {
    // 전압 급락 시 즉시 반영
    if (voltage < smoothedVoltage - 0.2) {
      smoothedVoltage = voltage;
    } else {
      smoothedVoltage = alpha * voltage + (1.0 - alpha) * smoothedVoltage;
    }
  }

  return smoothedVoltage;
}

void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(RELAY_PIN2, OUTPUT);        // D4
  digitalWrite(RELAY_PIN2, LOW);      // 기본 OFF 상태
  relay2State = false;

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(ADC_PIN, INPUT);            // A0

  Serial.println("GPIO Initialized");

  // === 전원 ON 직후 10초간 부스팅 시작 ===
  digitalWrite(RELAY_PIN2, HIGH);
  relay2State = true;
  bootAssistActive = true;
  bootAssistStart = millis();
  isDisconnected = false; // 부스팅 동안 단선 판정 보류
  Serial.println("BOOT-ASSIST: Relay2 ON (10s)");
}

void gpio_run() {
  // 상태 LED
  digitalWrite(BUILTIN_LED, currentState == DOCKING_OK ? HIGH : LOW);

  float voltage = getFilteredVoltage();
  unsigned long now = millis();

  // === 부팅 부스팅 단계 처리 ===
  if (bootAssistActive) {
    // 안전장치: 부스팅 중 과충전은 즉시 차단
    if (voltage >= CHARGE_STOP_V) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      bootAssistActive = false;
      Serial.println("BOOT-ASSIST: Over-voltage → Relay2 OFF");
    } else if (now - bootAssistStart >= BOOT_ASSIST_MS) {
      // 10초 경과 → 일반 로직으로 복귀
      bootAssistActive = false;
      Serial.println("BOOT-ASSIST: End → handover to normal logic");
    }

    // 1초마다 상태 출력
    if (now - lastPrintTime >= 1000) {
      lastPrintTime = now;
      Serial.print("[ADC] A0 Voltage: ");
      Serial.print(voltage, 3);
      Serial.print(" V | Relay2: ");
      Serial.print(relay2State ? "ON" : "OFF");
      Serial.println(" | BootAssist:Y");
    }
    return; //부스팅 중에는 일반 로직을 실행하지 않음(단선 오탐 방지)
  }

  // === 일반 로직 시작 ===

  // 단선 감지 (Relay 상태와 관계 없이)
  if (voltage <= DISCONNECT_V) {
    if (!isDisconnected) {
      isDisconnected = true;
      if (relay2State) {
        digitalWrite(RELAY_PIN2, LOW);
        relay2State = false;
      }
      Serial.println("단선 감지 → Relay2 OFF");
    }
  } else {
    // 단선 아님 → 플래그 해제
    isDisconnected = false;

    // Relay ON 상태일 때
    if (relay2State) {
      if (voltage >= CHARGE_STOP_V) {
        digitalWrite(RELAY_PIN2, LOW);
        relay2State = false;
        Serial.println("과충전 감지 → Relay2 OFF");
      }
    }
    // Relay OFF 상태일 때
    else {
      if (voltage >= CHARGE_START_MIN_V && voltage <= CHARGE_START_MAX_V) {
        digitalWrite(RELAY_PIN2, HIGH);
        relay2State = true;
        Serial.println("충전 조건 만족 → Relay2 ON");
      }
    }
  }

  // 1초마다 상태 출력
  if (now - lastPrintTime >= 1000) {
    lastPrintTime = now;
    Serial.print("[ADC] A0 Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Relay2: ");
    Serial.println(relay2State ? "ON" : "OFF");
  }
}
*/

#include <Arduino.h>
#include "station_gpio.h"
#include "station_fsm.h"

// ======================= 기본 상태 =======================
bool relay2State = false;      // RELAY_PIN2(D4)의 현재 상태
bool isDisconnected = false;   // 단선 상태 플래그
unsigned long lastPrintTime = 0;

// ======================= ADC 필터 =======================
float smoothedVoltage = 0.0;   // 입력전압(Vin) 기준으로 필터링
bool firstSample = true;
const float alpha = 0.03;      // 더 작을수록 반응 느림/안정

// ======================= 분압/보정 =======================
const float R_TOP = 270000.0f;
const float R_BOT =  30000.0f;
const float VIN_SCALE = (R_TOP + R_BOT) / R_BOT; // = 10.0

const float ADC_VREF_V = 3.27f;

// 보정
const float VIN_OFFSET = 0.04f;

// ======================= 전압 조건 (Vin 기준) =======================
const float DISCONNECT_V          = 6.0;     // 단선/비정상
const float CHARGE_START_MIN_V    = 19.0;    // 충전 시작 하한
const float CHARGE_START_MAX_V    = 27.5;    // 충전 시작 상한
const float CHARGE_STOP_V         = 29.5;    // 과전압 차단

// ======================= RS485 옵션 =======================
// 필요 없으면 0으로 내려도 됨
#define ENABLE_RS485 1

#if ENABLE_RS485
  #define RS485_BAUD     9600
  #define RS485_DIR_PIN  2       // DE+ /RE
  static inline void rs485SetRx() { digitalWrite(RS485_DIR_PIN, LOW); }
  static inline void rs485SetTx() { digitalWrite(RS485_DIR_PIN, HIGH); }

  // 수신 직후 일정 시간 동안은 송신 금지(충돌/머리 잘림 방지)
  static unsigned long lastRs485RxByteMs = 0;
  static inline bool rs485BusQuiet(unsigned long quietMs = 30) {
    return (millis() - lastRs485RxByteMs) >= quietMs;
  }

  // TX 큐(간단하게 4개만)
  static String txQ[4];
  static uint8_t txHead = 0, txTail = 0;

  static inline bool txQEmpty() { return txHead == txTail; }
  static inline bool txQFull()  { return (uint8_t)(txTail + 1) % 4 == txHead; }

  static void rs485QueueLine(const String& line) {
    if (txQFull()) {
      txHead = (uint8_t)(txHead + 1) % 4;
    }
    txQ[txTail] = line;
    txTail = (uint8_t)(txTail + 1) % 4;
  }

  static void rs485TrySend() {
    if (txQEmpty()) return;
    if (!rs485BusQuiet(30)) return;

    const String& line = txQ[txHead];

    rs485SetTx();
    delayMicroseconds(200);
    Serial1.print(line);
    Serial1.print("\r\n");
    Serial1.flush();
    delayMicroseconds(200);
    rs485SetRx();

    txHead = (uint8_t)(txHead + 1) % 4;
  }

  static void rs485SendStatusFrame(const char* label, uint8_t value) {
    String s = "ST,0,";
    s += label;
    s += ",";
    s += String((int)value);
    s += ",ED";
    rs485QueueLine(s);
  }

  static void rs485SendAck(const String& label, int value) {
    // ACK,<LABEL>,<VALUE>,ED
    String s = "ACK,";
    s += label;
    s += ",";
    s += String(value);
    s += ",ED";
    rs485QueueLine(s);
  }

  // 스트림 파싱
  static String rs485RxStream;

  static void processRs485Frame(const String& frame);

  static void rs485ParseStream() {
    while (true) {
      int st = rs485RxStream.indexOf("ST,0,");
      if (st < 0) {
        // 꼬리만
        if (rs485RxStream.length() > 64)
          rs485RxStream.remove(0, rs485RxStream.length() - 64);
        return;
      }
      if (st > 0) rs485RxStream.remove(0, st);

      int ed = rs485RxStream.indexOf(",ED");
      if (ed < 0) return; // 덜 들어옴

      String one = rs485RxStream.substring(0, ed + 3); // ",ED" 포함
      rs485RxStream.remove(0, ed + 3);

      one.trim();
      if (one.length() > 0) processRs485Frame(one);
    }
  }

  static void rs485Run() {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      lastRs485RxByteMs = millis();

      // CR/LF는 버리고 내용만 누적(프레임 끝은 ,ED)
      if (c == '\r' || c == '\n') continue;

      rs485RxStream += c;
      if (rs485RxStream.length() > 256)
        rs485RxStream.remove(0, rs485RxStream.length() - 256);

      rs485ParseStream();
    }

    rs485TrySend();
  }
#endif

// ======================= 입력전압(Vin) 필터 =======================
float getFilteredVoltage()
{
  int raw = analogRead(ADC_PIN);
  const float ADC_MAX = 1023.0f;

  float vadc = (raw / ADC_MAX) * ADC_VREF_V;
  float vin  = vadc * VIN_SCALE;

  vin += VIN_OFFSET;
  if (vin < 0.0f) vin = 0.0f;

  if (firstSample) {
    smoothedVoltage = vin;
    firstSample = false;
  } else {
    // 급락
    if (vin < smoothedVoltage - 2.0f) {
      smoothedVoltage = vin;
    } else {
      smoothedVoltage = alpha * vin + (1.0f - alpha) * smoothedVoltage;
    }
  }
  return smoothedVoltage;
}

// ======================= 릴레이 변경 =======================
static void reportRelay2Change(bool newState, const char* reason)
{
  Serial.print(reason);
  Serial.print(" | Relay2: ");
  Serial.print(newState ? "1(HIGH)" : "0(LOW)");
  Serial.println();

#if ENABLE_RS485
  rs485SendStatusFrame("BMS_STATION_BAT_ON", newState ? 1 : 0);
#endif
}

static void setRelay2(bool on, const char* reason)
{
  if (relay2State == on) return;

  digitalWrite(RELAY_PIN2, on ? HIGH : LOW);
  relay2State = on;

  reportRelay2Change(on, reason);
}

// ======================= RS485 프레임 처리(ACK 포함) =======================
#if ENABLE_RS485
static void processRs485Frame(const String& frame)
{
  // frame: "ST,0,BMS_ROBOT_CTRL_BAT_ON,1,ED"
  // payload = "BMS_ROBOT_CTRL_BAT_ON,1"
  String payload = frame;
  payload.remove(0, 5); // "ST,0," 제거
  // 뒤 ",ED" 제거
  if (payload.endsWith(",ED")) payload.remove(payload.length() - 3);

  int comma = payload.indexOf(',');
  if (comma < 0) return;

  String label = payload.substring(0, comma);
  String vstr  = payload.substring(comma + 1);
  label.trim();
  vstr.trim();

  int value = vstr.toInt();

  // ACK
  rs485SendAck(label, value);

  if (label == "BMS_ROBOT_CTRL_BAT_ON" || label == "BMS_STATION_CTRL_BAT_ON") {
    if (value == 1) {
      setRelay2(true,  "RS485 CMD -> Relay2 ON");
    } else if (value == 0) {
      setRelay2(false, "RS485 CMD -> Relay2 OFF");
    }
  }
}
#endif

// ======================= GPIO init/run =======================
void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(RELAY_PIN2, OUTPUT);
  digitalWrite(RELAY_PIN2, LOW);
  relay2State = false;

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(ADC_PIN, INPUT);

  Serial.println("GPIO Initialized");

#if ENABLE_RS485
  pinMode(RS485_DIR_PIN, OUTPUT);
  rs485SetRx();
  Serial1.begin(RS485_BAUD);
  delay(30);
  while (Serial1.available()) (void)Serial1.read();
#endif
}

void gpio_run() {
  // RS485 먼저 돌려서 RX 밀리지 않게
#if ENABLE_RS485
  rs485Run();
#endif

  // 상태 LED
  digitalWrite(BUILTIN_LED, currentState == DOCKING_OK ? HIGH : LOW);

  float voltage = getFilteredVoltage();
  unsigned long now = millis();

  // 단선 -> 과전압 -> 충전조건 ===
  if (voltage <= DISCONNECT_V) {
    if (!isDisconnected) {
      isDisconnected = true;
      if (relay2State) {
        setRelay2(false, "단선 감지 -> Relay2 OFF");
      } else {
      }
    }
  } else {
    isDisconnected = false;

    if (relay2State) {
      if (voltage >= CHARGE_STOP_V) {
        setRelay2(false, "과전압 감지 -> Relay2 OFF");
      }
    } else {
      if (voltage >= CHARGE_START_MIN_V && voltage <= CHARGE_START_MAX_V) {
        setRelay2(true, "충전 조건 만족 -> Relay2 ON");
      }
    }
  }

  // 1초마다 상태 출력 (0/1 표기)
  if (now - lastPrintTime >= 1000) {
    lastPrintTime = now;
    Serial.print("[ADC] VIN: ");
    Serial.print(voltage, 2);
    Serial.print(" V | Relay2: ");
    Serial.print(relay2State ? "1(HIGH)" : "0(LOW)");
    Serial.println();
  }
}
