/*#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = false;  // RELAY_PIN2(D4)의 현재 상태
unsigned long lastPrintTime = 0;

// === 전역 변수 (station_gpio.cpp 상단에 위치) ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03; // 더 작을수록 반응이 느리지만 안정적

const float RELAY_ON_MIN_V = 1.088;  // 45.0V
const float RELAY_ON_MAX_V = 1.245;  // 50.0V
const float RELAY_OFF_FULL_V = 1.300; // 53.0V

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
    smoothedVoltage = alpha * voltage + (1.0 - alpha) * smoothedVoltage;
  }

  return smoothedVoltage;
}

void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(RELAY_PIN2, OUTPUT);        // D4
  digitalWrite(RELAY_PIN2, LOW);     // 기본 OFF 상태

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(ADC_PIN, INPUT);            // A0

  Serial.println("GPIO Initialized");
}

void gpio_run() {
  // LED 제어
  if (currentState == DOCKING_OK) {
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
    digitalWrite(BUILTIN_LED, LOW);
  }

  // === 필터링된 전압 기반 릴레이 제어 ===
  float voltage = getFilteredVoltage();

  if (relay2State) {
    // 릴레이 ON 상태일 때 → OFF 조건 확인
    if (voltage >= RELAY_OFF_FULL_V) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      Serial.println("A0 >= 1.320V (53V) → Relay2 OFF (FULL CHARGED)");
    }
    else if (voltage < RELAY_ON_MIN_V) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      Serial.println("A0 < 1.120V (45V) → Relay2 OFF (LOW VOLTAGE)");
    }
  } else {
    // 릴레이 OFF 상태일 때 → ON 조건 확인
    if (voltage >= RELAY_ON_MIN_V && voltage <= RELAY_ON_MAX_V) {
      digitalWrite(RELAY_PIN2, HIGH);
      relay2State = true;
      Serial.println("A0 1.120V~1.245V (45V~50V) → Relay2 ON (Charging)");
    }
  }

  // 3초마다 상태 출력
  if (millis() - lastPrintTime >= 3000) {
    lastPrintTime = millis();
    Serial.print("[ADC] A0 Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Relay2: ");
    Serial.println(relay2State ? "ON" : "OFF");
  }
}*/

#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = false;  // RELAY_PIN2(D4)의 현재 상태
unsigned long lastPrintTime = 0;

// === 전역 변수 (station_gpio.cpp 상단에 위치) ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03; // 더 작을수록 반응이 느리지만 안정적

// === 전압 조건 ===
const float CHARGE_ON_MIN_V        = 1.088;     // 45.0V
const float CHARGE_ON_MAX_V        = 1.285;     // 52.2V
const float CHARGE_OFF_CLOSE_V     = 1.310;     // Remote CLOSE 기준 과충전 OFF
const float CHARGE_BLOCK_OPEN_V    = 1.296;     // Remote OPEN 기준 과충전 유지
const float LOW_VOLTAGE_WHILE_ON   = 1.05;      // CLOSE 중 낮은 전압 → 단선 판단

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
    smoothedVoltage = alpha * voltage + (1.0 - alpha) * smoothedVoltage;
  }

  return smoothedVoltage;
}

void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(RELAY_PIN2, OUTPUT);        // D4
  digitalWrite(RELAY_PIN2, LOW);     // 기본 OFF 상태

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(ADC_PIN, INPUT);            // A0

  Serial.println("GPIO Initialized");
}

void gpio_run() {
  // LED 제어
  if (currentState == DOCKING_OK) {
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
    digitalWrite(BUILTIN_LED, LOW);
  }

  float voltage = getFilteredVoltage();

  // === Remote 상태에 따른 기준 전압 조정 ===
  float overVoltageThreshold = relay2State ? CHARGE_OFF_CLOSE_V : CHARGE_BLOCK_OPEN_V;

  if (relay2State) {
    // 🔸 충전 중일 때
    if (voltage >= overVoltageThreshold) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      Serial.println("과충전 감지 → Relay2 OPEN");
    } else if (voltage < LOW_VOLTAGE_WHILE_ON) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      Serial.println("전압 낮음 → 단선 판단, Relay2 OPEN");
    }
  } else {
    // 🔹 충전 중이 아닐 때
    if (voltage >= overVoltageThreshold) {
      // 과충전 상태 유지 → 충전 금지
      // 아무 동작 없음
    } else if (voltage >= CHARGE_ON_MIN_V && voltage <= CHARGE_ON_MAX_V) {
      digitalWrite(RELAY_PIN2, HIGH);
      relay2State = true;
      Serial.println("충전 시작 → Relay2 CLOSE (ON)");
    }
  }

  // 3초마다 상태 출력
  if (millis() - lastPrintTime >= 3000) {
    lastPrintTime = millis();
    Serial.print("[ADC] A0 Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Relay2: ");
    Serial.println(relay2State ? "ON" : "OFF");
  }
}
