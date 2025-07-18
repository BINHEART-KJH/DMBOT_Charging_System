#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = false;  // RELAY_PIN2(D4)의 현재 상태
unsigned long lastPrintTime = 0;

// === 전역 변수 (station_gpio.cpp 상단에 위치) ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03; // 더 작을수록 반응이 느리지만 안정적

// === A0 전압 조건 상수 (히스테리시스 적용, 분압비 약 40.17 기준) ===
//const float RELAY_ON_MIN_V = 1.120;     // 45.0V 이상
//const float RELAY_ON_MAX_V = 1.245;     // 50.0V 이하
//const float RELAY_OFF_FULL_V = 1.320;   // 53.0V 이상

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
}

/*
#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = false;       // RELAY_PIN2(D4)의 현재 상태
bool isDisconnected = false;    // 단선 or 과충전 상태
unsigned long disconnectTime = 0;
const unsigned long DISCONNECT_HOLD_MS = 5000;  // 단선 후 릴레이 차단 유지 시간

unsigned long lastPrintTime = 0;

// === 전역 변수 (station_gpio.cpp 상단에 위치) ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03; // 더 작을수록 반응이 느리지만 안정적

// === A0 전압 조건 상수 ===
const float RELAY_ON_MIN_V       = 1.088;  // 최소 45V일 때 ON 가능
const float RELAY_ON_MAX_V       = 1.235;  // 80% 이하 충전 시작 상한
const float RELAY_OFF_FULL_V     = 1.280;  // 90% 이상이면 OFF
const float DISCONNECTED_LOW_V   = 0.5;    // 단선 감지
const float DISCONNECTED_HIGH_V  = 2.8;    // 튐 감지
const float OVERCHARGE_V         = 1.310;  // 53.0V → 충전기 Remote OFF

// === 필터링된 전압 읽기 함수 ===
float getFilteredVoltage()
{
  int raw = analogRead(ADC_PIN);

  // 이상치 제거
  if (raw < 100 || raw > 1023)
    return smoothedVoltage;

  float voltage = (raw / 1023.0) * 3.3;

  if (firstSample) {
    smoothedVoltage = voltage;
    firstSample = false;
  } else {
    smoothedVoltage = alpha * voltage + (1.0 - alpha) * smoothedVoltage;
  }

  return smoothedVoltage;
}

void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(RELAY_PIN2, OUTPUT);
  digitalWrite(RELAY_PIN2, LOW);

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(ADC_PIN, INPUT);

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

  // === Disconnected 감지 ===
  if (voltage < DISCONNECTED_LOW_V || voltage > DISCONNECTED_HIGH_V || voltage >= OVERCHARGE_V) {
    if (relay2State || !isDisconnected) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      isDisconnected = true;
      disconnectTime = millis();
      Serial.println("A0 abnormal or >= 1.281V → Relay2 OFF (DISCONNECTED or OVERCHARGED)");
    }
  }

  // === Disconnected 해제 처리 ===
  if (isDisconnected) {
    if (millis() - disconnectTime < DISCONNECT_HOLD_MS) {
      return;  // 아직 HOLD 중
    } else {
      isDisconnected = false;
      if (voltage >= RELAY_ON_MIN_V && voltage <= RELAY_ON_MAX_V) {
        digitalWrite(RELAY_PIN2, HIGH);
        relay2State = true;
        Serial.println("Disconnected Hold Released → Relay2 ON (Charging Resumed)");
      } else {
        Serial.println("Disconnected Hold Released → Voltage not in range, stay OFF");
      }
      return;
    }
  }

  // === 정상 충전 제어 ===
  if (relay2State) {
    if (voltage >= RELAY_OFF_FULL_V) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      Serial.println("A0 >= 1.273V (≈90%) → Relay2 OFF (FULL CHARGED)");
    }
    // 유지 구간은 do nothing
  } else {
    if (voltage >= RELAY_ON_MIN_V && voltage <= RELAY_ON_MAX_V) {
      digitalWrite(RELAY_PIN2, HIGH);
      relay2State = true;
      Serial.println("A0 in 45V~80% range → Relay2 ON (Charging)");
    }
    // 중립 구간은 유지
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
