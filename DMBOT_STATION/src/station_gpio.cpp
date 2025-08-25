#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = false;  // RELAY_PIN2(D4)의 현재 상태
bool isDisconnected = false;  // 단선 상태 플래그
unsigned long lastPrintTime = 0;

// === 전역 변수 (station_gpio.cpp 상단에 위치) ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03; // 더 작을수록 반응이 느리지만 안정적

// === 전압 조건 ===
const float DISCONNECT_V             = 0.600;   // 단선 판단 기준
const float CHARGE_START_MIN_V      = 0.850;   // 충전 시작 하한
const float CHARGE_START_MAX_V      = 1.275;   // 충전 시작 상한
const float CHARGE_STOP_V           = 1.330;   // 과충전 판단 기준


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

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);

  pinMode(ADC_PIN, INPUT);            // A0

  Serial.println("GPIO Initialized");
}

void gpio_run() {
  // 상태 LED
  digitalWrite(BUILTIN_LED, currentState == DOCKING_OK ? HIGH : LOW);

  float voltage = getFilteredVoltage();

  // === 단선 감지 (Relay 상태와 관계 없이) ===
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

    // === Relay ON 상태일 때 ===
    if (relay2State) {
      if (voltage >= CHARGE_STOP_V) {
        digitalWrite(RELAY_PIN2, LOW);
        relay2State = false;
        Serial.println("과충전 감지 → Relay2 OFF");
      }
    }
    // === Relay OFF 상태일 때 ===
    else {
      if (voltage >= CHARGE_START_MIN_V && voltage <= CHARGE_START_MAX_V) {
        digitalWrite(RELAY_PIN2, HIGH);
        relay2State = true;
        Serial.println("충전 조건 만족 → Relay2 ON");
      }
    }
  }

  // 1초마다 상태 출력
  if (millis() - lastPrintTime >= 1000) {
    lastPrintTime = millis();
    Serial.print("[ADC] A0 Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Relay2: ");
    Serial.println(relay2State ? "ON" : "OFF");
  }
}
