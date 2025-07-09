#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

// === 상태 변수 ===
bool relay2State = true;  // RELAY_PIN2(D4)의 현재 상태
unsigned long lastPrintTime = 0;

// === 지수 이동 평균 관련 ===
float smoothedVoltage = 0.0;
bool firstSample = true;
const float alpha = 0.03;  // 더 작을수록 반응이 느리지만 안정적

// === 전압 필터 함수 (지수 평균 + 이상치 제거) ===
float getFilteredVoltage() {
  int raw = analogRead(ADC_PIN);
  
  // 이상치 제거 (하드웨어 노이즈 등)
  if (raw < 100 || raw > 1023) return smoothedVoltage;

  float voltage = (raw / 1023.0) * 3.3;

  if (firstSample) {
    smoothedVoltage = voltage;
    firstSample = false;
  } else {
    smoothedVoltage = alpha * voltage + (1.0 - alpha) * smoothedVoltage;
  }

  return smoothedVoltage;
}

// === GPIO 초기화 ===
void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // 기본 OFF

  pinMode(RELAY_PIN2, OUTPUT);        // D4
  digitalWrite(RELAY_PIN2, HIGH);     // 기본 ON

  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);     // 기본 OFF

  pinMode(ADC_PIN, INPUT);            // A0

  Serial.println("GPIO Initialized");
}

// === 메인 루프에서 주기적으로 호출 ===
void gpio_run() {
  // 상태 LED 제어
  if (currentState == DOCKING_OK) {
    digitalWrite(BUILTIN_LED, HIGH);
  } else {
    digitalWrite(BUILTIN_LED, LOW);
  }

  // 필터링된 전압 읽기
  float voltage = getFilteredVoltage();

  // === 예외: 전압 너무 낮음 → 연결 끊김 ===
  if (voltage <= 0.85) {
    if (relay2State) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      Serial.println("A0 <= 0.85V → Relay2 OFF (DISCONNECTED)");
    }
  }
  // === 릴레이 ON 조건 ===
  else if (voltage > 0.9 && voltage < 1.1654 && !relay2State) {
    digitalWrite(RELAY_PIN2, HIGH);
    relay2State = true;
    Serial.println("A0 > 0.9V && < 1.1654V → Relay2 ON");
  }
  // === 릴레이 OFF 조건 ===
  else if (voltage >= 1.28 && relay2State) {
    digitalWrite(RELAY_PIN2, LOW);
    relay2State = false;
    Serial.println("A0 >= 1.28V → Relay2 OFF");
  }

  // 3초마다 상태 출력
  if (millis() - lastPrintTime >= 3000) {
    lastPrintTime = millis();
    Serial.print("[ADC] A0 Voltage: ");
    Serial.print(voltage, 3);
    Serial.print(" V | Relay2: ");
    if (voltage <= 0.85) {
      Serial.println("DISCONNECTED");
    } else {
      Serial.println(relay2State ? "ON" : "OFF");
    }
  }
}
