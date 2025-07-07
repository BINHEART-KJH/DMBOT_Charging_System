#include "station_gpio.h"
#include "station_fsm.h"  // currentState 참조용

bool relay2State = true;  // RELAY_PIN2(D4)의 현재 상태
unsigned long lastPrintTime = 0;

void gpio_init() {
  pinMode(DOCKING_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(RELAY_PIN2, OUTPUT);        // D4
  digitalWrite(RELAY_PIN2, HIGH);     // 기본 ON 상태

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

  // A0 전압 기반으로 RELAY_PIN2 제어
  
  int adcValue = analogRead(ADC_PIN);
  float voltage = (adcValue / 1023.0) * 3.3;

  if (voltage > 2.54 && !relay2State) {
  digitalWrite(RELAY_PIN2, HIGH);   // 릴레이 ON
  relay2State = true;
  Serial.println("A0 > 2.54V → Relay2 ON");
} else if (voltage < 2.3 && relay2State) {
  digitalWrite(RELAY_PIN2, LOW);    // 릴레이 OFF
  relay2State = false;
  Serial.println("A0 < 2.3V → Relay2 OFF");
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