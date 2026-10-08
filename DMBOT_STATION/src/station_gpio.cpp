#include "station_gpio.h"
#include "station_fsm.h"
#include "config.h"

bool relay2State = false;
bool isDisconnected = false;
unsigned long lastPrintTime = 0;

/* ===== ORIGINAL CODE (기존 코드 주석 처리) =====
// float smoothedVoltage = 0.0;
// bool firstSample = true;
// const float alpha = 0.03;
// float getFilteredVoltage() { ... }
===== END ORIGINAL CODE ===== */

// ===== IMPROVED ADC FILTER (개선된 ADC 필터) =====
// Sliding window + Median + EMA 방식으로 노이즈 제거
struct AdcFilter {
  int raw[ADC_WINDOW_SIZE];
  uint8_t idx = 0;
  uint8_t count = 0;
  float smoothed = 0.0f;
  unsigned long lastUpdateMs = 0;
};

static AdcFilter adcFilter = {};

float getFilteredVoltage() {
  unsigned long now = millis();

  // 50ms마다 한 번만 업데이트
  if (now - adcFilter.lastUpdateMs < ADC_FILTER_UPDATE_MS) {
    return adcFilter.smoothed;
  }
  adcFilter.lastUpdateMs = now;

  int raw = analogRead(ADC_PIN);

  // 이상치 제거 (하드웨어 노이즈, 핀 오류 등)
  if (raw < 50 || raw > 1023) {
    return adcFilter.smoothed;
  }

  // 슬라이딩 윈도우에 샘플 추가
  adcFilter.raw[adcFilter.idx] = raw;
  adcFilter.idx = (adcFilter.idx + 1) % ADC_WINDOW_SIZE;
  if (adcFilter.count < ADC_WINDOW_SIZE) {
    adcFilter.count++;
  }

  // 중앙값(Median) 계산 - 이상치 제거
  int sorted[ADC_WINDOW_SIZE];
  memcpy(sorted, adcFilter.raw, adcFilter.count * sizeof(int));

  // 정렬 (bubble sort)
  for (uint8_t i = 0; i < adcFilter.count - 1; ++i) {
    for (uint8_t j = i + 1; j < adcFilter.count; ++j) {
      if (sorted[j] < sorted[i]) {
        int tmp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = tmp;
      }
    }
  }

  int median = sorted[adcFilter.count / 2];
  float voltage = (median / 1023.0f) * 3.3f;

  // EMA(Exponential Moving Average) 필터
  if (adcFilter.count == 1) {
    adcFilter.smoothed = voltage;
  } else {
    adcFilter.smoothed = ADC_EMA_ALPHA * voltage + (1.0f - ADC_EMA_ALPHA) * adcFilter.smoothed;
  }

  return adcFilter.smoothed;
}

// ===== 충전 제어 파라미터 =====
const float DISCONNECT_V = VOLT_DISCONNECT_THRESHOLD;
const float CHARGE_START_MIN_V = VOLT_CHARGE_START_MIN;
const float CHARGE_START_MAX_V = VOLT_CHARGE_START_MAX;
const float CHARGE_STOP_V = VOLT_CHARGE_STOP_THRESHOLD;

const unsigned long BOOT_ASSIST_MS = BOOT_ASSIST_DURATION_MS;
bool bootAssistActive = false;
unsigned long bootAssistStart = 0;

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

  LOG_INFO("GPIO initialized");

  // === 부팅 직후 10초 강제 ON (부스팅) ===
  digitalWrite(RELAY_PIN2, HIGH);
  relay2State = true;
  bootAssistActive = true;
  bootAssistStart = millis();
  isDisconnected = false;
  LOG_INFO("BOOT-ASSIST: Relay2 ON (10s)");
}

void gpio_run() {
  // === LED 상태 표시 ===
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  unsigned long now = millis();

  if (currentState == DOCKING_OK) {
    // HIGH 고정 (상시 켜짐)
    digitalWrite(BUILTIN_LED, HIGH);
  } else if (currentState == CONNECTED) {
    // 빠른 점멸 (150ms)
    if (now - lastBlink >= 150) {
      lastBlink = now;
      blinkState = !blinkState;
      digitalWrite(BUILTIN_LED, blinkState ? HIGH : LOW);
    }
  } else if (currentState == ADVERTISING) {
    // 느린 점멸 (500ms)
    if (now - lastBlink >= 500) {
      lastBlink = now;
      blinkState = !blinkState;
      digitalWrite(BUILTIN_LED, blinkState ? HIGH : LOW);
    }
  } else {
    // IDLE, CONNECTING: OFF
    digitalWrite(BUILTIN_LED, LOW);
  }

  float voltage = getFilteredVoltage();

  // === 부팅 부스팅 단계 처리 ===
  if (bootAssistActive) {
    // 부스팅 중 과충전 감지 시 즉시 차단
    if (voltage >= CHARGE_STOP_V) {
      digitalWrite(RELAY_PIN2, LOW);
      relay2State = false;
      bootAssistActive = false;
      LOG_WARN("BOOT-ASSIST: Over-voltage -> Relay2 OFF");
    }
    // 10초 경과 후 정상 로직으로 복귀
    else if (now - bootAssistStart >= BOOT_ASSIST_MS) {
      bootAssistActive = false;
      LOG_INFO("BOOT-ASSIST: End -> handover to normal logic");
    }

    // 1초마다 상태 출력
    if (now - lastPrintTime >= 1000) {
      lastPrintTime = now;
      Serial.print("[ADC] Voltage: ");
      Serial.print(voltage, 3);
      Serial.print("V | Relay2: ");
      Serial.print(relay2State ? "ON" : "OFF");
      Serial.println(" | BootAssist:Y");
    }
    return;  // 부스팅 중에는 정상 로직 실행 금지
  }

  // === 정상 제어 로직 ===

  // 단선 감지
  if (voltage <= DISCONNECT_V) {
    if (!isDisconnected) {
      isDisconnected = true;
      if (relay2State) {
        digitalWrite(RELAY_PIN2, LOW);
        relay2State = false;
      }
      LOG_WARN("Disconnection detected -> Relay2 OFF");
    }
  }
  // 단선이 아님
  else {
    isDisconnected = false;

    // Relay ON 상태일 때: 과충전 감지
    if (relay2State) {
      if (voltage >= CHARGE_STOP_V) {
        digitalWrite(RELAY_PIN2, LOW);
        relay2State = false;
        LOG_WARN("Over-charge detected -> Relay2 OFF");
      }
    }
    // Relay OFF 상태일 때: 충전 시작 조건 확인
    else {
      if (voltage >= CHARGE_START_MIN_V && voltage <= CHARGE_START_MAX_V) {
        digitalWrite(RELAY_PIN2, HIGH);
        relay2State = true;
        LOG_INFO("Charge condition met -> Relay2 ON");
      }
    }
  }

  // 1초마다 상태 출력
  if (now - lastPrintTime >= 1000) {
    lastPrintTime = now;
    Serial.print("[ADC] Voltage: ");
    Serial.print(voltage, 3);
    Serial.print("V | Relay2: ");
    Serial.print(relay2State ? "ON" : "OFF");
    Serial.print(" | State: ");
    switch (currentState) {
      case IDLE: Serial.println("IDLE"); break;
      case DOCKING_OK: Serial.println("DOCKING_OK"); break;
      case ADVERTISING: Serial.println("ADVERTISING"); break;
      case CONNECTING: Serial.println("CONNECTING"); break;
      case CONNECTED: Serial.println("CONNECTED"); break;
      default: Serial.println("UNKNOWN"); break;
    }
  }
}
