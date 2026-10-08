#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// ===== Station 전용 설정 =====
// ============================================================

// ===== GPIO pin map =====
#define DOCKING_PIN      8
#define BATTERY_FULL_PIN 5
#define CHARGER_OK_PIN   6
#define RELAY_PIN2       4
#define RELAY_PIN        7
#define BUILTIN_LED      LED_BUILTIN
#define ADC_PIN          A0

// ===== 전압 기준값 =====
#define VOLT_DISCONNECT_THRESHOLD       (0.600f)
#define VOLT_CHARGE_START_MIN           (0.850f)
#define VOLT_CHARGE_START_MAX           (1.275f)
#define VOLT_CHARGE_STOP_THRESHOLD      (1.325f)
#define VOLT_HYSTERESIS_DB              (0.050f)

// ===== 타이머 =====
#define BOOT_ASSIST_DURATION_MS        (10000UL)
#define DOCK_LOW_HARD_RESET_MS         (15UL * 60UL * 1000UL)
#define DOCK_OK_DELAY_MS               (3000)
#define ADC_FILTER_UPDATE_MS           (50)
#define ADC_WINDOW_SIZE                (8)
#define ADC_EMA_ALPHA                  (0.10f)

// ===== BLE 인증 =====
#define AUTH_TIMEOUT_MS                (5000)
#define AUTH_RETRY_LIMIT               (3)
#define AUTH_HARDRESET_LIMIT           (10)

// ===== 로그 =====
#define LOG_ERROR(fmt, ...) do { Serial.print("[ERROR] "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)
#define LOG_WARN(fmt, ...)  do { Serial.print("[WARN]  "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)
#define LOG_INFO(fmt, ...)  do { Serial.print("[INFO]  "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)
#define LOG_DEBUG(fmt, ...) do { Serial.print("[DEBUG] "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)

#endif // CONFIG_H
