#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================
// ===== 공통 설정값 (매직 넘버 제거) =====
// ============================================================

// ===== BLE RSSI 기준값 (Robot) =====
// 환경에 따라 조정 필요:
//   - EOS 환경: -70 dBm
//   - TP 환경: -50 dBm
//   - 일반 실내: -80 dBm
#define BLE_SCAN_RSSI_THRESHOLD_DBM         (-80)      // 스캔 중 연결 기준값
#define BLE_SCAN_CONSEC_GOOD_FRAMES         (10)       // 연속 프레임 수
#define BLE_CONNECTED_RSSI_BAD_DBM          (-95)      // 연결 해제 기준값
#define BLE_CONNECTED_CONSEC_BAD_FRAMES     (5)        // 연속 약신호 프레임 수

// ===== 인증 관련 설정 (IMPROVED) =====
// 기존: 3회 실패 시 즉시 하드리셋 → 위험
// 개선: 3회 소프트리셋 → 10회 하드리셋 (더 관대)
#define AUTH_FAIL_SOFT_RESET_THRESHOLD      (3)        // 소프트 리셋 기준
#define AUTH_FAIL_HARD_RESET_THRESHOLD      (10)       // 하드 리셋 기준
#define AUTH_FAIL_DECAY_MS                  (300000UL) // 5분 후 카운터 리셋 (기존: 2분)
#define AUTH_TIMEOUT_MS                     (5000)     // 인증 타임아웃 5초

// ===== 타이머 =====
#define SCAN_STALL_MS                       (8000)     // 스캔 정지 감지
#define MIN_RESTART_GAP_MS                  (1500)     // 스캔 재시작 최소 간격
#define POST_EVENT_GRACE_MS                 (6000)     // 이벤트 후 유예시간
#define TARGET_ADV_MISS_MS                  (3000)     // 타깃 광고 미수신 기준
#define HARD_RESET_NO_ADV_MS                (30UL * 60UL * 1000UL)  // 30분
#define RSSI_UPDATE_MS                      (200)      // RSSI 샘플 주기
#define BLE_STATE_SYNC_MS                   (100)      // BLE 상태 동기화 주기 (NEW)
#define REPORT_INTERVAL_MS                  (5000)     // 상태 보고 주기

// ===== Watchdog 설정 =====
#define WATCHDOG_TIMEOUT_MS                 (5000)     // 5초
#define LOOP_MAX_TIME_MS                    (100)      // 경고 기준 100ms

// ===== Station 전압 기준값 =====
// ADC: 10비트, 범위 0-3.3V
// 분압 비: 1/41 (R1=200k, R2=5k)
// 실제 전압 = ADC_읽음값 * 41
#define VOLT_DISCONNECT_THRESHOLD           (0.600f)   // 단선 감지
#define VOLT_CHARGE_START_MIN               (0.850f)   // 충전 시작 하한
#define VOLT_CHARGE_START_MAX               (1.275f)   // 충전 시작 상한
#define VOLT_CHARGE_STOP_THRESHOLD          (1.325f)   // 과충전 차단
#define VOLT_HYSTERESIS_DB                  (0.050f)   // 히스테리시스

// ===== Station 타이머 =====
#define BOOT_ASSIST_DURATION_MS             (10000UL)  // 부팅 후 10초 부스팅
#define DOCK_LOW_HARD_RESET_MS              (15UL * 60UL * 1000UL)  // 15분 도킹 LOW
#define DOCK_OK_DELAY_MS                    (3000)     // 도킹 HIGH 유지 후 광고
#define ADC_FILTER_UPDATE_MS                (50)       // ADC 필터 업데이트 주기
#define ADC_WINDOW_SIZE                     (8)        // 슬라이딩 윈도우 크기
#define ADC_EMA_ALPHA                       (0.1f)     // EMA 계수

// ===== 로깅 매크로 =====
#define LOG_ERROR(fmt, ...) do { Serial.print("[ERROR] "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)
#define LOG_WARN(fmt, ...)  do { Serial.print("[WARN]  "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)
#define LOG_INFO(fmt, ...)  do { Serial.print("[INFO]  "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)
#define LOG_DEBUG(fmt, ...) do { Serial.print("[DEBUG] "); Serial.printf(fmt, ##__VA_ARGS__); Serial.println(); } while(0)

#endif // CONFIG_H
