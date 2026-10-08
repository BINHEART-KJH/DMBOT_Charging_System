#include <Arduino.h>
#include "robot_fsm.h"
#include "robot_ble.h"
#include "robot_gpio.h"
#include "robot_rs485.h"
#include "config.h"
#include "mbed.h"

mbed::Watchdog &wdt = mbed::Watchdog::get_instance();

void setup() {
  delay(2000);
  Serial.begin(9600);
  Serial.println("\n========================================");
  Serial.println("DMBOT Robot - Charging System (Improved)");
  Serial.println("========================================");

  gpio_init();
  delay(100);

  rs485_init();
  delay(100);

  ble_init();
  delay(100);

  // Watchdog 타이머 시작
  wdt.start(WATCHDOG_TIMEOUT_MS);
  LOG_INFO("Watchdog started (%ldms)", (unsigned long)WATCHDOG_TIMEOUT_MS);
  LOG_INFO("Setup complete - entering main loop");
}

void loop() {
  unsigned long loop_start = millis();
  
  // (IMPROVED) Watchdog 리셋 - 시스템이 살아있다는 신호
  wdt.kick();

  // (IMPROVED) 비블로킹 타입의 함수들만 호출
  rs485_run();        // RS485 수신 처리
  ble_run();          // BLE FSM 실행
  rs485_report();     // 주기적 BLE + 상태 보고

  unsigned long loop_time = millis() - loop_start;
  
  /* ===== ORIGINAL CODE (기존 코드 - 블로킹 위험) =====
  delay(10);          // 너무 길게 하지 말 것 (5초 안에는 최소 한 번 loop 돌아야 함)
  ===== END ORIGINAL CODE ===== */
  
  // (IMPROVED) 루프 시간 모니터링
  static unsigned long last_warn_time = 0;
  if (loop_time > LOOP_MAX_TIME_MS) {
    if (millis() - last_warn_time > 5000) {  // 5초마다 한 번만 출력
      LOG_WARN("Loop took %ldms - Risk of watchdog timeout!", loop_time);
      last_warn_time = millis();
    }
  }
  
  // (IMPROVED) 적절한 지연 (Watchdog 여유 고려)
  delay(10);
}
