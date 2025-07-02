#include "robot_fsm.h"

RobotState robotState = IDLE;

void update_state() {
  // 현재는 상태 전이를 외부에서 직접 처리하므로,
  // 필요 시 여기에 타이머 기반 상태 리셋/복구 로직 등 추가 가능
}