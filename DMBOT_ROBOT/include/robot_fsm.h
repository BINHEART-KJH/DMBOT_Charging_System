#ifndef ROBOT_FSM_H
#define ROBOT_FSM_H

enum RobotState {
  IDLE,
  SCANNING,
  CONNECTING,
  CONNECTED
};

extern RobotState robotState;

void update_state();  // 추후 조건에 따라 상태 전이 추가 예정

#endif