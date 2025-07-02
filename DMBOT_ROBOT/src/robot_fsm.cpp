// #include "robot_fsm.h"
// #include "robot_ble.h"
// #include "robot_gpio.h"
// #include "robot_rs485.h"

// enum RobotState {
//   FSM_IDLE,
//   FSM_CONNECTING,
//   FSM_CONNECTED
// };

// static RobotState currentState = FSM_IDLE;

// void robotFSM_init() {
//   robotGPIO_init();
//   robotRS485_init();
// }

// void robotFSM_update() {
//   robotRS485_update(); // RS485 명령 수신 처리

//   switch (currentState) {
//     case FSM_IDLE:
//       if (robotBLE_isConnected()) {
//         Serial.println("[FSM] BLE 연결됨 → CONNECTING 상태 전환");
//         currentState = FSM_CONNECTING;
//       }
//       break;

//     case FSM_CONNECTING:
//       if (robotBLE_isAuthenticated()) {
//         Serial.println("[FSM] 인증 완료 → CONNECTED 상태 전환");
//         robotRS485_sendBLEStatus(true); // BLE 연결됨 전송
//         currentState = FSM_CONNECTED;
//       } else if (!robotBLE_isConnected()) {
//         Serial.println("[FSM] BLE 연결 끊김 → IDLE 상태 복귀");
//         robotRS485_sendBLEStatus(false);
//         currentState = FSM_IDLE;
//       }
//       break;

//     case FSM_CONNECTED:
//       if (!robotBLE_isConnected()) {
//         Serial.println("[FSM] BLE 연결 끊김 → IDLE 상태 복귀");
//         robotRS485_sendBLEStatus(false);
//         currentState = FSM_IDLE;
//       }
//       break;
//   }
// }