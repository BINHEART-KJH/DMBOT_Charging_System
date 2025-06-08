#include <Arduino.h>
#include <ModbusRtu.h>
#include "modbus_slave.h"

#define SLAVE_ID       1
#define BAUDRATE       9600
#define DE_PIN         2
#define REG_BLE_CMD    0  // Holding register 0번 주소

Modbus slave(SLAVE_ID, Serial1, DE_PIN);
uint16_t holdingRegs[1] = {0};
static bool bleCmdState = false;

void setupModbus() {
  Serial1.begin(BAUDRATE);
  slave.start();  // 반드시 호출
  Serial.println("[MODBUS] 슬레이브 시작됨");
}

void updateModbus() {
  slave.poll(holdingRegs, 1);  // 슬레이브 poll

  if (holdingRegs[REG_BLE_CMD] == 1 && !bleCmdState) {
    bleCmdState = true;
    holdingRegs[REG_BLE_CMD] = 0;
    Serial.println("[MODBUS] BLE ON 명령 수신");
  } else if (holdingRegs[REG_BLE_CMD] == 2 && bleCmdState) {
    bleCmdState = false;
    holdingRegs[REG_BLE_CMD] = 0;
    Serial.println("[MODBUS] BLE OFF 명령 수신");
  }
}

bool modbusGetBLECmd() {
  return bleCmdState;
}