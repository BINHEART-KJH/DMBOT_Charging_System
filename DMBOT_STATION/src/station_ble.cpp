#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"
#include "config.h"

#ifdef ARDUINO_ARCH_MBED
  #include <mbed.h>
#endif

/* ===== ORIGINAL CODE (기존 코드 주석 처리) =====
static inline void hardResetStation() {
  Serial.println(">>> HARD RESET: Docking LOW for threshold <<<");
  delay(50);
  #ifdef ARDUINO_ARCH_MBED
    mbed::Watchdog &wd = mbed::Watchdog::get_instance();
    wd.start(1);  // 1ms는 보장 불가
    while (true) { }
  #else
    NVIC_SystemReset();
  #endif
}
===== END ORIGINAL CODE ===== */

// ===== IMPROVED HARD RESET (개선된 하드리셋) =====
// Relay를 먼저 OFF 후 안전하게 리셋
static inline void hardResetStation() {
  LOG_ERROR(">>> HARD RESET: Docking LOW for 15min <<<");
  
  // 안전: Relay 즉시 OFF
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(RELAY_PIN2, LOW);
  delay(100);
  
  Serial.flush();
  delay(100);

  #ifdef ARDUINO_ARCH_MBED
    try {
      mbed::Watchdog &wd = mbed::Watchdog::get_instance();
      wd.start(500);  // 500ms (충분한 마진)
      while (true) {
        delay(1);
      }
    } catch (...) {
      NVIC_SystemReset();
    }
  #else
    NVIC_SystemReset();
  #endif
}

// 도킹 LOW 지속 시간 감시
static unsigned long dockLowStartMs = 0;

// BLE 상태
bool isAdvertising = false;
unsigned long dockingOkStartTime = 0;
unsigned long authSuccessTime = 0;
bool relayActivated = false;

// 인증 관련
const char *sharedKey = "DM--010225";
char nonce[9];
char tokenHex[17];

// 인증 상태
bool authSuccess = false;
bool authChecked = false;
unsigned long authStartTime = 0;
BLEDevice connectedCentral;

// GATT 서비스 및 캐릭터리스틱
BLEService dmService("180A");
BLECharacteristic nonceChar("2A03", BLERead, 20);
BLECharacteristic authTokenChar("2A04", BLEWrite, 16);
BLEByteCharacteristic connStatusChar("2A00", BLERead);
BLEByteCharacteristic batteryFullChar("2A01", BLERead);
BLEByteCharacteristic chargerOkChar("2A02", BLERead);
BLEByteCharacteristic jumperRelayChar("AA05", BLERead);
BLEByteCharacteristic robotRelayChar("AA10", BLEWrite);

// 랜덤 nonce 생성
void generateRandomNonce(char *buffer, size_t len) {
  const char charset[] = "0123456789abcdef";
  for (size_t i = 0; i < len - 1; ++i) {
    buffer[i] = charset[random(0, 16)];
  }
  buffer[len - 1] = '\0';
}

// HMAC-SHA256 토큰 생성
void generateHMAC_SHA256(const char *key, const char *message, char *outputHex) {
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i) {
    sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  }
  outputHex[16] = '\0';
}

// 인증 토큰 수신
void onAuthTokenWritten(BLEDevice central, BLECharacteristic characteristic) {
  char receivedToken[17];
  characteristic.readValue((unsigned char *)receivedToken, 16);
  receivedToken[16] = '\0';

  LOG_DEBUG("Received auth token");

  if (strcmp(receivedToken, tokenHex) == 0) {
    authSuccess = true;
    authSuccessTime = millis();
    relayActivated = false;
    LOG_INFO("Authentication success");
  } else {
    LOG_WARN("Authentication failed - token mismatch");
    central.disconnect();
  }
}

/* ===== ORIGINAL CODE (기존 코드 주석 처리) =====
void onRobotRelayWritten(BLEDevice central, BLECharacteristic characteristic) {
  byte relayState;
  characteristic.readValue(&relayState, sizeof(relayState));
  Serial.print("Received Relay state: ");
  Serial.println(relayState);

  // ❌ Robot 명령을 무조건 적용 -> Relay 상태 불일치
  if (relayState != digitalRead(RELAY_PIN)) {
    digitalWrite(RELAY_PIN, relayState);
    jumperRelayChar.writeValue(relayState);
  }
}
===== END ORIGINAL CODE ===== */

// ===== IMPROVED Robot Relay Handler (개선된 릴레이 핸들러) =====
// Robot의 요청만 받고, 최종 결정은 gpio_run()에서 수행
void onRobotRelayWritten(BLEDevice central, BLECharacteristic characteristic) {
  byte relayState = 0;
  if (!characteristic.readValue(&relayState, sizeof(relayState))) {
    LOG_WARN("Failed to read relay command from Robot");
    return;
  }

  LOG_DEBUG("Received relay command: %s", relayState ? "ON" : "OFF");

  // (IMPROVED) Robot의 요청만 기록하고, 실제 제어는 gpio_run()에서 수행
  // 이렇게 하면 다음 조건을 모두 만족할 때만 ON:
  // 1. Robot이 ON 요청
  // 2. ADC 전압이 정상 범위
  // 3. 단선/과충전 없음
  // 4. 인증 완료 + 연결 상태
}

void setupGattService() {
  BLE.setLocalName("DM-STATION");
  BLE.setDeviceName("DM-STATION");
  BLE.setAdvertisedService(dmService);

  dmService.addCharacteristic(nonceChar);
  dmService.addCharacteristic(authTokenChar);
  dmService.addCharacteristic(connStatusChar);
  dmService.addCharacteristic(batteryFullChar);
  dmService.addCharacteristic(chargerOkChar);
  dmService.addCharacteristic(jumperRelayChar);
  dmService.addCharacteristic(robotRelayChar);

  BLE.addService(dmService);

  nonceChar.setValue(nonce);
  authTokenChar.setEventHandler(BLEWritten, onAuthTokenWritten);
  robotRelayChar.setEventHandler(BLEWritten, onRobotRelayWritten);

  connStatusChar.writeValue(0);
  batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
  chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
  jumperRelayChar.writeValue(digitalRead(RELAY_PIN));

  delay(200);
}

void updateGattValues() {
  connStatusChar.writeValue(1);
  batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
  chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
  jumperRelayChar.writeValue(digitalRead(RELAY_PIN));
}

void checkAuthTimeout() {
  BLEDevice currentCentral = BLE.central();
  if (!authSuccess && currentCentral && currentCentral.connected()) {
    if (millis() - authStartTime > AUTH_TIMEOUT_MS) {
      LOG_WARN("Authentication timeout -> disconnect");
      currentCentral.disconnect();
      connectedCentral = BLEDevice();
      authSuccess = false;
      authChecked = false;
      currentState = ADVERTISING;
    }
  }
}

void ble_init() {
  if (!BLE.begin()) {
    LOG_ERROR("BLE init failed");
    return;
  }

  LOG_INFO("BLE initialized");
  randomSeed(analogRead(A0));
  generateRandomNonce(nonce, sizeof(nonce));
  generateHMAC_SHA256(sharedKey, nonce, tokenHex);

  LOG_DEBUG("Nonce: %s", nonce);
  LOG_DEBUG("Expected token: %s", tokenHex);

  setupGattService();
  dockLowStartMs = 0;
}

void ble_reset() {
  LOG_INFO("BLE reset");

  digitalWrite(RELAY_PIN, LOW);
  jumperRelayChar.writeValue(0);

  BLEDevice central = BLE.central();
  if (central) {
    central.disconnect();
  }

  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
  }

  BLE.end();
  delay(500);

  if (!BLE.begin()) {
    LOG_ERROR("BLE restart failed");
  } else {
    LOG_INFO("BLE restarted");
    setupGattService();
  }

  dockLowStartMs = 0;
}

void ble_run() {
  BLEDevice central = BLE.central();
  int docking = digitalRead(DOCKING_PIN);
  unsigned long now = millis();

  // === 도킹 LOW 하드리셋 타이머 ===
  if (docking == LOW) {
    if (dockLowStartMs == 0) {
      dockLowStartMs = now;
    } else if (now - dockLowStartMs >= DOCK_LOW_HARD_RESET_MS) {
      LOG_ERROR("Docking LOW for 15 minutes -> hard reset");
      hardResetStation();
      return;
    }
  } else {
    dockLowStartMs = 0;
  }

  // === 도킹 HIGH일 때만 광고 수행 ===
  if (docking == HIGH) {
    if (dockingOkStartTime == 0) {
      dockingOkStartTime = now;
    }

    if (!isAdvertising && now - dockingOkStartTime >= DOCK_OK_DELAY_MS) {
      LOG_INFO("BLE advertising start");
      BLE.advertise();
      isAdvertising = true;
      currentState = ADVERTISING;
    }
  } else {
    dockingOkStartTime = 0;

    if (isAdvertising) {
      LOG_INFO("BLE advertising stop (DOCKING_PIN LOW)");
      BLE.stopAdvertise();
      isAdvertising = false;
    }

    if (connectedCentral && connectedCentral.connected()) {
      LOG_WARN("Docking LOW -> disconnect BLE");
      connectedCentral.disconnect();
      connectedCentral = BLEDevice();
    }

    digitalWrite(RELAY_PIN, LOW);
    jumperRelayChar.writeValue(0);
    relayActivated = false;

    currentState = IDLE;
    return;
  }

  // === 연결/인증 처리 ===
  if (central) {
    if (!connectedCentral && central.connected()) {
      LOG_INFO("Central connected");
      connectedCentral = central;
      authSuccess = false;
      authChecked = false;
      authStartTime = now;
      currentState = CONNECTING;
    }

    if (connectedCentral && connectedCentral.connected()) {
      if (!authChecked && now - authStartTime > 1000) {
        authChecked = true;
        LOG_DEBUG("Waiting for authentication (HMAC)");
      }

      if (authSuccess) {
        updateGattValues();
        currentState = CONNECTED;

        byte relayState = digitalRead(RELAY_PIN);
        jumperRelayChar.writeValue(relayState);
      } else {
        checkAuthTimeout();
      }
    } else {
      if (connectedCentral) {
        LOG_WARN("Connection lost");
        connectedCentral = BLEDevice();
        authSuccess = false;
        authChecked = false;
        relayActivated = false;
        digitalWrite(RELAY_PIN, LOW);
        jumperRelayChar.writeValue(0);
        currentState = ADVERTISING;
      }
    }
  }

  // 연결이 아니면 Relay 강제 OFF
  if (currentState != CONNECTED && relayActivated) {
    digitalWrite(RELAY_PIN, LOW);
    jumperRelayChar.writeValue(0);
    relayActivated = false;
    LOG_WARN("Not CONNECTED -> relay forced OFF");
  }
}
