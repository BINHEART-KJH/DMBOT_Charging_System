/*
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"

#ifdef ARDUINO_ARCH_MBED
  #include <mbed.h>
#endif

// =================== 하드리셋 유틸 ===================
static inline void hardResetStation() {
  Serial.println(">>> HARD RESET: Docking LOW for threshold <<<");
  delay(50);
#ifdef ARDUINO_ARCH_MBED
  // mbed Watchdog을 1ms로 시작해 하드 리셋 유도
  mbed::Watchdog &wd = mbed::Watchdog::get_instance();
  wd.start(1);
  while (true) {  }
#else
  // 다른 코어라면 NVIC 리셋로 폴백
  NVIC_SystemReset();
#endif
}

// 도킹 LOW 지속 시간 감시 (15분)
static unsigned long dockLowStartMs = 0;
const unsigned long DOCK_LOW_RESET_MS = 15UL * 60UL * 1000UL;

// BLE 상태
bool isAdvertising = false;
unsigned long dockingOkStartTime = 0;
unsigned long authSuccessTime = 0;
bool relayActivated = false;

// 인증 관련
const char *sharedKey = "DM--010225";
char nonce[9];     // 8자리 + null
char tokenHex[17]; // 8바이트 = 16 hex chars + null

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
BLEByteCharacteristic jumperRelayChar("AA05", BLERead);  // Station → Robot: 릴레이 상태 공유
BLEByteCharacteristic robotRelayChar("AA10", BLEWrite);  // Robot → Station: 로봇 릴레이 명령 수신
// BLEByteCharacteristic dockingStatusChar("AA06", BLERead);  // 필요시 사용

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

  Serial.print("Received Auth Token: ");
  Serial.println(receivedToken);

  if (strcmp(receivedToken, tokenHex) == 0) {
    authSuccess = true;
    authSuccessTime = millis();  // 인증 시간 기록
    relayActivated = false;      // 릴레이 상태 초기화
    Serial.println("인증 성공!");
  } else {
    Serial.println("인증 실패.");
    central.disconnect();  // 인증 실패 시 연결 종료
  }
}

// Robot → Station 릴레이 상태 동기화
void onRobotRelayWritten(BLEDevice central, BLECharacteristic characteristic) {
  byte relayState;
  characteristic.readValue(&relayState, sizeof(relayState));  // Robot에서 전송된 Relay 상태 읽기
  Serial.print("Received Relay state: ");
  Serial.println(relayState);

  // 변할 때만 반영
  if (relayState != digitalRead(RELAY_PIN)) {
    digitalWrite(RELAY_PIN, relayState);
    jumperRelayChar.writeValue(relayState);  // Station → Robot으로 다시 공유
  }
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
  // dmService.addCharacteristic(dockingStatusChar);

  BLE.addService(dmService);

  nonceChar.setValue(nonce);
  authTokenChar.setEventHandler(BLEWritten, onAuthTokenWritten);
  robotRelayChar.setEventHandler(BLEWritten, onRobotRelayWritten);

  connStatusChar.writeValue(0);
  batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
  chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
  jumperRelayChar.writeValue(digitalRead(RELAY_PIN));
  // dockingStatusChar.writeValue(digitalRead(DOCKING_PIN));

  delay(200);
}

void updateGattValues() {
  connStatusChar.writeValue(1);
  batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
  chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
  jumperRelayChar.writeValue(digitalRead(RELAY_PIN));
  // dockingStatusChar.writeValue(digitalRead(DOCKING_PIN));
}

void checkAuthTimeout() {
  BLEDevice currentCentral = BLE.central();
  if (!authSuccess && currentCentral && currentCentral.connected()) {
    if (millis() - authStartTime > 5000) {
      Serial.println("인증 타임아웃. 연결 해제합니다.");
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
    Serial.println("BLE 초기화 실패!");
    return;
  }
  Serial.println("BLE 초기화 완료");
  randomSeed(analogRead(A0));
  generateRandomNonce(nonce, sizeof(nonce));
  generateHMAC_SHA256(sharedKey, nonce, tokenHex);
  Serial.print("Generated Nonce: ");
  Serial.println(nonce);
  Serial.print("Expected Auth Token: ");
  Serial.println(tokenHex);
  setupGattService();

  // 도킹 LOW 타이머 초기화
  dockLowStartMs = 0;
}

// BLE 초기화 및 설정
void ble_reset() {
  Serial.println("BLE 리셋: 연결 해제 + 초기화");

  // Relay OFF
  digitalWrite(RELAY_PIN, LOW);
  jumperRelayChar.writeValue(0);

  // 기존 연결 종료
  BLEDevice central = BLE.central();
  if (central) {
    central.disconnect();
    Serial.println("BLE Central 연결 끊김");
  }

  // 광고 중지
  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
  }

  BLE.end();
  delay(500);

  // BLE 재시작
  Serial.println("BLE 재시작 중...");
  if (!BLE.begin()) {
    Serial.println("BLE 재시작 실패!");
  } else {
    Serial.println("BLE 재시작 성공");
    setupGattService();
  }

  // 도킹 LOW 타이머도 초기화
  dockLowStartMs = 0;
}

// BLE 연결 및 상태 처리 + 도킹 LOW 하드리셋 감시
void ble_run() {
  BLEDevice central = BLE.central();
  int docking = digitalRead(DOCKING_PIN);
  unsigned long now = millis();

  // === 도킹 LOW 하드리셋 타이머 ===
  if (docking == LOW) {
    if (dockLowStartMs == 0) {
      dockLowStartMs = now;
    } else if (now - dockLowStartMs >= DOCK_LOW_RESET_MS) {
      Serial.println("Docking LOW 15분 지속 → 하드리셋 실행");
      hardResetStation();
      return; // (실제로는 돌아오지 않음)
    }
  } else {
    // 도킹 HIGH가 되면 타이머 리셋
    dockLowStartMs = 0;
  }

  // === 도킹 HIGH일 때만 광고 수행 ===
  if (docking == HIGH) {
    if (dockingOkStartTime == 0) dockingOkStartTime = now;

    // 3초 유지 후 광고 시작
    if (!isAdvertising && now - dockingOkStartTime >= 3000) {
      Serial.println("BLE Advertising 시작");
      BLE.advertise();
      isAdvertising = true;
      currentState = ADVERTISING;
    }
  } else {
    dockingOkStartTime = 0;

    // 도킹 LOW이면 광고 중지
    if (isAdvertising) {
      Serial.println("BLE Advertising 중지 (DOCKING_PIN LOW)");
      BLE.stopAdvertise();
      isAdvertising = false;
    }

    // 연결되어 있으면 강제 해제
    if (connectedCentral && connectedCentral.connected()) {
      Serial.println("Docking LOW 상태 - BLE 연결 강제 해제");
      connectedCentral.disconnect();
      connectedCentral = BLEDevice();
    }

    // Relay OFF
    digitalWrite(RELAY_PIN, LOW);
    jumperRelayChar.writeValue(0);
    relayActivated = false;

    currentState = IDLE;
    return;
  }

  // === 연결/인증 처리 ===
  if (central) {
    if (!connectedCentral && central.connected()) {
      Serial.println("Central 연결 감지");
      connectedCentral = central;
      authSuccess = false;
      authChecked = false;
      authStartTime = now;
      currentState = CONNECTING;
    }

    if (connectedCentral && connectedCentral.connected()) {
      if (!authChecked && now - authStartTime > 1000) {
        authChecked = true;
        Serial.println("인증 대기 중 (HMAC 방식)");
      }

      if (authSuccess) {
        updateGattValues();
        currentState = CONNECTED;

        // Station → Robot 현재 릴레이 상태 반복 공유(필요 시)
        byte relayState = digitalRead(RELAY_PIN);
        jumperRelayChar.writeValue(relayState);
      } else {
        checkAuthTimeout();
      }
    } else {
      if (connectedCentral) {
        Serial.println("연결 끊김 감지");
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

  // 연결 아님이면 Relay 강제 OFF
  if (currentState != CONNECTED && relayActivated) {
    digitalWrite(RELAY_PIN, LOW);
    jumperRelayChar.writeValue(0);
    relayActivated = false;
    Serial.println("CONNECTED 아님 - Relay 강제 OFF");
  }
}
*/
/*
#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"

#ifdef ARDUINO_ARCH_MBED
  #include <mbed.h>
#endif

// =================== 하드리셋 유틸 ===================
static inline void hardResetStation() {
  Serial.println(">>> HARD RESET: Docking LOW for threshold <<<");
  delay(50);
#ifdef ARDUINO_ARCH_MBED
  mbed::Watchdog &wd = mbed::Watchdog::get_instance();
  wd.start(1);
  while (true) { }
#else
  NVIC_SystemReset();
#endif
}

// 도킹 LOW 지속 시간 감시 (15분)
static unsigned long dockLowStartMs = 0;
const unsigned long DOCK_LOW_RESET_MS = 15UL * 60UL * 1000UL;

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

// GATT
BLEService dmService("180A");
BLECharacteristic nonceChar("2A03", BLERead, 20);
BLECharacteristic authTokenChar("2A04", BLEWrite, 16);
BLEByteCharacteristic connStatusChar("2A00", BLERead);
BLEByteCharacteristic batteryFullChar("2A01", BLERead);
BLEByteCharacteristic chargerOkChar("2A02", BLERead);
BLEByteCharacteristic jumperRelayChar("AA05", BLERead);  // Station → Robot
BLEByteCharacteristic robotRelayChar("AA10", BLEWrite);  // Robot → Station

void generateRandomNonce(char *buffer, size_t len) {
  const char charset[] = "0123456789abcdef";
  for (size_t i = 0; i < len - 1; ++i) buffer[i] = charset[random(0, 16)];
  buffer[len - 1] = '\0';
}

void generateHMAC_SHA256(const char *key, const char *message, char *outputHex) {
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i) sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  outputHex[16] = '\0';
}

void onAuthTokenWritten(BLEDevice central, BLECharacteristic characteristic) {
  char receivedToken[17];
  characteristic.readValue((unsigned char *)receivedToken, 16);
  receivedToken[16] = '\0';

  Serial.print("Received Auth Token: ");
  Serial.println(receivedToken);

  if (strcmp(receivedToken, tokenHex) == 0) {
    authSuccess = true;
    authSuccessTime = millis();
    relayActivated = false;
    Serial.println("인증 성공!");
  } else {
    Serial.println("인증 실패.");
    central.disconnect();
  }
}

// Robot → Station : RELAY_PIN(D4) ON/OFF
void onRobotRelayWritten(BLEDevice central, BLECharacteristic characteristic) {
  byte relayState = 0;
  characteristic.readValue(&relayState, sizeof(relayState));

  Serial.print("Received Relay state (BLE): ");
  Serial.println(relayState);

  bool on = (relayState != 0);

  // 릴레이는 D7만
  station_setRelay(on, "BLE CMD -> Relay(D7)");

  // Station → Robot으로 현재 상태 공유
  jumperRelayChar.writeValue((byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0));

  // 연결 끊김 강제 OFF 로직에 쓸 플래그
  relayActivated = on;
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
  jumperRelayChar.writeValue((byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0));

  delay(200);
}

void updateGattValues() {
  connStatusChar.writeValue(1);
  batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
  chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
  jumperRelayChar.writeValue((byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0));
}

void checkAuthTimeout() {
  BLEDevice currentCentral = BLE.central();
  if (!authSuccess && currentCentral && currentCentral.connected()) {
    if (millis() - authStartTime > 5000) {
      Serial.println("인증 타임아웃. 연결 해제합니다.");
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
    Serial.println("BLE 초기화 실패!");
    return;
  }
  Serial.println("BLE 초기화 완료");

  randomSeed(analogRead(A0));
  generateRandomNonce(nonce, sizeof(nonce));
  generateHMAC_SHA256(sharedKey, nonce, tokenHex);

  Serial.print("Generated Nonce: ");
  Serial.println(nonce);
  Serial.print("Expected Auth Token: ");
  Serial.println(tokenHex);

  setupGattService();

  dockLowStartMs = 0;
}

void ble_reset() {
  Serial.println("BLE 리셋: 연결 해제 + 초기화");

  // 릴레이 OFF (연결 리셋 시 안전)
  station_setRelay(false, "BLE reset -> Relay OFF");
  jumperRelayChar.writeValue(0);
  relayActivated = false;

  BLEDevice central = BLE.central();
  if (central) {
    central.disconnect();
    Serial.println("BLE Central 연결 끊김");
  }

  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
  }

  BLE.end();
  delay(500);

  Serial.println("BLE 재시작 중...");
  if (!BLE.begin()) {
    Serial.println("BLE 재시작 실패!");
  } else {
    Serial.println("BLE 재시작 성공");
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
    if (dockLowStartMs == 0) dockLowStartMs = now;
    else if (now - dockLowStartMs >= DOCK_LOW_RESET_MS) {
      Serial.println("Docking LOW 15분 지속 → 하드리셋 실행");
      hardResetStation();
      return;
    }
  } else {
    dockLowStartMs = 0;
  }

  // === 도킹 HIGH일 때만 광고 ===
  if (docking == HIGH) {
    if (dockingOkStartTime == 0) dockingOkStartTime = now;

    if (!isAdvertising && now - dockingOkStartTime >= 3000) {
      Serial.println("BLE Advertising 시작");
      BLE.advertise();
      isAdvertising = true;
      currentState = ADVERTISING;
    }
  } else {
    dockingOkStartTime = 0;

    if (isAdvertising) {
      Serial.println("BLE Advertising 중지 (DOCKING_PIN LOW)");
      BLE.stopAdvertise();
      isAdvertising = false;
    }

    if (connectedCentral && connectedCentral.connected()) {
      Serial.println("Docking LOW 상태 - BLE 연결 강제 해제");
      connectedCentral.disconnect();
      connectedCentral = BLEDevice();
    }

    // 도킹 풀리면 안전하게 OFF
    station_setRelay(false, "Docking LOW -> Relay OFF");
    jumperRelayChar.writeValue(0);
    relayActivated = false;

    currentState = IDLE;
    return;
  }

  // === 연결/인증 처리 ===
  if (central) {
    if (!connectedCentral && central.connected()) {
      Serial.println("Central 연결 감지");
      connectedCentral = central;
      authSuccess = false;
      authChecked = false;
      authStartTime = now;
      currentState = CONNECTING;
    }

    if (connectedCentral && connectedCentral.connected()) {
      if (!authChecked && now - authStartTime > 1000) {
        authChecked = true;
        Serial.println("인증 대기 중 (HMAC 방식)");
      }

      if (authSuccess) {
        updateGattValues();
        currentState = CONNECTED;

        byte rs = (byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0);
        jumperRelayChar.writeValue(rs);
      } else {
        checkAuthTimeout();
      }
    } else {
      if (connectedCentral) {
        Serial.println("연결 끊김 감지");
        connectedCentral = BLEDevice();
        authSuccess = false;
        authChecked = false;

        // 연결 끊기면 OFF
        station_setRelay(false, "BLE disconnected -> Relay OFF");
        jumperRelayChar.writeValue(0);
        relayActivated = false;

        currentState = ADVERTISING;
      }
    }
  }

  // 연결 아님인데 릴레이가 켜져 있으면 OFF
  if (currentState != CONNECTED && relayActivated) {
    station_setRelay(false, "Not CONNECTED -> Relay OFF");
    jumperRelayChar.writeValue(0);
    relayActivated = false;
    Serial.println("CONNECTED 아님 - Relay 강제 OFF");
  }
}
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"

#ifdef ARDUINO_ARCH_MBED
  #include <mbed.h>
#endif

// =================== 하드리셋 유틸 ===================
static inline void hardResetStation() {
  Serial.println(">>> HARD RESET: Docking LOW for threshold <<<");
  delay(50);
#ifdef ARDUINO_ARCH_MBED
  mbed::Watchdog &wd = mbed::Watchdog::get_instance();
  wd.start(1);
  while (true) { }
#else
  NVIC_SystemReset();
#endif
}

// 도킹 LOW 지속 시간 감시 (15분)
static unsigned long dockLowStartMs = 0;
const unsigned long DOCK_LOW_RESET_MS = 15UL * 60UL * 1000UL;

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

// GATT
BLEService dmService("180A");
BLECharacteristic nonceChar("2A03", BLERead, 20);
BLECharacteristic authTokenChar("2A04", BLEWrite, 16);
BLEByteCharacteristic connStatusChar("2A00", BLERead);
BLEByteCharacteristic batteryFullChar("2A01", BLERead);
BLEByteCharacteristic chargerOkChar("2A02", BLERead);
BLEByteCharacteristic jumperRelayChar("AA05", BLERead);  // Station → Robot
BLEByteCharacteristic robotRelayChar("AA10", BLEWrite);  // Robot → Station

static inline bool isAuthedConnected() {
  return (connectedCentral && connectedCentral.connected() && authSuccess);
}

static inline void forceRelayOff(const char* reason) {
  station_setRelay(false, reason);
  jumperRelayChar.writeValue(0);
  relayActivated = false;
}

void generateRandomNonce(char *buffer, size_t len) {
  const char charset[] = "0123456789abcdef";
  for (size_t i = 0; i < len - 1; ++i) buffer[i] = charset[random(0, 16)];
  buffer[len - 1] = '\0';
}

void generateHMAC_SHA256(const char *key, const char *message, char *outputHex) {
  uint8_t hmacResult[32];
  HMAC hmac;
  hmac.init((const uint8_t *)key, strlen(key));
  hmac.update((const uint8_t *)message, strlen(message));
  hmac.finalize(hmacResult, sizeof(hmacResult));
  for (int i = 0; i < 8; ++i) sprintf(&outputHex[i * 2], "%02x", hmacResult[i]);
  outputHex[16] = '\0';
}

void onAuthTokenWritten(BLEDevice central, BLECharacteristic characteristic) {
  char receivedToken[17];
  characteristic.readValue((unsigned char *)receivedToken, 16);
  receivedToken[16] = '\0';

  Serial.print("Received Auth Token: ");
  Serial.println(receivedToken);

  if (strcmp(receivedToken, tokenHex) == 0) {
    authSuccess = true;
    authSuccessTime = millis();
    Serial.println("인증 성공!");
  } else {
    Serial.println("인증 실패.");
    central.disconnect();
  }
}

// Robot → Station : Relay ON/OFF
void onRobotRelayWritten(BLEDevice central, BLECharacteristic characteristic) {
  // 인증 전/연결 이상 상태에서는 명령 무시 (안전)
  if (!isAuthedConnected()) {
    Serial.println("Robot relay write ignored (not authed/connected).");
    return;
  }

  uint8_t relayState = 0xFF;

  // 1바이트만 읽고, 읽기 성공 여부 확인
  int n = characteristic.readValue(&relayState, 1);
  if (n != 1) {
    Serial.print("Robot relay readValue failed, n=");
    Serial.println(n);
    return; // 읽기 실패면 무시
  }

  Serial.print("Received Relay state (BLE): ");
  Serial.println(relayState);

  // 0/1
  if (relayState != 0 && relayState != 1) {
    Serial.print("Ignored invalid relayState: ");
    Serial.println(relayState);
    return;
  }

  bool on = (relayState == 1);

  station_setRelay(on, "BLE CMD -> Relay(D4)");
  jumperRelayChar.writeValue((byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0));
  relayActivated = on;
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
  jumperRelayChar.writeValue((byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0));

  delay(200);
}

void updateGattValues() {
  connStatusChar.writeValue(1);
  batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
  chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
  jumperRelayChar.writeValue((byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0));
}

void checkAuthTimeout(unsigned long now) {
  if (!authSuccess && connectedCentral && connectedCentral.connected()) {
    if (now - authStartTime > 5000) {
      Serial.println("인증 타임아웃. 연결 해제합니다.");
      connectedCentral.disconnect();
      connectedCentral = BLEDevice();
      authSuccess = false;
      authChecked = false;
      currentState = ADVERTISING;

      forceRelayOff("Auth timeout -> Relay OFF");
    }
  }
}

void ble_init() {
  if (!BLE.begin()) {
    Serial.println("BLE 초기화 실패!");
    return;
  }
  Serial.println("BLE 초기화 완료");

  randomSeed(analogRead(A0));
  generateRandomNonce(nonce, sizeof(nonce));
  generateHMAC_SHA256(sharedKey, nonce, tokenHex);

  Serial.print("Generated Nonce: ");
  Serial.println(nonce);
  Serial.print("Expected Auth Token: ");
  Serial.println(tokenHex);

  setupGattService();

  dockLowStartMs = 0;
  dockingOkStartTime = 0;
  isAdvertising = false;

  authSuccess = false;
  authChecked = false;
  authStartTime = 0;
  relayActivated = false;
  connectedCentral = BLEDevice();
}

void ble_reset() {
  Serial.println("BLE 리셋: 연결 해제 + 초기화");

  forceRelayOff("BLE reset -> Relay OFF");

  if (connectedCentral && connectedCentral.connected()) {
    connectedCentral.disconnect();
    Serial.println("BLE Central 연결 끊김");
  }
  connectedCentral = BLEDevice();

  if (isAdvertising) {
    BLE.stopAdvertise();
    isAdvertising = false;
  }

  BLE.end();
  delay(500);

  Serial.println("BLE 재시작 중...");
  if (!BLE.begin()) {
    Serial.println("BLE 재시작 실패!");
  } else {
    Serial.println("BLE 재시작 성공");
    setupGattService();
  }

  dockLowStartMs = 0;
  dockingOkStartTime = 0;

  authSuccess = false;
  authChecked = false;
  authStartTime = 0;
}

void ble_run() {
  // ArduinoBLE 이벤트 처리 (콜백 동작에 필요)
  BLE.poll();

  unsigned long now = millis();
  int docking = digitalRead(DOCKING_PIN);

  // === 도킹 LOW 하드리셋 타이머 ===
  if (docking == LOW) {
    if (dockLowStartMs == 0) dockLowStartMs = now;
    else if (now - dockLowStartMs >= DOCK_LOW_RESET_MS) {
      Serial.println("Docking LOW 15분 지속 → 하드리셋 실행");
      hardResetStation();
      return;
    }
  } else {
    dockLowStartMs = 0;
  }

  // === 도킹 HIGH일 때만 광고 ===
  if (docking == HIGH) {
    if (dockingOkStartTime == 0) dockingOkStartTime = now;

    if (!isAdvertising && now - dockingOkStartTime >= 3000) {
      Serial.println("BLE Advertising 시작");
      BLE.advertise();
      isAdvertising = true;
      currentState = ADVERTISING;
    }
  } else {
    // 도킹 LOW면 즉시 안전모드
    dockingOkStartTime = 0;

    if (isAdvertising) {
      Serial.println("BLE Advertising 중지 (DOCKING_PIN LOW)");
      BLE.stopAdvertise();
      isAdvertising = false;
    }

    if (connectedCentral && connectedCentral.connected()) {
      Serial.println("Docking LOW 상태 - BLE 연결 강제 해제");
      connectedCentral.disconnect();
      connectedCentral = BLEDevice();
    }

    authSuccess = false;
    authChecked = false;

    forceRelayOff("Docking LOW -> Relay OFF");

    currentState = IDLE;
    return;
  }

  // === 새 연결 감지: '처음 1번만' 잡아야 한다 ===
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    // 이미 잡힌 연결이 없을 때만 "새 연결"로 처리
    if (!connectedCentral || !connectedCentral.connected()) {
      Serial.println("Central 연결 감지 (NEW)");
      connectedCentral = central;

      authSuccess = false;
      authChecked = false;
      authStartTime = now;

      // 연결되면 광고 플래그 정리(필요시)
      if (isAdvertising) {
        BLE.stopAdvertise();
        isAdvertising = false;
      }

      currentState = CONNECTING;
    }
  }

  // === 연결 유지/인증 처리 ===
  if (connectedCentral) {
    if (connectedCentral.connected()) {
      if (!authChecked && (now - authStartTime > 1000)) {
        authChecked = true;
        Serial.println("인증 대기 중 (HMAC 방식)");
      }

      if (authSuccess) {
        updateGattValues();
        currentState = CONNECTED;

        byte rs = (byte)(digitalRead(RELAY_PIN2) == HIGH ? 1 : 0);
        jumperRelayChar.writeValue(rs);
      } else {
        checkAuthTimeout(now);
      }
    } else {
      Serial.println("연결 끊김 감지");
      connectedCentral = BLEDevice();
      authSuccess = false;
      authChecked = false;

      forceRelayOff("BLE disconnected -> Relay OFF");

      currentState = ADVERTISING;
    }
  }

  // === 핵심: 강제 OFF 판단은 currentState가 아니라 "실제 연결+인증" ===
  if (!isAuthedConnected() && relayActivated) {
    forceRelayOff("Not authed/connected -> Relay OFF");
    Serial.println("AUTH/CONNECTED 아님 - Relay 강제 OFF");
  }
}