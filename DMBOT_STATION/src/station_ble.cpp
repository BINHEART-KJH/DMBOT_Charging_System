/*#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"

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
BLEByteCharacteristic jumperRelayChar("AA05", BLERead);

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
        central.disconnect();
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
    BLE.addService(dmService);
    nonceChar.setValue(nonce);
    authTokenChar.setEventHandler(BLEWritten, onAuthTokenWritten);

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
}

void ble_run() {
    BLEDevice central = BLE.central();
    int docking = digitalRead(DOCKING_PIN);

    if (docking == HIGH) {
        if (dockingOkStartTime == 0) dockingOkStartTime = millis();
        if (!isAdvertising && millis() - dockingOkStartTime >= 3000) {
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

        digitalWrite(RELAY_PIN, LOW);
        jumperRelayChar.writeValue(0);
        relayActivated = false;

        currentState = IDLE;
        return;
    }

    if (central) {
        if (!connectedCentral && central.connected()) {
            Serial.println("Central 연결 감지");
            connectedCentral = central;
            authSuccess = false;
            authChecked = false;
            authStartTime = millis();
            currentState = CONNECTING;
        }

        if (connectedCentral && connectedCentral.connected()) {
            if (!authChecked && millis() - authStartTime > 1000) {
                authChecked = true;
                Serial.println("인증 대기 중 (HMAC 방식)");
            }

            if (authSuccess) {
                updateGattValues();
                currentState = CONNECTED;

                if (!relayActivated && millis() - authSuccessTime >= 10000) {
                    digitalWrite(RELAY_PIN, HIGH);
                    jumperRelayChar.writeValue(1);
                    Serial.println("Charger Jumper Relay ON");
                    relayActivated = true;
                }
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

    if (currentState != CONNECTED && relayActivated) {
        digitalWrite(RELAY_PIN, LOW);
        jumperRelayChar.writeValue(0);
        relayActivated = false;
        Serial.println("CONNECTED 아님 - Relay 강제 OFF");
    }
}

void ble_reset() {
    Serial.println("BLE 리셋: 연결 해제 + 초기화");
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
    delay(100);
    if (!BLE.begin()) {
        Serial.println("BLE 재시작 실패!");
    } else {
        Serial.println("BLE 재시작 성공");
        setupGattService();
    }
}
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"

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
BLEByteCharacteristic jumperRelayChar("AA05", BLERead);  // Station에서 Relay 상태 전달용
BLEByteCharacteristic robotRelayChar("AA10", BLEWrite);  // Robot에 Relay 제어 신호 전달
BLEByteCharacteristic dockingStatusChar("AA06", BLERead);  // Docking 상태 전달용

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

// 인증 토큰이 작성될 때 호출되는 함수
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

// Relay 상태 동기화: Robot → Station
void onRobotRelayWritten(BLEDevice central, BLECharacteristic characteristic) {
    byte relayState;
    characteristic.readValue(&relayState, sizeof(relayState));  // Robot에서 전송된 Relay 상태 읽기
    Serial.print("Received Relay state: ");
    Serial.println(relayState);

    // Relay 상태가 변할 때만 Relay 상태 설정
    if (relayState != digitalRead(RELAY_PIN)) {
        digitalWrite(RELAY_PIN, relayState);  // Station Relay 상태 변경
        jumperRelayChar.writeValue(relayState);  // 상태를 다시 Station → Robot으로 전송
        //Serial.print("Relay 상태 변경: ");
        //Serial.println(relayState);
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
    dmService.addCharacteristic(robotRelayChar);  // Robot Relay 상태 제어 특성 추가
    dmService.addCharacteristic(dockingStatusChar);

    BLE.addService(dmService);
    nonceChar.setValue(nonce);
    authTokenChar.setEventHandler(BLEWritten, onAuthTokenWritten);  // 인증 토큰을 받았을 때 처리하는 이벤트 핸들러
    robotRelayChar.setEventHandler(BLEWritten, onRobotRelayWritten);  // Robot Relay 상태 수신 이벤트 핸들러

    connStatusChar.writeValue(0);
    batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
    chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
    jumperRelayChar.writeValue(digitalRead(RELAY_PIN));  // 초기 상태 전송
    dockingStatusChar.writeValue(digitalRead(DOCKING_PIN));

    delay(200);
}

void updateGattValues() {
    connStatusChar.writeValue(1);
    batteryFullChar.writeValue(digitalRead(BATTERY_FULL_PIN));
    chargerOkChar.writeValue(digitalRead(CHARGER_OK_PIN));
    jumperRelayChar.writeValue(digitalRead(RELAY_PIN));  // Relay 상태 갱신
    dockingStatusChar.writeValue(digitalRead(DOCKING_PIN));  // docking_pin 상태 갱신

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
}


// BLE 초기화 및 설정
void ble_reset() {
    Serial.println("BLE 리셋: 연결 해제 + 초기화");

    // Relay 상태 OFF로 설정
    digitalWrite(RELAY_PIN, LOW);  // Relay OFF
    jumperRelayChar.writeValue(0); // Station Relay 상태를 OFF로 설정

    // 기존 BLE 연결이 되어 있으면 끊기
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

    BLE.end();  // BLE 종료
    delay(500);  // 충분한 시간 대기 후 BLE 종료

    // BLE 재시작
    Serial.println("BLE 재시작 중...");
    if (!BLE.begin()) {
        Serial.println("BLE 재시작 실패!");
    } else {
        Serial.println("BLE 재시작 성공");
        setupGattService();  // GATT 서비스 재설정
    }
}

// BLE 연결 및 상태 처리
void ble_run() {
    BLEDevice central = BLE.central();
    int docking = digitalRead(DOCKING_PIN);

    // DOCKING_PIN이 HIGH일 때 BLE 광고 시작
    if (docking == HIGH) {
        if (dockingOkStartTime == 0) dockingOkStartTime = millis();
        
        // 3초가 지난 후 BLE 광고 시작
        if (!isAdvertising && millis() - dockingOkStartTime >= 3000) {
            Serial.println("BLE Advertising 시작");
            BLE.advertise();
            isAdvertising = true;
            currentState = ADVERTISING;
        }
    } else {
        dockingOkStartTime = 0;
        
        // DOCKING_PIN이 LOW일 때 BLE 광고를 중지
        if (isAdvertising) {
            Serial.println("BLE Advertising 중지 (DOCKING_PIN LOW)");
            BLE.stopAdvertise();
            isAdvertising = false;
        }
        
        // 연결이 되어 있다면 강제로 해제
        if (connectedCentral && connectedCentral.connected()) {
            Serial.println("Docking LOW 상태 - BLE 연결 강제 해제");
            connectedCentral.disconnect();
            connectedCentral = BLEDevice();
        }

        // Relay OFF 상태로 설정
        digitalWrite(RELAY_PIN, LOW);
        jumperRelayChar.writeValue(0);  // 상태가 OFF로 변경되었을 때만 전송
        relayActivated = false;

        currentState = IDLE;
        return;
    }

    // BLE 연결 감지 및 인증 처리
    if (central) {
        if (!connectedCentral && central.connected()) {
            Serial.println("Central 연결 감지");
            connectedCentral = central;
            authSuccess = false;
            authChecked = false;
            authStartTime = millis();
            currentState = CONNECTING;
        }

        if (connectedCentral && connectedCentral.connected()) {
            if (!authChecked && millis() - authStartTime > 1000) {
                authChecked = true;
                Serial.println("인증 대기 중 (HMAC 방식)");
            }

            if (authSuccess) {
                updateGattValues();
                currentState = CONNECTED;

                // GATT 특성 값 처리: Relay 상태를 Robot에 전송
                byte relayState = digitalRead(RELAY_PIN);
                jumperRelayChar.writeValue(relayState);  // Station Relay 상태를 Robot으로 전송
                //Serial.print("Relay 상태 전송: ");
                //Serial.println(relayState);
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

    // 연결이 되지 않은 상태에서 Relay는 강제로 OFF
    if (currentState != CONNECTED && relayActivated) {
        digitalWrite(RELAY_PIN, LOW);
        jumperRelayChar.writeValue(0);
        relayActivated = false;
        Serial.println("CONNECTED 아님 - Relay 강제 OFF");
    }
}