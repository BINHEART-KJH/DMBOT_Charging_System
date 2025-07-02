#include <Arduino.h>
#include <ArduinoBLE.h>
#include "station_gpio.h"
#include "station_fsm.h"
#include "sha256.h"
#include "hmac.h"
#include "black_white_list.h"

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
        authSuccessTime = millis();  // ✅ 인증 시간 기록
        relayActivated = false;      // ✅ 릴레이 상태 초기화
        Serial.println("✅ 인증 성공!");
    } else {
        Serial.println("❌ 인증 실패. 블랙리스트에 추가 후 연결 종료 예정.");
        String centralMAC = central.address();
        char macStr[18];
        centralMAC.toCharArray(macStr, sizeof(macStr));
        for (int i = 0; i < 17; i++) macStr[i] = toupper(macStr[i]);
        addToBlacklist(macStr);
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
            Serial.println("⏱ 인증 타임아웃. 연결 해제합니다.");
            String centralMAC = currentCentral.address();
            char macStr[18];
            centralMAC.toCharArray(macStr, sizeof(macStr));
            for (int i = 0; i < 17; i++) macStr[i] = toupper(macStr[i]);
            addToBlacklist(macStr);
            Serial.print("🚫 블랙리스트 등록됨: ");
            Serial.println(macStr);
            currentCentral.disconnect();
            connectedCentral = BLEDevice();
            authSuccess = false;
            authChecked = false;
            currentState = ADVERTISING;
            Serial.println("🔌 연결 해제 완료. 광고는 유지됩니다.");
        }
    }
}

void ble_init() {
    if (!BLE.begin()) {
        Serial.println("❌ BLE 초기화 실패!");
        return;
    }
    Serial.println("✅ BLE 초기화 완료");
    randomSeed(analogRead(A0));
    generateRandomNonce(nonce, sizeof(nonce));
    generateHMAC_SHA256(sharedKey, nonce, tokenHex);
    Serial.print("Generated Nonce: ");
    Serial.println(nonce);
    Serial.print("Expected Auth Token: ");
    Serial.println(tokenHex);
    setupGattService();
    initLists();
}

void ble_run() {
    BLEDevice central = BLE.central();
    int docking = digitalRead(DOCKING_PIN);

    if (docking == HIGH) {
        if (dockingOkStartTime == 0) dockingOkStartTime = millis();
        if (!isAdvertising && millis() - dockingOkStartTime >= 3000) {
            Serial.println("📢 BLE Advertising 시작: DM-STATION");
            BLE.advertise();
            isAdvertising = true;
            currentState = ADVERTISING;
        }
    } else {
        dockingOkStartTime = 0;
        if (isAdvertising) {
            Serial.println("🛑 BLE Advertising 중지 (DOCKING_PIN LOW)");
            BLE.stopAdvertise();
            isAdvertising = false;
        }
        if (connectedCentral && connectedCentral.connected()) {
            Serial.println("🛑 Docking LOW 상태 - BLE 연결 강제 해제");
            connectedCentral.disconnect();
            connectedCentral = BLEDevice();
        }
        currentState = IDLE;
        return;
    }

    if (central) {
        if (!connectedCentral && central.connected()) {
            Serial.println("📶 Central 연결 감지!");
            connectedCentral = central;
            authSuccess = false;
            authChecked = false;
            authStartTime = millis();
            currentState = CONNECTING;
        }

        if (connectedCentral && connectedCentral.connected()) {
            if (!authChecked && millis() - authStartTime > 1000) {
                authChecked = true;
                String centralMAC = connectedCentral.address();
                char macStr[18];
                centralMAC.toCharArray(macStr, sizeof(macStr));
                for (int i = 0; i < 17; i++) macStr[i] = toupper(macStr[i]);
                Serial.print("🔍 MAC 주소 확인: ");
                Serial.println(macStr);
                if (isInBlacklist(macStr)) {
                    Serial.println("⛔ 블랙리스트 장치. 연결 해제합니다.");
                    connectedCentral.disconnect();
                    connectedCentral = BLEDevice();
                    return;
                }
                if (isWhitelisted(macStr)) {
                    Serial.println("✅ 화이트리스트 장치. 인증 자동 성공!");
                    authSuccess = true;
                    authSuccessTime = millis();
                    relayActivated = false;
                    return;
                }
                Serial.println("🟡 인증 대기 중 (HMAC 방식)");
            }

            if (authSuccess) {
                updateGattValues();
                currentState = CONNECTED;

                // ✅ 인증 성공 후 10초 경과 시 릴레이 ON
                if (!relayActivated && millis() - authSuccessTime >= 10000) {
                    digitalWrite(RELAY_PIN, HIGH);
                    jumperRelayChar.writeValue(1);
                    Serial.println("🔗 10초 경과 - Charger Jumper Relay ON");
                    relayActivated = true;
                }
            } else {
                checkAuthTimeout();
            }
        } else {
            if (connectedCentral) {
                Serial.println("🔌 연결 끊김 감지");
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
}

void ble_reset() {
    Serial.println("♻️ BLE 리셋: 연결 해제 + 초기화");
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