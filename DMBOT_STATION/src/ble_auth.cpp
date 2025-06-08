#include "ble_auth.h"
#include "blacklist.h"
#include <ArduinoBLE.h>
#include <Crypto.h>
#include <SHA256.h>
#include <HMAC.h>
#include <mbed.h>
#include <string.h>

// BLE 서비스 및 특성
BLEService authService("180A");
BLECharacteristic nonceChar("2A29", BLERead, 20);
BLECharacteristic tokenChar("2A2A", BLEWrite, 64);
BLECharacteristic chargerStateChar("2A2C", BLERead, 10);

// BLE 연결 상태
BLEDevice central;
char currentNonce[20] = "";
bool authenticated = false;
bool connected = false;
char connectedMac[18] = "";

// 인증 타임아웃 관리
unsigned long authStartTime = 0;
unsigned long authTimeoutMs = 5000;
const unsigned long AUTH_TIMEOUT_MS_NORMAL = 5000;
const unsigned long AUTH_TIMEOUT_MS_BLACKLIST = 500;

// 시크릿 키
const char *SECRET_KEY = "DMBOT_SECRET";

// 도킹 체크 핀
const int DOCKING_PIN = 8;
bool lastDockingState = LOW;
bool isAdvertising = false;
unsigned long dockingChangeTime = 0;
const unsigned long DEBOUNCE_MS = 1000;

void generateRandomNonce(char *out, size_t len = 16) {
    for (size_t i = 0; i < len; i++) {
        out[i] = 'A' + (rand() % 26);
    }
    out[len] = '\0';
}

void generateNonce(char *outNonce) {
    generateRandomNonce(outNonce);
    strcpy(currentNonce, outNonce);
    nonceChar.writeValue(currentNonce);
}

bool computeHMAC(const char *key, const char *message, char *outHex) {
    HMAC<SHA256> hmac((const uint8_t *)key, strlen(key));
    hmac.update((const uint8_t *)message, strlen(message));

    uint8_t result[32];
    hmac.finalize(result, sizeof(result));

    for (int i = 0; i < 32; ++i) {
        sprintf(outHex + (i * 2), "%02x", result[i]);
    }
    outHex[64] = '\0';
    return true;
}

bool validateToken(const char *mac, const char *token) {
    char expected[65];
    computeHMAC(SECRET_KEY, currentNonce, expected);

    if (strncmp(token, expected, 64) == 0) {
        authenticated = true;
        Serial.println("[BLE] 인증 성공");
        connected = true;
        return true;
    } else {
        Serial.println("[BLE] 인증 실패 → 블랙리스트 등록");
        addToBlacklist(mac);
        resetBLEState();
        return false;
    }
}

void resetBLEState() {
    BLE.disconnect();
    connected = false;
    authenticated = false;
    strcpy(connectedMac, "");
    authStartTime = 0;
    authTimeoutMs = AUTH_TIMEOUT_MS_NORMAL;
    generateNonce(currentNonce);
    nonceChar.writeValue(currentNonce);
}

void setupBLE() {
    pinMode(DOCKING_PIN, INPUT);
    lastDockingState = digitalRead(DOCKING_PIN);
    dockingChangeTime = millis();  // 디바운스 기준 시점 초기화

    if (!BLE.begin()) {
        Serial.println("[BLE] BLE 시작 실패");
        return;
    }

    BLE.setLocalName("DM-STATION");
    BLE.setAdvertisedService(authService);
    authService.addCharacteristic(nonceChar);
    authService.addCharacteristic(tokenChar);
    authService.addCharacteristic(chargerStateChar); 
    BLE.addService(authService);

    generateNonce(currentNonce);
    nonceChar.writeValue(currentNonce);

    Serial.println("[BLE] 초기화 완료");
}

void updateBLEStateMachine() {
    BLE.poll();

    // 도킹 상태 모니터링
    bool currentDockingState = digitalRead(DOCKING_PIN);
    if (currentDockingState != lastDockingState) {
        dockingChangeTime = millis();
        lastDockingState = currentDockingState;
    }

    if (!connected) {
        if (currentDockingState == HIGH && !isAdvertising &&
            millis() - dockingChangeTime >= DEBOUNCE_MS) {
            BLE.advertise();
            isAdvertising = true;
            Serial.println("[BLE] 도킹 감지됨 → 광고 시작");
        }

        if (currentDockingState == LOW && isAdvertising &&
            millis() - dockingChangeTime >= DEBOUNCE_MS) {
            BLE.stopAdvertise();
            isAdvertising = false;
            Serial.println("[BLE] 도킹 해제됨 → 광고 중지");
        }
    }

    // 연결 처리
    if (!connected) {
        BLEDevice centralDevice = BLE.central();
        if (centralDevice && centralDevice.connected()) {
            const char *mac = centralDevice.address().c_str();

            if (isBlacklisted(mac)) {
                Serial.print("[BLE] 블랙리스트 MAC 연결 시도: ");
                Serial.println(mac);
                authTimeoutMs = AUTH_TIMEOUT_MS_BLACKLIST;
            } else {
                authTimeoutMs = AUTH_TIMEOUT_MS_NORMAL;
            }

            central = centralDevice;
            strncpy(connectedMac, mac, sizeof(connectedMac));
            connectedMac[sizeof(connectedMac) - 1] = '\0';
            connected = true;
            authStartTime = millis();
            generateNonce(currentNonce);
            nonceChar.writeValue(currentNonce);
            Serial.print("[BLE] 연결 시도 MAC: ");
            Serial.println(connectedMac);
            Serial.println("[BLE] 인증 대기 중...");
        }
    }

    // 연결 해제 감지
    if (connected && central && !central.connected()) {
        Serial.println("[BLE] 연결 해제됨");
        resetBLEState();
        return;
    }

    // 인증 타임아웃
    if (connected && !authenticated &&
        authStartTime > 0 && millis() - authStartTime > authTimeoutMs) {
        Serial.println("[BLE] 인증 타임아웃 → 연결 종료");
        addToBlacklist(connectedMac);
        resetBLEState();
        return;
    }

    // 인증 처리
    if (connected && !authenticated && tokenChar.written()) {
        char token[65];
        tokenChar.readValue(token, sizeof(token));
        validateToken(connectedMac, token);
    }
}