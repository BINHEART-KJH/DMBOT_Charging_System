#include "ble_scanner.h"
#include <ArduinoBLE.h>
#include <Crypto.h>
#include <SHA256.h>
#include <HMAC.h>
#include <string.h>

#define RELAY_PIN 4 // D4

static bool scanning = false;
static bool connected = false;
BLEDevice station;
BLECharacteristic nonceChar;
BLECharacteristic tokenChar;
BLECharacteristic chargerStateChar;

const char *SECRET_KEY = "DMBOT_SECRET";

bool isConnectedToStation()
{
    return connected;
}

void setupBLEScanner()
{
    if (!BLE.begin())
    {
        Serial.println("[BLE] 초기화 실패");
        return;
    }

    BLE.setLocalName("DM-ROBOT");
    Serial.println("[BLE] 초기화 완료");
}

void startScan()
{
    if (!scanning)
    {
        BLE.scan();
        scanning = true;
        Serial.println("[BLE] BLE 스캔 시작");
    }
}

void stopScan()
{
    if (scanning)
    {
        BLE.stopScan();
        scanning = false;
        Serial.println("[BLE] BLE 스캔 중단");
    }
}

void disconnectFromStation()
{
    if (connected && station)
    {
        station.disconnect(); // ✅ 연결 해제
        connected = false;
        Serial.println("[BLE] 수동 연결 해제 요청 → 연결 끊김");
    }
}

bool computeHMAC(const char *key, const char *message, char *outHex)
{
    HMAC<SHA256> hmac((const uint8_t *)key, strlen(key));
    hmac.update((const uint8_t *)message, strlen(message));
    uint8_t result[32];
    hmac.finalize(result, sizeof(result));
    for (int i = 0; i < 32; ++i)
    {
        sprintf(outHex + i * 2, "%02x", result[i]);
    }
    outHex[64] = '\0';
    return true;
}

void updateBLEScanLoop()
{
    BLE.poll();

    if (scanning)
    {
        BLEDevice peripheral = BLE.available();
        if (peripheral)
        {
            String localName = peripheral.localName();
            int rssi = peripheral.rssi();

            if (localName == "DM-STATION" && rssi > -80)
            {
                Serial.println("[BLE] DM-STATION 발견 → 연결 시도");
                stopScan();

                if (peripheral.connect())
                {
                    Serial.println("[BLE] 연결 성공");
                    delay(200);

                    // GATT 서비스 탐색
                    if (peripheral.discoverAttributes())
                    {
                        nonceChar = peripheral.characteristic("2A29");
                        tokenChar = peripheral.characteristic("2A2A");
                        chargerStateChar = peripheral.characteristic("2A2C");

                        if (nonceChar && tokenChar && chargerStateChar)
                        {
                            Serial.println("[BLE] 인증 특성 발견");

                            char nonce[20] = "";
                            if (nonceChar.readValue(nonce, sizeof(nonce)))
                            {
                                Serial.print("[BLE] nonce 수신: ");
                                Serial.println(nonce);

                                char token[65];
                                computeHMAC(SECRET_KEY, nonce, token);
                                if (tokenChar.writeValue(token))
                                {
                                    Serial.println("[BLE] token 전송 완료 → 인증 완료");
                                    station = peripheral;
                                    connected = true;
                                    delay(200);
                                }
                                else
                                {
                                    Serial.println("[BLE] token 전송 실패");
                                    BLE.disconnect();
                                }
                            }
                            else
                            {
                                Serial.println("[BLE] nonce 읽기 실패");
                                BLE.disconnect();
                            }
                        }
                        else
                        {
                            Serial.println("[BLE] GATT 인증 특성 누락");
                            BLE.disconnect();
                        }
                    }
                    else
                    {
                        Serial.println("[BLE] GATT 탐색 실패");
                        BLE.disconnect();
                    }
                }
                else
                {
                    Serial.println("[BLE] 연결 실패");
                }
            }
        }
    }

    // 연결 유지 중일 때 chargerStateChar 모니터링
    if (connected)
    {
        if (!station.connected())
        {
            Serial.println("[BLE] 연결 해제 감지");
            connected = false;
            digitalWrite(RELAY_PIN, LOW); // 릴레이 OFF
            startScan();
            return;
        }

        if (chargerStateChar)
        {
            char chargerState[10] = "";

            static bool lastRelayState = false;

            if (chargerStateChar.readValue(chargerState, sizeof(chargerState)))
            {
                String stateStr = String(chargerState);
                stateStr.trim();

                bool shouldRelayBeOn = (stateStr == "ON");

                if (shouldRelayBeOn != lastRelayState)
                {
                    lastRelayState = shouldRelayBeOn;
                    digitalWrite(RELAY_PIN, shouldRelayBeOn ? HIGH : LOW);

                    Serial.print("[BLE] chargerState 원본 값: [");
                    Serial.print(stateStr);
                    Serial.println("]");

                    if (shouldRelayBeOn)
                    {
                        Serial.println("[RELAY] 상태 변경 → 릴레이 ON");
                    }
                    else
                    {
                        Serial.println("[RELAY] 상태 변경 → 릴레이 OFF");
                    }
                }
            }
        }
    }
}