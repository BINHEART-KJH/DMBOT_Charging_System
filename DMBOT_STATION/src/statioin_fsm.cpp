#include "station_fsm.h"
#include "station_gpio.h"
#include "station_auth.h"
#include "station_led.h"
#include <ArduinoBLE.h>

static StationState currentState = ST_IDLE;
static BLEDevice central;
static unsigned long authStartTime = 0;

StationState getStationState()
{
  return currentState;
}

void setStationState(StationState newState)
{
  if (newState != currentState)
  {
    currentState = newState;
    setLedState(newState);
  }
}

void stationFSM_init()
{
  BLE.begin();
  BLE.setLocalName("DMBOT-STATION");
  stationAuth_init();                                                   // 추가
  BLE.setAdvertisedServiceUuid("12345678-1234-5678-1234-56789DMBOTA1"); // 예시 UUID
}

void stationFSM_update()
{
  switch (currentState)
  {
  case ST_IDLE:
    if (isDockingPinHigh())
    {
      setStationState(ST_DOCKED);
    }
    break;

  case ST_DOCKED:
    BLE.advertise();
    setStationState(ST_ADVERTISING);
    break;

  case ST_ADVERTISING:
    central = BLE.central();
    if (central)
    {
      authStartTime = millis();
      setStationState(ST_CONNECTING);
    }
    else if (!isDockingPinHigh())
    {
      BLE.stopAdvertise();
      setStationState(ST_IDLE);
    }
    break;

  case ST_CONNECTING:
    if (central.connected())
    {
      setStationState(ST_AUTHENTICATING);
    }
    else
    {
      setStationState(ST_ADVERTISING);
    }
    break;

  case ST_AUTHENTICATING:
    stationAuth_update(central);

    if (isStationAuthenticated())
    {
      setStationState(ST_CONNECTED);
    }
    else if (!central.connected())
    {
      setStationState(ST_ADVERTISING);
    }
    else if (millis() - authStartTime > 5000)
    {
      Serial.println("[AUTH] 인증 실패 - 연결 해제");
      central.disconnect();
      stationAuth_reset();
      setStationState(ST_ADVERTISING);
    }
    else if (!isDockingPinHigh()) {
  Serial.println("[BLE] 인증 중 도킹 해제됨 → 연결 종료 및 Advertising 중지");
  central.disconnect();
  stationAuth_reset();
  BLE.stopAdvertise();
  setStationState(ST_IDLE);
}
    break;

  case ST_CONNECTED:
    if (central.connected())
    {

      // ✅ 도킹 해제되면 즉시 연결 종료
      if (!isDockingPinHigh())
      {
        Serial.println("[BLE] 도킹 해제됨 → 연결 종료 및 Advertising 중지");
        central.disconnect();
        relay_set(false);
        stationAuth_reset();
        BLE.stopAdvertise();
        currentState = ST_IDLE;
        break;
      }

      stationAuth_update(central);

      if (isStationAuthenticated())
      {
        relay_set(true);
      }
      else if (millis() - authStartTime > 5000)
      {
        Serial.println("[AUTH] 인증 실패 - 연결 종료");
        central.disconnect();
        relay_set(false);
        stationAuth_reset();
        currentState = ST_ADVERTISING;
      }
    }
    else
    {
      Serial.println("[BLE] 연결 해제됨");
      relay_set(false);
      stationAuth_reset();

      if (isDockingPinHigh())
      {
        Serial.println("[BLE] Docking_pin HIGH → Advertising 유지");
        BLE.advertise();
        currentState = ST_ADVERTISING;
      }
      else
      {
        Serial.println("[BLE] Docking_pin LOW → IDLE 상태로 전환");
        BLE.stopAdvertise();
        currentState = ST_IDLE;
      }
    }
    break;

  case ST_ERROR:
    // 오류 상태, 향후 확장
    break;
  }
}