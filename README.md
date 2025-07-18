# DMBOT_Charging_System

DMBOT Charging System by Arduino Nano RP2040 (BLE based)  
충전 시스템은 BLE 통신 기반으로 Robot과 Station이 상호작용하며 충전 여부를 제어하는 구조로 설계됨.

- **Revision**: 2025-07-18 REV 3.0 확정  
- **환경에 따라 수정 가능한 항목**:  
  - Robot: RSSI 거리 기준  
  - Station: ADC 기반 Voltage 기준 (배터리 상태)

---

## Summary

### 🔋 Station (충전기 제어부)
- **도킹 상태 판단**: Docking Pin(D8) 입력 감지 (300ms 이상 HIGH 유지 시 도킹으로 판단)
- **전압 측정**: A0 핀에서 분압 회로를 통해 배터리 전압 모니터링  
  - 분압 비: `R1 = 200kΩ`, `R2+Pulldown = 10kΩ‖10kΩ = 5kΩ` → 약 1/41  
  - `Vin = A0 × 41`

- **릴레이 제어 조건 (Remote Control 제어용)**:
  | 입력 전압(Vin)     | 릴레이 상태 | 동작 설명                         |
  |--------------------|-------------|-----------------------------------|
  | < 45.92V           | OFF         | 저전압 or 단선, 충전 금지         |
  | 45.92V ~ 51.04V    | ON          | 정상 충전 범위                    |
  | 51.05V ~ 54.11V    | 유지        | 히스테리시스 중립 상태            |
  | ≥ 54.12V           | OFF         | 과충전 보호 (FULL 처리)          |
  | 단선 감지 (A0 이상/이하) | OFF   | Remote 핀 Open → NPB 출력 0V로 차단 |

- **NPB-750 충전기 Remote 제어**  
  - 릴레이로 7–8번 핀 쇼트 시 충전 시작 (`Vboost = 57.6V`)  
  - A0 전압 기반 과충전 감지 시 Remote 핀 Open → 출력 0V로 안전 차단

---

### 🤖 Robot (중앙 제어부)
- **BLE Central 역할**로 Station과 연결
- **RSSI 거리 기반 Docking 판단**  
- 릴레이 제어 전 BLE 인증 및 Station과의 연결 상태 필터링 포함
- 충전이 필요한 조건에서 Station에 릴레이 ON 신호 전송

---

### 시스템 동작 시퀀스 요약

```mermaid
graph TD
    A[Robot -> BLE 연결 시도] --> B{RSSI 필터 통과 여부}
    B -- No --> A
    B -- Yes --> C[BLE 인증 완료 -> 연결 성공]
    C --> D[Station: 도킹 핀 HIGH 유지 -> DOCKING_OK 상태]
    D --> E{A0 전압 < 54.12V 인가?}
    E -- Yes --> F[Relay ON -> Remote Pin 쇼트 -> 충전 시작]
    E -- No --> G[Relay OFF -> Remote Pin Open -> 충전 차단]
    G --> H[NPB-750 출력 전압 0V로 차단됨]
    H --> I{A0 전압 ≤ 50V로 다시 떨어짐?}
    I -- Yes --> F
