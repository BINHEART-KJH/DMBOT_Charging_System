# DMBOT Charging System - Improvement Notes

This repository was updated to improve project safety and maintainability.

## Main changes
- Reduced race conditions in BLE state handling
- Improved auth retry logic (soft reset before hard reset)
- Separated configuration constants into config.h
- Improved ADC smoothing in Station with median+EMA logic
- Prevented relay state mismatch between Robot and Station
- Added watchdog and loop time monitoring for RP2040
- Kept original logic in comments where compatibility was important

## Files updated
- DMBOT_ROBOT/include/config.h
- DMBOT_ROBOT/include/robot_ble.h
- DMBOT_ROBOT/src/robot_ble.cpp
- DMBOT_ROBOT/src/main.cpp
- DMBOT_ROBOT/src/robot_fsm.cpp
- DMBOT_STATION/include/config.h
- DMBOT_STATION/include/station_gpio.h
- DMBOT_STATION/src/station_gpio.cpp
- DMBOT_STATION/src/station_ble.cpp
- DMBOT_STATION/src/station_fsm.cpp
- DMBOT_STATION/src/main.cpp

## Notes
- The original logic remains documented in comments to preserve traceability.
- For production deployment, hardware validation is still required on the actual RP2040 board.
