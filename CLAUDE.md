# Repository guidance: STM32FC_v3 on PCB v2.2

Current behavior and hardware are documented in README.md and docs/HARDWARE_V2.md. The user authorized the v2 hardware migration; historical v1 constraints are archived in docs/history/CLAUDE-v1.md and do not define the new board.

Current card confirmed 2026-10-08: dual BMI270 is the only supported IMU configuration. The default `v2_bmi270` profile enables BMM350 acquisition and inhibits motor arming. Magnetic fusion requires measured installation calibration; see docs/MAGNETIC_HEADING.md. The user reports BMM350 online on 2026-10-09. Current bench evidence is in docs/ASSEMBLED_BOARD.md; do not substitute older assembly results.

Current aircraft output layout: SERVO1 aileron, SERVO2 elevator, SERVO3 throttle, SERVO4 rudder, SERVO6 reversed aileron. SERVO5 remains centered and SERVO8/9 remain at 1000 us. Motor safety classification includes SERVO3; never apply surface transition smoothing to it.

Use the supplied v2.2 KiCad pin map: F407VG, dual BMI270, BMP581, BMM350, SAM-M10Q, UART4/J2 for ER8. Preserve the user's choice of automatic throttle restoration after stable RC recovery. Clearly distinguish implementation, host simulation, hardware tests and flight qualification.

Keep sensor/control modules portable through hal.hpp. Arduino boundaries remain in the STM32 HAL, application entry, USB and SD adapters. Keep allocations and unbounded work out of control tasks. The scheduler is cooperative; assigning a task lower priority does not make blocking I/O safe. Runtime SD uses a bounded command/DMA state machine over a boot-preallocated FAT32 extent. Never add filesystem calls or blocking HAL command waits to its runtime path. Real-card timing qualification is still pending.

Use the exact Bosch APIs and configuration image with their license/provenance files. Do not copy ArduPilot source. Do not change transmitter assignments, sensor axes, failsafe semantics or motor gating without updating the corresponding documentation and behavioral regressions.

Run tools/test_host.ps1 on Windows or the equivalent tools/test_host.py on Linux for control/driver changes, and compile the v2 target. Run tests/mag_calibration.py when changing the compass calibration tool. Do not treat passing simulation as airframe gain validation. Do not flash connected hardware without a user request for flashing. Never put credentials in source, command output or repository configuration.
