# Repository guidance: STM32FC_v3 on PCB v2.2

Current behavior and hardware are documented in README.md and docs/HARDWARE_V2.md. The user authorized the v2 hardware migration; historical v1 constraints are archived in docs/history/CLAUDE-v1.md and do not define the new board.

Assembly correction confirmed 2026-10-02: the physical board uses TWO BMI323s and NO magnetometer. The default `v2_bmi323` profile selects that assembly; `v2` remains the separate planned BMI270 target. See docs/ASSEMBLED_BOARD.md and do not confuse the profiles.

Use the supplied v2.2 KiCad pin map: F407VG, dual BMI270, BMP581, BMM350, SAM-M10Q, UART4/J2 for ER8. Preserve the user's choice of automatic throttle restoration after stable RC recovery. Clearly distinguish implementation, host simulation, hardware tests and flight qualification.

Keep sensor/control modules portable through hal.hpp. Arduino boundaries remain in the STM32 HAL, application entry, USB and SD adapters. Keep allocations and unbounded work out of control tasks. The scheduler is cooperative; assigning a task lower priority does not make blocking I/O safe. Runtime SD uses a bounded command/DMA state machine over a boot-preallocated FAT32 extent. Never add filesystem calls or blocking HAL command waits to its runtime path. Real-card timing qualification is still pending.

Use the exact Bosch APIs and configuration image with their license/provenance files. Do not copy ArduPilot source. Do not change transmitter assignments, sensor axes, failsafe semantics or motor gating without updating the corresponding documentation and behavioral regressions.

Run tools/test_host.ps1 for control/driver changes and compile the v2 target. Do not treat passing simulation as airframe gain validation. Do not flash connected hardware without a user request for flashing. Never put credentials in source, command output or repository configuration.
