# Review findings and firmware changes

Baseline: STM32FC_v2 commit `3daf518b5e1759f63fb84008d8d34c5e17c0dda6`. Target: the user's manufactured v2.2 board and confirmed operating preferences. The resulting firmware is maintained in STM32FC_v3. No board was flashed during this development work.

## Original firmware findings

| Review finding | Change | Status and evidence |
|---|---|---|
| R01: any valid CRSF traffic could preserve stale RC commands | Only complete CRC-valid channel packets refresh RC age; one 200 ms timeout, bounded drain and partial-frame recovery | Application regression covers link-statistics-only traffic, short packets and stale control |
| R02: SD waits could hang; low-priority writes still stalled cooperative control | Boot-only FAT32 preallocation and runtime nonblocking SDIO DMA state machine | Card-command/ring/recovery models tested; real-card timing/power-loss qualification pending |
| R03: missing/stale/invalid IMU data could still be treated as healthy | New BMI270 drivers check identity/error/FIFO/saturation/progress/freshness; estimator eligibility propagated to mode handling | Real Bosch API + SPI/FIFO fault emulator, application demotion/failsafe tests |
| R04: latest-register reads at a lower rate discarded IMU samples and invited aliasing | Sensor filtering plus 400 Hz paired FIFO consumption; bounded backlog rejection | Driver checks sample order/scaling/count; physical vibration/alias rejection still needs measurement |
| R05: integrator preload limits could create a mode-entry output step | Reset controllers and slew stabilized surfaces from the previous output; throttle bypasses slew | Application/controller pulse-continuity tests |
| R06: enabling integrators could reuse stale entry outputs | Removed entry preload reuse; integration freeze/thaw does not reset output state | Mode-entry and integrator-enable regression |
| R07: constant rotation could be accepted as stationary gyro calibration | Absolute rate, gravity magnitude, finite/time checks and motion-span restart; no calibration after first arming | Constant 30 deg/s and invalid-gravity tests; slow motion below thresholds remains a documented limitation |
| R08: acceleration frame/sign could yield an incorrect horizon | Explicit proper mounting rotation and specific-force sign conversion for each new IMU | PCB/datasheet mapping plus level-attitude host tests; physical six-orientation/sign check still required |
| R09: stale or invalid GPS could remain usable | Validity/field checks, double-precision parsing, explicit invalid-fix handling and age expiry | NMEA malformed/void/no-fix/stale regressions |
| R10: host/receiver baud changes could desynchronize GNSS | SAM-M10Q default 38400, passive host-side discovery, no unacknowledged receiver reconfiguration | UART command-capture regression |
| R11: barometer advertised 50 Hz but effective forced sequence was about 10 Hz | Normal 50 Hz configuration, pressure oversampling, explicit ready-source enable and fresh-event polling | Real Bosch API + I2C register model tests; physical ODR/timing validation pending |
| R12: disarm could reset the altitude reference while airborne | First-arm flight-session latch holds the pressure datum across disarm and RC loss | Actual application regression |
| R13: USB bootloader command could reboot an armed controller | DFU requires disarm/idle; after arming also fresh RC and completed failsafe recovery. Overflowed/binary lines discarded; simulated-flight override removed; reset-first ROM entry | Application USB command tests |
| R14: logging could miss final partial data and hide flush failures | Sector envelopes, asynchronous tail drain, session/sequence/CRC recovery, owned-extent bounds | Logger/card fault models and Python recovery tests; actual card behavior still untested |

## v2-specific implementation

- F407VG flash/linker profile and confirmed 8 MHz crystal; actual SPI1/SPI2/I2C1/I2C2/UART4/USART2 pins.
- Dedicated ER8 UART4 on J2, freeing PB10/PB11 for BMM350 as the board requires.
- Actual timer-capable outputs; physical SERVO7/PD11 unused; motors on physical SERVO8/9.
- 50 Hz PWM defaults and per-output endpoint/reversal configuration.
- Two official Bosch BMI270 API instances with separate calibration/filter state, primary-to-backup failover, a disagreement latch and one-time startup FIFO discard.
- Official BMM350 acquisition for diagnostics; no uncalibrated magnetic heading fused.
- User-selected throttle-cut/level failsafe and automatic throttle restoration after stable reception.
- Control timing fault latch and watchdog containment; watchdog-recovery boot keeps motors inhibited.
- GUI updated for dual-IMU health and stale attitude/barometer/GPS displays; firmware suppresses normal attitude streaming when unhealthy.
- New hardware, transmitter, fusion, validation and feature documentation; historical instructions preserved separately.

## Remaining material limitations

The actual manufactured board has the electrical blockers described in [HARDWARE_V2.md](HARDWARE_V2.md). No physical sensor, clock, GPIO, PWM, receiver-loss, watchdog or flight test was available. The default binary is a motor-inhibited development build.

The attitude estimator is a simple gated complementary filter. It can drift or misinterpret sustained specific force. There is no full navigation filter, absolute yaw fusion, airspeed/stall protection or independent controller redundancy. A matched pair of IMUs cannot identify the correct sensor solely from a plausible disagreement.

The cooperative scheduler still depends on bounded execution in every hardware call. A watchdog limits a stall's duration but does not preserve control during reset. Analog rail failure, output pin failure, reboot and ESC behavior require hardware tests. Output drivers cannot verify actual servo motion or motor stoppage.

Default gains, mounting axes, servo reversals, CG/trim, control authority and aerodynamic behavior need validation on the real airframe. The toy SITL model is a software regression tool, not evidence that the real aircraft is stable. Surface slew changes effective control response and needs flight-dynamics validation too.

Physical SD qualification, flash parameters, measured six-face/temperature calibration, magnetometer airframe calibration, validated UBX navigation, battery/current sensing, automated takeoff, autonomous navigation and MAVLink remain outstanding future work. See the [feature catalog](FEATURES.md) for the exact boundary of implemented behavior.
