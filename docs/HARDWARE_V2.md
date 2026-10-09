# v2.2 hardware and connection guide

**Current assembly, 2026-10-08:** the user identifies both IMUs as **BMI270**. Use the default `v2_bmi270` profile, which enables BMM350 polling and inhibits motor authorization. The user reports BMM350 online on 2026-10-09; [heading integration](MAGNETIC_HEADING.md) now requires measured installation calibration. Current software and bench observations are recorded in [assembled-board bring-up](ASSEMBLED_BOARD.md). The design notes below are existing reference material, not new physical inspection.

This pin map was extracted from both `Plane_Board_Design.kicad_sch` and `Plane_Board_Design.kicad_pcb` in the supplied v2.2 project on 2026-09-28. PCB SHA-256: `650df54016c0231be7eda7023f24f1544af7a5d8e15cb844f49c6cadcc7422e5`. The firmware targets this F407VG board.

## Manufactured-board corrections come first

These are findings in the supplied design files, not measurements of the manufactured assembly. Check continuity and actual fitted parts on the board before rework. Firmware cannot correct these connections.

| Finding | Evidence in project | Required action |
|---|---|---|
| Missing independent MCU core decoupling | C47, 2.2 uF, connects VCAP1 (U26 pin 49) to VCAP2 (pin 73); neither end is ground | Replace that arrangement with a separate 2.2 uF low-ESR capacitor from **each** VCAP pin to ground, close to the MCU. Do not tie VCAP to 3.3 V. |
| Floating analog reference | U26 VREF+ pin 21 goes to C48; the other capacitor terminal is ground, with no DC reference feed | Supply VREF+ appropriately, normally from VDDA for this design, with the reference decoupling required by ST. Verify VDDA/VSSA too. |
| BMM350 core rail is too low | U29 VDD is on `STM1V3`; U8 TLV62569 feedback R26/R27 are both 100k | That divider calculates approximately **1.2 V**, despite the net label. BMM350 needs 1.72-1.98 V. Rework the regulator to a verified 1.8 V output; check all loads sharing the rail. A 200k upper / 100k lower divider gives a nominal 1.8 V with the fitted regulator's 0.6 V reference. Confirm stability, tolerance and fitted values before using it. |
| Magnetometer I2C pull-ups use the wrong rail | R11 and R91 pull SDA/SCL to `STM1V3`, while BMM350 VDDIO is 3.3 V | Move the pull-up supply to the valid **3.3 V I/O rail**. Fixing VDD to 1.8 V alone does not fix the logic-high level. Do not apply 3.3 V to BMM350 VDD. |
| 6S input is outside regulator operating range | PS1/PS2 TPS513885 VIN pins connect directly to VBAT/J3 | Recommended VIN ends at 24 V; a full 6S LiPo is 25.2 V before transients. **Do not power this design directly from 6S** until the input power design is corrected and qualified. An ESC's 6S rating does not qualify the FC power input. |

ST specifies separate VCAP decoupling and reference supply arrangements in the [STM32F407 datasheet, Figure 21, Table 16 and Figures 51-52](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf). BMM350 supply and input thresholds are in the [Bosch datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf). Regulator values come from [TI TLV62569](https://www.ti.com/lit/ds/symlink/tlv62569.pdf) and [TI TPS513885](https://www.ti.com/lit/ds/symlink/tps513885.pdf).

This is not a completed power-stage/layout qualification. Also inspect power-gate startup voltages and back-power paths, regulator ripple under servo/ESC load, capacitor voltage ratings, soldering, and the actual servo rail voltage. The BMP581 I2C lines contain 1k series resistors (R42/R43); firmware uses 100 kHz, but rise times and low-level voltages still need measurement.

## MCU, clock and buses

MCU U26 is **STM32F407VGT6**, with 1 MiB flash. The linker uses 128 KiB ordinary SRAM; the separate CCM area is not used as general DMA memory. User-confirmed Y2 is **8 MHz**. The pinned framework's F407VG startup uses the PLL for 168 MHz core and 48 MHz USB.

| Device/function | Connection | Firmware use |
|---|---|---|
| U4 BMI270, primary | SPI1: PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS | Filtered accel/gyro FIFO, 400 Hz, +/-8 g / +/-2000 deg/s |
| U7 BMI270, backup | SPI2: PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS | Independent driver, FIFO, calibration and health |
| U9 BMP581 | I2C1: PB6 SCL, PB7 SDA, address **0x47** | 50 Hz pressure / temperature; 100 Hz ready polling |
| U29 BMM350 | I2C2: PB10 SCL, PB11 SDA, address **0x14** | 25 Hz compensated acquisition; magnetic heading after installation calibration |
| BMM350 interrupt | PE12 through R10 | Wired but unused; driver polls ready status |
| U28 SAM-M10Q | USART2: PA2 MCU TX -> GPS RX through R95; PA3 MCU RX <- GPS TX through R94 | NMEA, initially 38400 baud; passive auto-detection |
| ER8 through J2 | UART4: PA1 MCU RX, PA0 MCU TX | Uninverted, full-duplex CRSF, 420000 baud, 8N1 |
| USB | PA11 D-, PA12 D+ | CDC diagnostics |
| microSD | SDIO PC8-PC12, PD2 | Boot-preallocated FAT32 log; asynchronous SDIO DMA runtime path (physical qualification pending) |
| U3 W25Q16 | PB0 CS, PB3 SCK, PB4 MISO, PB5 MOSI | Deselected; no parameter or flight-log implementation yet. Bus can be SPI3 despite schematic SPI1 labels. |
| U5 24AA32 | GNSS-side SDA/SCL | Not connected to an MCU I2C bus; not used as firmware parameter storage |

Both BMI270 interrupt outputs are unconnected. FIFO polling therefore replaces DRDY interrupts. The software requests 5 MHz SPI; the peripheral divider determines the actual clock. The Bosch configuration image is uploaded separately to each device at boot.

Power controls are real on this board: **PA15 LOW enables BMP power through Q2; PE3 LOW enables GPS power through Q1. PE2 HIGH asserts GPS reset through Q3; LOW releases it.** Initialization enables both supplies and releases reset. BMM350 and BMI270 rails are not individually switched by this firmware.

## ER8 wiring

J2 was originally labeled for Raspberry Pi use; the user assigned it to the ER8.

| J2 pin | PCB label | FC signal | Connect to ER8 |
|---|---|---|---|
| 1 | GND | GND | GND |
| 2 | `MCU_RX_PI_TX` | PA1 / UART4 RX | **TX** on dedicated serial connector |
| 3 | `MCU_TX_PI_RX` | PA0 / UART4 TX | **RX** on dedicated serial connector |

The `PI` part is the original Raspberry Pi naming. Read the **MCU** portion
to identify the controller's direction: ER8 TX connects to **MCU_RX**, and
ER8 RX connects to **MCU_TX**. Power down before changing the wires. The first
signal carries receiver channels into the FC; the second carries FC telemetry
back through ER8 to the transmitter. A bound radio/receiver link does not prove
either serial direction works. TX plus ground alone cannot return FC telemetry.

**J2 carries no power.** Feed ER8 from an appropriate regulated supply and join grounds. ER8 accepts 4.5-8.4 V at its power input and has a dedicated CRSF connector; use its labels/manual to identify pins, not guessed cable colors. The separate EXT voltage-sense connection is not the receiver power input. See [RadioMaster ER8 specifications](https://radiomasterrc.com/products/er8-2-4ghz-elrs-pwm-receiver).

Do not connect the LiPo directly to ER8 power or J2. Do not parallel the two ESC BEC outputs with each other or the board regulator without a designed sharing/ORing arrangement. The project connects all servo-header middle pins to **VOUT2**, so plugging an ESC's red BEC wire there connects it to that rail. Decide the supply arrangement and verify the rail before connecting ESC/servo leads. Signal and common ground are still required.

## Output connections: physical numbering matters

All listed three-pin output headers have **pin 1 = ground, pin 2 = VOUT2, pin 3 = signal**. Identify pin 1 from the actual PCB footprint; the table does not assume a viewing direction. Logical indices below match `OUT` telemetry and `servo_*` configuration arrays, using human numbering 1-8.

| Logical output | Function | PCB label / header | MCU timer |
|---|---|---|---|
| 1 | Aileron | SERVO1 / J4 | PC7, TIM3 CH2 |
| 2 | Elevator | SERVO2 / J5 | PC6, TIM3 CH1 |
| 3 | Throttle / ESC | SERVO3 / J6 | PD15, TIM4 CH4 |
| 4 | Rudder | SERVO4 / J7 | PD14, TIM4 CH3 |
| 5 | Spare (1500 us) | SERVO5 / J10 | PD13, TIM4 CH2 |
| 6 | Aileron, reversed | SERVO6 / J12 | PD12, TIM4 CH1 |
| 7 | Reserved ESC header (1000 us) | **SERVO8 / J20** | PA8, TIM1 CH1 |
| 8 | Reserved ESC header (1000 us) | **SERVO9 / J21** | PA9, TIM1 CH2 |
| Unused | No hardware PWM | **SERVO7 / J19** | PD11 has no timer PWM alternate function |
| Spare, inactive | Future output | SERVO10 / J22 | PA10, TIM1 CH3 |

PD11's alternate functions were checked against the [STM32F407 pin table](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf). Do not connect a motor to physical SERVO7 expecting logical output 7.

Each timer group shares its output frequency. Defaults are 50 Hz for ailerons, tail and motors, with 1000/1500/2000 us minimum/center/maximum. This is a conservative starting point for EMAX ES08MD II servos and the specified PWM ESC options, not a verified high-rate qualification. No DShot, bidirectional motor control, integrated power-stage commutation or ESC telemetry is implemented. Verify that the Flycolor unit is configured for the intended throttle behavior before enabling motor output.

The current aircraft mapping was selected on 2026-10-08: **SERVO1 aileron, SERVO2 elevator, SERVO3 throttle, SERVO4 rudder, SERVO6 reversed aileron**. SERVO6 has `servo_reverse[5]=true`, applied once after the common roll controller; with default endpoints its pulse is `3000 - SERVO1`. SERVO5 stays centered and SERVO8/9 stay at 1000 us. SERVO3 uses the same arming, failsafe and timing-fault protections as motor outputs, including a 1000 us pulse from startup.

TIM3 now serves SERVO1/2, TIM4 serves SERVO3/4/5/6, and TIM1 serves reserved SERVO8/9. The legacy configuration names `pwm_hz_ailerons`, `pwm_hz_tail`, `pwm_hz_motors` refer to those timer groups, not their new individual roles. Keep all groups at 50 Hz for this setup. Check each active output independently. Do not use transmitter channel reversal to compensate for an incorrectly oriented IMU or mechanically reversed individual servo.

### Schematic and PCB cross-check, 2026-10-08

The supplied `Plane_Board_Design.kicad_pcb` and a fresh KiCad schematic netlist
agree on every header signal: SERVO1/2/3/4/5/6/7/8/9/10 map to
PC7/PC6/PD15/PD14/PD13/PD12/PD11/PA8/PA9/PA10, respectively. Their MCU U26
package pins are 64/63/62/61/60/59/58/67/68/69. The STM32 Arduino variant's
timer alternate functions match all eight configured outputs. KiCad reported
no unconnected items; the design still has other DRC violations, so this
focused pin-map check is not a complete PCB qualification.

Before the mapping change, live USB first showed RC loss, then recovered to
LQ 100% with changing pilot/output commands (for example CH1 1432 us and
aileron command 1439 us). This confirms command reception and calculation,
not electrical pulse shape or physical servo movement. Separate BEC power
still requires a common ground with the FC. All output-header pin 1 pads are
GND, pin 2 pads VOUT2, and pin 3 pads the listed signal.

## Chosen mounting orientation

View the PCB in KiCad's unrotated top view. Mount with **components up and the nose toward decreasing PCB X**, the left / Y2 crystal side. Aircraft body X points toward the nose, Y toward the right wing, Z downward.

```text
                    aircraft RIGHT (+body Y)
                              ^
                              |
 aircraft NOSE  <-----  [ component side ]  -----> TAIL
      (+body X)         [     PCB v2.2    ]
                              |
                    aircraft LEFT

           +body Z points down through the board
```

Both BMI270 footprints are on F.Cu at 90 degrees. Combining this with Bosch's package axis drawing gives the configured proper rotation **body = {sensor X, -sensor Y, -sensor Z}**, independently for both chips. The AHRS then negates accelerometer specific force to obtain its gravity-like input. At rest and level, `BMI` acceleration should be approximately `0,0,+1 g`; `ATT` roll/pitch should be near zero.

The mapping derives from the PCB and the [BMI270 axis diagram, datasheet section 8.4](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf). It must still be verified on the physical assembly: right wing down gives positive roll, nose up gives positive pitch, clockwise viewed from above gives positive yaw rate. The `imu_orientation_confirmed` constant records the reviewed geometry, **not a physical acceptance test**. If mounting changes, update both transforms and repeat these checks.
