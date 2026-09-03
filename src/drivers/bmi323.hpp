#pragma once
//
// bmi323.hpp -- Bosch BMI323 IMU (accel + gyro).
//
// TEMPORARY: transferred over bit-bang SPI on the hal GPIO path because the
// BMI323 solder joints are marginal and hardware SPI fails (README blocker #1).
// One accel+gyro read is ~1.1-1.7 ms of blocking CPU. Do not build flight
// sampling on this -- it is replaced by hardware SPI + DMA + FIFO after the
// reflow is verified.
//
// Depends on hal:: and the vendored lib/bmi323 Bosch API. No Arduino / STM32.
//
#include <cstdint>

namespace bmi323 {

struct Sample {
    float ax_g, ay_g, az_g;      // g
    float gx_dps, gy_dps, gz_dps; // deg/s
};

// Configure the four bit-bang GPIO lines (idempotent). Call once at boot.
void config_pins();

// Soft-reset, verify chip id 0x43, apply the accel/gyro config. false on any
// failure. Safe to retry.
bool begin();

// One accel+gyro burst read with raw->physical scaling applied. false on a
// bus/comm error.
bool read(Sample& out);

// Raw single-transaction CHIP_ID (reg 0x00) read, no config. 0x43 = present.
uint8_t raw_chip_id();

} // namespace bmi323
