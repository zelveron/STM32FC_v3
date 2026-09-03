#pragma once
//
// bmi323.hpp -- Bosch BMI323 IMU (accel + gyro) over hardware SPI1.
//
// Bit-bang is gone. SPI1 (PA5/PA6/PA7), CS = PA4, ~5.25 MHz, blocking -- a
// 28-byte burst is ~45 us (vs 1436 us bit-bang). Reads are data-ready gated so
// there is no register tearing.
//
// Flight config: 1600 Hz ODR both sensors, internal filter on, +/-8 g accel,
// 2000 dps gyro, high-performance mode. FIFO + DMA is the next optimisation.
//
// Depends on hal:: and vendored lib/bmi323. No Arduino / STM32.
//
#include <cstdint>

namespace bmi323 {

struct Sample {
    float ax_g, ay_g, az_g;      // g
    float gx_dps, gy_dps, gz_dps; // deg/s
};

enum class Result : uint8_t {
    ok,          // fresh sample in `out`
    no_data,     // nothing new since the last call (DRDY not set)
    comm_error,  // SPI / sensor fault -- call begin() again
};

// Configure SPI1 and the sensor (soft reset, verify chip id 0x43, flight
// ODR/range). Safe to retry. false on any failure.
bool begin();

// Data-ready-gated read of accel + gyro with raw->physical scaling applied.
Result read(Sample& out);

// Raw single-transaction CHIP_ID (reg 0x00). 0x43 = present. Diagnostic.
uint8_t raw_chip_id();

} // namespace bmi323
