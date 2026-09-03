#pragma once
//
// bmp581.hpp -- Bosch BMP581 barometric pressure / temperature sensor.
//
// I2C1, addr 0x47. Forced mode, non-blocking: begin() once, then call poll()
// often; it triggers a conversion on its own ~10 Hz cadence and returns the
// result on a later call once the conversion has had time to finish. No delay().
//
// Depends on hal:: and the vendored lib/bmp5 Bosch API. No Arduino / STM32.
//
#include <cstdint>

namespace bmp581 {

struct Sample {
    float pressure_pa;
    float temp_c;
};

bool begin();

// Drives the trigger/collect state machine. Returns true exactly once per
// conversion, with the fresh reading in `out`.
bool poll(Sample& out);

} // namespace bmp581
