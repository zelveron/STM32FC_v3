#pragma once
//
// bmp581.hpp -- Bosch BMP581 barometric pressure / temperature sensor.
//
// I2C1, addr 0x47. Normal mode at 50 Hz. Call poll() at 100 Hz; it returns
// fresh data-ready measurements without a conversion delay in the task.
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
bool healthy();
int error();

// Returns true for a new data-ready event, with the fresh reading in `out`.
bool poll(Sample& out);

} // namespace bmp581
