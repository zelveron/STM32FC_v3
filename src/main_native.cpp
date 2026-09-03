//
// main_native.cpp -- [env:native] entry point.
//
// For now this only proves that the portable layers (hal.hpp, drivers/,
// estimation/) compile and link against the native hal backend. Unit tests
// and the SITL harness attach here in later phases.
//
#include "hal/hal.hpp"
#include "estimation/ahrs.hpp"

int main()
{
    hal::init();

    // Smoke: a level, still IMU sample should leave attitude near zero.
    ahrs::reset();
    for (int i = 0; i < 5; i++)
        ahrs::update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f);

    return 0;
}
