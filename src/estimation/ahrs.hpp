#pragma once
//
// ahrs.hpp -- attitude estimate (complementary filter).
//
// Portable: no hal, no hardware. The caller owns the clock and passes dt_s.
//
// NOT flight-grade. Fixed blend gain, no accel gating, no gyro-bias
// calibration, unreferenced (drifting) yaw. Phase 1 replaces this with a
// gated complementary AHRS + bias estimation.
//
// Frame: body FRD (X fwd, Y right, Z down). +roll = right wing down,
// +pitch = nose up, +yaw = clockwise from above. State is radians.
//
#include <cstdint>

namespace ahrs {

// Re-seed roll/pitch from the next sample's accel; zero yaw.
void reset();

// One filter step. accel in g, gyro in deg/s, dt in seconds.
void update(float ax_g, float ay_g, float az_g,
            float gx_dps, float gy_dps, float gz_dps,
            float dt_s);

float roll_rad();
float pitch_rad();
float yaw_rad();   // wrapped to [-pi, pi]

} // namespace ahrs
