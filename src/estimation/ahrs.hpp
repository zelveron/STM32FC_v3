#pragma once
//
// ahrs.hpp -- attitude estimate: gated complementary filter (Mahony) with
// online gyro-bias estimation.
//
// Quaternion state (no Euler gimbal lock). The accel correction is gated: it
// is trusted only when |accel| is near 1 g and the body rate is low, so
// launch / gusts / maneuvering do not pull the attitude toward the specific-
// force vector. When the accel is not trusted the estimate coasts on the
// (bias-corrected) gyro.
//
// Yaw is still unreferenced -- no magnetometer. With the online bias estimate
// its drift is small; for fixed-wing, GPS ground course covers heading once
// moving (CLAUDE.md).
//
// Frame: body FRD, world NED. +roll = right wing down, +pitch = nose up,
// +yaw = clockwise from above. State is radians.
//
// Portable.
//
#include <cstdint>

namespace ahrs {

// Re-seed roll/pitch from the next sample's accel; zero yaw; clear bias.
void reset();

// accel in g, gyro in deg/s, dt in seconds. (IMU is expected pre-conditioned:
// bias-calibrated + low-passed -- see estimation/imu_prep.)
void update(float ax_g, float ay_g, float az_g,
            float gx_dps, float gy_dps, float gz_dps, float dt_s);

float roll_rad();
float pitch_rad();
float yaw_rad();

// --- diagnostics ---
void  gyro_bias_dps(float& bx, float& by, float& bz);
float acc_trust();   // 0..1 weight applied to the last accel correction

// --- tuning (sane defaults set at first use) ---
void  set_gains(float kp, float ki);

} // namespace ahrs
