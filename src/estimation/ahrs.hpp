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
// Gyro yaw is referenced by accepted BMM350 observations through correct_yaw.
// Without them it coasts and can drift. GNSS course is never body heading.
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
bool valid();
// Preserve attitude when changing to an independently calibrated gyro.
void clear_residual_bias();

// accel is NEGATIVE body-FRD specific force, in g (+Z when level at rest).
// Gyro is body-FRD angular velocity in deg/s. dt in seconds. Sensor mounting
// rotation and the accel sign conversion belong at the driver boundary.
// (IMU is expected pre-conditioned:
// bias-calibrated + low-passed -- see estimation/imu_prep.)
void update(float ax_g, float ay_g, float az_g,
            float gx_dps, float gy_dps, float gz_dps, float dt_s);

float roll_rad();
float pitch_rad();
float yaw_rad();
// Apply a validated world-Z rotation without disturbing roll, pitch or bias.
void correct_yaw(float delta_rad);

// --- diagnostics ---
void  gyro_bias_dps(float& bx, float& by, float& bz);
float acc_trust();   // 0..1 weight applied to the last accel correction

// --- tuning (sane defaults set at first use) ---
void  set_gains(float kp, float ki);

} // namespace ahrs
