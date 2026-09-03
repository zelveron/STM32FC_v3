#include "ahrs.hpp"
#include <cmath>

namespace ahrs {
namespace {

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;

float s_roll_rad  = 0.0f;
float s_pitch_rad = 0.0f;
float s_yaw_rad   = 0.0f;
bool  s_init      = false;

float wrap_pi(float a)
{
    while (a >  kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

} // namespace

void reset() { s_init = false; }

void update(float ax_g, float ay_g, float az_g,
            float gx_dps, float gy_dps, float gz_dps,
            float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;

    // Roll / pitch from the gravity vector (valid only when not accelerating).
    const float acc_roll  = std::atan2(ay_g, az_g);
    const float acc_pitch = std::atan2(-ax_g, std::sqrt(ay_g * ay_g + az_g * az_g));

    if (!s_init) {
        s_roll_rad  = acc_roll;
        s_pitch_rad = acc_pitch;
        s_yaw_rad   = 0.0f;
        s_init      = true;
        return;
    }

    // Gyro body rates -> Euler angle rates (rad/s).
    const float p = gx_dps * kDeg2Rad;
    const float q = gy_dps * kDeg2Rad;
    const float r = gz_dps * kDeg2Rad;

    const float sp = std::sin(s_roll_rad);
    const float cp = std::cos(s_roll_rad);
    const float tt = std::tan(s_pitch_rad);
    float ct = std::cos(s_pitch_rad);
    if (std::fabs(ct) < 0.1f) ct = (ct < 0.0f) ? -0.1f : 0.1f;

    const float phi_dot   = p + sp * tt * q + cp * tt * r;
    const float theta_dot = cp * q - sp * r;
    const float psi_dot   = (sp / ct) * q + (cp / ct) * r;

    // Blend gyro integration (fast, drifts) with the accel reference (noisy,
    // no drift). alpha = 0.98 -> ~0.5 s time constant at 100 Hz.
    const float alpha = 0.98f;
    s_roll_rad  = alpha * (s_roll_rad  + phi_dot   * dt_s) + (1.0f - alpha) * acc_roll;
    s_pitch_rad = alpha * (s_pitch_rad + theta_dot * dt_s) + (1.0f - alpha) * acc_pitch;
    s_yaw_rad   = wrap_pi(s_yaw_rad + psi_dot * dt_s);
}

float roll_rad()  { return s_roll_rad; }
float pitch_rad() { return s_pitch_rad; }
float yaw_rad()   { return s_yaw_rad; }

} // namespace ahrs
