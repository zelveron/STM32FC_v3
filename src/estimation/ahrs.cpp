#include "ahrs.hpp"
#include <cmath>

namespace ahrs {
namespace {

constexpr float kPi      = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;

// accel-trust gate. Tight magnitude band: a coordinated turn or a throttle
// surge adds ~0.3-0.5 g of horizontal specific force, which tilts the measured
// "gravity" -- reject it and coast on the gyro. (Kinematic / centripetal
// compensation from GPS velocity comes with the nav filter, Phase 5.)
constexpr float kMagBandG   = 0.10f;   // |a| must be within this of 1 g
constexpr float kRateLoDps  = 30.0f;   // full trust below this body rate
constexpr float kRateHiDps  = 120.0f;  // zero trust above this

float s_q[4]    = { 1, 0, 0, 0 };   // w,x,y,z  body->world
float s_bias[3] = { 0, 0, 0 };      // rad/s
float s_kp      = 1.0f;
float s_ki      = 0.05f;
float s_trust   = 0.0f;
bool  s_init    = false;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void q_from_accel(float ax, float ay, float az)
{
    // roll/pitch from the gravity direction; yaw 0.
    const float roll  = std::atan2(ay, az);
    const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const float cr = std::cos(roll * 0.5f),  sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    s_q[0] = cr * cp;
    s_q[1] = sr * cp;
    s_q[2] = cr * sp;
    s_q[3] = -sr * sp;
}

void normalize()
{
    float n = std::sqrt(s_q[0]*s_q[0] + s_q[1]*s_q[1] + s_q[2]*s_q[2] + s_q[3]*s_q[3]);
    if (n < 1e-9f) { s_q[0] = 1; s_q[1] = s_q[2] = s_q[3] = 0; return; }
    const float inv = 1.0f / n;
    for (int i = 0; i < 4; i++) s_q[i] *= inv;
}

} // namespace

void reset()
{
    s_init = false;
    s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
    s_trust = 0.0f;
}

void set_gains(float kp, float ki) { s_kp = kp; s_ki = ki; }

void update(float ax_g, float ay_g, float az_g,
            float gx_dps, float gy_dps, float gz_dps, float dt_s)
{
    if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.01f;

    if (!s_init) {
        const float m = std::sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        if (m > 0.1f) q_from_accel(ax_g / m, ay_g / m, az_g / m);
        else          { s_q[0] = 1; s_q[1] = s_q[2] = s_q[3] = 0; }
        s_bias[0] = s_bias[1] = s_bias[2] = 0.0f;
        s_init = true;
        return;
    }

    float wx = gx_dps * kDeg2Rad;
    float wy = gy_dps * kDeg2Rad;
    float wz = gz_dps * kDeg2Rad;

    // --- accel-trust weight ---
    const float amag = std::sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    float w = 0.0f;
    if (amag > 0.1f) {
        const float mag_w  = 1.0f - std::fabs(amag - 1.0f) / kMagBandG;      // 1 at |a|=1g
        const float rate   = std::sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
        const float rate_w = (kRateHiDps - rate) / (kRateHiDps - kRateLoDps);
        w = clampf(mag_w, 0.0f, 1.0f) * clampf(rate_w, 0.0f, 1.0f);
    }
    s_trust = w;

    if (w > 0.0f) {
        // gravity direction in body from the current quaternion (3rd row of R)
        const float qw=s_q[0], qx=s_q[1], qy=s_q[2], qz=s_q[3];
        const float vx = 2.0f * (qx*qz - qw*qy);
        const float vy = 2.0f * (qy*qz + qw*qx);
        const float vz = qw*qw - qx*qx - qy*qy + qz*qz;

        const float inv = 1.0f / amag;
        const float axn = ax_g * inv, ayn = ay_g * inv, azn = az_g * inv;

        // e = a_meas x v_est  (correction in the gyro frame)
        const float ex = ayn*vz - azn*vy;
        const float ey = azn*vx - axn*vz;
        const float ez = axn*vy - ayn*vx;

        // Mahony explicit complementary filter (b_dot = -Ki e ; w_hat = w - b + Kp e)
        s_bias[0] -= s_ki * w * ex * dt_s;
        s_bias[1] -= s_ki * w * ey * dt_s;
        s_bias[2] -= s_ki * w * ez * dt_s;

        wx += s_kp * w * ex;
        wy += s_kp * w * ey;
        wz += s_kp * w * ez;
    }

    wx -= s_bias[0];
    wy -= s_bias[1];
    wz -= s_bias[2];

    // q_dot = 0.5 * q (x) [0, w]
    const float qw=s_q[0], qx=s_q[1], qy=s_q[2], qz=s_q[3];
    s_q[0] += 0.5f * (-qx*wx - qy*wy - qz*wz) * dt_s;
    s_q[1] += 0.5f * ( qw*wx + qy*wz - qz*wy) * dt_s;
    s_q[2] += 0.5f * ( qw*wy - qx*wz + qz*wx) * dt_s;
    s_q[3] += 0.5f * ( qw*wz + qx*wy - qy*wx) * dt_s;
    normalize();
}

float roll_rad()
{
    const float w=s_q[0], x=s_q[1], y=s_q[2], z=s_q[3];
    return std::atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
}
float pitch_rad()
{
    const float w=s_q[0], x=s_q[1], y=s_q[2], z=s_q[3];
    float sp = 2.0f * (w*y - z*x);
    sp = clampf(sp, -1.0f, 1.0f);
    return std::asin(sp);
}
float yaw_rad()
{
    const float w=s_q[0], x=s_q[1], y=s_q[2], z=s_q[3];
    return std::atan2(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
}

void gyro_bias_dps(float& bx, float& by, float& bz)
{
    bx = s_bias[0] * kRad2Deg;
    by = s_bias[1] * kRad2Deg;
    bz = s_bias[2] * kRad2Deg;
}
float acc_trust() { return s_trust; }

} // namespace ahrs
