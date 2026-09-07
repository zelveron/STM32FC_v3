#include "attitude_ctrl.hpp"
#include <cmath>

namespace control {
namespace {
constexpr float kG       = 9.80665f;
constexpr float kRad2Deg = 57.29577951308232f;
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

void AttitudeController::update(float tgt_roll, float tgt_pitch,
                                float meas_roll, float meas_pitch, float airspeed_mps,
                                float& des_p_dps, float& des_q_dps) const
{
    des_p_dps = clampf(_c.roll_p_dps_per_rad  * (tgt_roll  - meas_roll),
                       -_c.max_roll_rate_dps,  _c.max_roll_rate_dps);
    float q  = clampf(_c.pitch_p_dps_per_rad * (tgt_pitch - meas_pitch),
                       -_c.max_pitch_rate_dps, _c.max_pitch_rate_dps);

    // turn compensation: pitch-up rate FF while banked
    if (_c.turn_comp_gain > 0.0f && airspeed_mps > 1.0f) {
        const float phi = clampf(meas_roll, -1.20f, 1.20f);   // |bank| <= ~69 deg
        // pitch rate the aircraft needs in a coordinated turn:
        //   psi_dot = g*tan(phi)/V ; pitch_rate = psi_dot*sin(phi)   [rad/s -> dps]
        const float ff  = _c.turn_comp_gain * kRad2Deg *
                          std::fabs(std::tan(phi)) * std::fabs(std::sin(phi)) *
                          (kG / airspeed_mps);
        q += clampf(ff, 0.0f, _c.turn_comp_max_dps);           // always nose-up
    }

    des_q_dps = clampf(q, -_c.max_pitch_rate_dps, _c.max_pitch_rate_dps);
}

} // namespace control
