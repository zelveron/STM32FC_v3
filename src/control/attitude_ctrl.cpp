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
    float unused_r;
    const float heading=coordinated_heading_rate(meas_roll,meas_pitch,airspeed_mps);
    body_rates(tgt_roll,tgt_pitch,meas_roll,meas_pitch,heading,des_p_dps,des_q_dps,unused_r);
}

float AttitudeController::coordinated_heading_rate(float roll,float pitch,float v) const {
    if(_c.turn_comp_gain<=0||!std::isfinite(v)||v<8) return 0;
    return clampf(_c.turn_comp_gain*kRad2Deg*kG*std::tan(clampf(roll,-1.05f,1.05f))*
                  std::cos(pitch)/v,-_c.turn_comp_max_dps,_c.turn_comp_max_dps);
}
void AttitudeController::body_rates(float tr,float tp,float roll,float pitch,float hdg,
                                   float& p,float& q,float& r) const {
    // Wrap roll error so recovery across +/-pi takes the shortest direction.
    const float er=std::atan2(std::sin(tr-roll),std::cos(tr-roll));
    const float phi_dot=clampf(_c.roll_p_dps_per_rad*er,-_c.max_roll_rate_dps,_c.max_roll_rate_dps);
    const float theta_dot=clampf(_c.pitch_p_dps_per_rad*(tp-pitch),-_c.max_pitch_rate_dps,_c.max_pitch_rate_dps);
    p=clampf(phi_dot-hdg*std::sin(pitch),-_c.max_roll_rate_dps,_c.max_roll_rate_dps);
    q=clampf(theta_dot*std::cos(roll)+hdg*std::sin(roll)*std::cos(pitch),-_c.max_pitch_rate_dps,_c.max_pitch_rate_dps);
    r=-theta_dot*std::sin(roll)+hdg*std::cos(roll)*std::cos(pitch);
}
} // namespace control
