#pragma once
// Fixed-wing outer attitude loop. Wrap roll error and transform Euler-rate
// requests to body rates; retain turn pitch coupling. Optional coordinated
// heading reference requires actual validated airspeed, never GPS speed.
namespace control {

struct AttitudeCtrlConfig {
    float roll_p_dps_per_rad  = 110.0f;   // ~2 dps per degree of error
    float pitch_p_dps_per_rad = 110.0f;
    float max_roll_rate_dps   = 200.0f;
    float max_pitch_rate_dps  = 120.0f;
    float turn_comp_gain      = 1.0f;     // 0 disables; ~1 holds the nose in a bank
    float turn_comp_max_dps   = 40.0f;
};

class AttitudeController {
public:
    void configure(const AttitudeCtrlConfig& c) { _c = c; }

    // Angles in radians. airspeed_mps <= 0 skips turn compensation.
    void update(float tgt_roll, float tgt_pitch,
                float meas_roll, float meas_pitch, float airspeed_mps,
                float& des_p_dps, float& des_q_dps) const;

    // Euler angle rates must be transformed to body rates in a banked turn.
    // With no measured airspeed, measured heading rate maintains pitch
    // kinematics. A washed-out yaw damper in ASSIST allows sustained turns.
    void body_rates(float target_roll,float target_pitch,float roll,float pitch,
                    float heading_rate_dps,float& p,float& q,float& r) const;
    float coordinated_heading_rate(float roll,float pitch,float airspeed) const;
private:
    AttitudeCtrlConfig _c;
};

} // namespace control
