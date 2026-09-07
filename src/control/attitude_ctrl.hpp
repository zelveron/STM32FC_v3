#pragma once
//
// attitude_ctrl.hpp -- outer loop: target roll/pitch angle -> desired body
// rate for the rate loop.
//
// A P controller on the angle error (rate = gain * error), rate-limited. Plus
// a turn-compensation feedforward: banking loses vertical lift, so a steady
// pitch-up rate demand proportional to tan(bank)*sin(bank)*g/V is added while
// banked, or the nose drops in every turn (CLAUDE.md control notes).
//
// Yaw is not an angle loop -- the caller feeds the yaw stick to the rate loop
// directly (optionally with a sideslip/coordination term later).
//
// Portable.
//
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

private:
    AttitudeCtrlConfig _c;
};

} // namespace control
