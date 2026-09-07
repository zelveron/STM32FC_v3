#pragma once
//
// mode_assist.hpp -- ASSIST: stick commands a clamped attitude ANGLE
// (ArduPlane FBWA equivalent). Angle loop -> rate loop -> mixer.
//
//   roll stick  -> target roll  (+/- max_roll_rad)
//   pitch stick -> target pitch (+/- max_pitch_rad)
//   yaw stick   -> direct yaw-rate demand
//   throttle    -> passthrough
//
// The rate-loop outputs go through the SAME mixer as MANUAL (output map,
// flaperon / differential-thrust scaffolding).
//
// enter() preloads the rate-loop integrators so the first output equals the
// current servo position -- a servo snap at 40 m/s loses the airframe.
//
#include "mode.hpp"
#include "../control/attitude_ctrl.hpp"
#include "../control/rate_ctrl.hpp"

namespace modes {

class ModeAssist : public Mode {
public:
    Id          id()   const override { return Id::assist; }
    const char* name() const override { return "ASSIST"; }

    void configure(const control::AttitudeCtrlConfig& att,
                   const control::RateCtrlConfig& rate,
                   float max_roll_rad, float max_pitch_rad,
                   float max_yaw_rate_dps);

    void enter(const control::Outputs& current) override;
    void update(const ModeInput& in, control::Outputs& out) override;

    control::MixParams mix;

    // diagnostics
    const control::RateController& rate_ctrl() const { return _rate; }

private:
    control::AttitudeController _att;
    control::RateController     _rate;
    float _max_roll = 0.7f, _max_pitch = 0.5f, _max_yaw_rate = 90.0f;

    bool             _need_preset = false;
    bool             _prev_allow  = true;   // edge-detect integrator re-enable
    control::Outputs _entry_out{};
};

} // namespace modes
