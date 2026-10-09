#pragma once
//
// mode_assist.hpp -- ASSIST: stick commands a clamped attitude ANGLE
// Angle loop -> body-rate geometry -> rate loop -> mixer.
//
//   roll stick  -> target roll  (+/- max_roll_rad)
//   pitch stick -> target pitch (+/- max_pitch_rad)
//   yaw stick   -> direct rudder with bounded transient yaw damping
//   throttle    -> passthrough
// Centered roll/rudder capture magnetic heading after the turn settles.
// Hold uses a limited bank demand; pilot input or loss of heading releases it.
//
// The rate-loop outputs go through the SAME mixer as MANUAL (output map,
// flaperon / differential-thrust scaffolding).
//
// enter() resets the controller and starts a bounded surface slew from the
// current outputs. This does not depend on integrator capacity.
//
#include "mode.hpp"
#include "../control/attitude_ctrl.hpp"
#include "../control/rate_ctrl.hpp"
#include "../control/transition.hpp"

namespace modes {

class ModeAssist : public Mode {
public:
    Id          id()   const override { return Id::assist; }
    const char* name() const override { return "ASSIST"; }

    void configure(const control::AttitudeCtrlConfig& att,
                   const control::RateCtrlConfig& rate,
                   float max_roll_rad, float max_pitch_rad);

    void enter(const control::Outputs& current) override;
    void update(const ModeInput& in, control::Outputs& out) override;

    control::MixParams mix;
    void set_tuning(const control::AssistTuning& t) { _tuning=t; _transition.configure(t.surface_rate_per_s,t.transition_s); }
    float demand_p() const { return _dp; }
    float demand_q() const { return _dq; }
    bool heading_hold() const { return _holding; }
    float heading_target_rad() const { return _heading_target; }
    float roll_target_rad() const { return _roll_target; }

    // diagnostics
    const control::RateController& rate_ctrl() const { return _rate; }

private:
    control::AttitudeController _att;
    control::RateController     _rate;
    float _max_roll = 0.7f, _max_pitch = 0.5f;

    control::SurfaceTransition _transition;
    control::AssistTuning _tuning;
    control::RateDemand _roll_demand,_pitch_demand;
    bool _seed=true;
    float _heading_slow=0,_dp=0,_dq=0;
    bool _holding=false;
    float _heading_target=0,_capture_s=0,_roll_target=0;
};

} // namespace modes
