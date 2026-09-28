#pragma once
//
// mode_takeoff.hpp -- TKOFF: wings-level roll assist, everything else manual.
//
// The takeoff roll, rotation and climb-out are flown by the pilot. This mode
// closes ONLY the roll loop: roll stick -> clamped bank angle -> rate loop ->
// aileron. Pitch, yaw and throttle pass straight through to the mixer exactly
// as MANUAL, so nothing fights the elevator while the pilot rotates and climbs
// away. Roll authority is deliberately small -- a wing-leveller with a little
// crosswind correction, not an aerobatic bank.
//
// Procedure: take off in TKOFF, then switch to ASSIST once settled in the
// climb. MANUAL is always the bail-out.
//
// Same bumpless-entry and ground-integrator-freeze behaviour as ASSIST.
//
#include "mode.hpp"
#include "../control/pid.hpp"
#include "../control/transition.hpp"

namespace modes {

class ModeTakeoff : public Mode {
public:
    Id          id()   const override { return Id::takeoff; }
    const char* name() const override { return "TKOFF"; }

    //   roll_pid            -- inner rate-loop gains (reuse the ASSIST roll set)
    //   sample_hz           -- control-loop rate
    //   roll_p_dps_per_rad  -- angle-loop P
    //   max_roll_rate_dps   -- rate demand clamp (keep it gentle near the ground)
    //   max_roll_rad        -- bank authority (small: crosswind / gentle turn)
    void configure(const control::PidGains& roll_pid, float sample_hz,
                   float roll_p_dps_per_rad, float max_roll_rate_dps,
                   float max_roll_rad);

    void enter(const control::Outputs& current) override;
    void update(const ModeInput& in, control::Outputs& out) override;

    control::MixParams mix;
    void set_tuning(const control::AssistTuning& t) { _tuning=t; _transition.configure(t.surface_rate_per_s,t.transition_s); }
    float demand_p() const { return _dp; }

    const control::Pid& roll_pid() const { return _roll_rate; }   // diagnostics

private:
    control::Pid _roll_rate;
    float _roll_p       = 110.0f;
    float _max_rollrate = 120.0f;
    float _max_roll     = 0.175f;   // ~10 deg -- crosswind wing-low only

    control::SurfaceTransition _transition;
    control::AssistTuning _tuning;
    control::RateDemand _demand;
    bool _seed=true;
    float _dp=0;
};

} // namespace modes
