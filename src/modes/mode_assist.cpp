#include "mode_assist.hpp"

namespace modes {

void ModeAssist::configure(const control::AttitudeCtrlConfig& att,
                           const control::RateCtrlConfig& rate,
                           float max_roll_rad, float max_pitch_rad,
                           float max_yaw_rate_dps)
{
    _att.configure(att);
    _rate.configure(rate);
    _max_roll     = max_roll_rad;
    _max_pitch    = max_pitch_rad;
    _max_yaw_rate = max_yaw_rate_dps;
}

void ModeAssist::enter(const control::Outputs& current)
{
    _entry_out   = current;
    _need_preset = true;    // rate integrators preloaded on the first update()
    _prev_allow  = true;    // enter()'s preset covers the initial state
}

void ModeAssist::update(const ModeInput& in, control::Outputs& out)
{
    // stick -> target angle (clamped)
    const float tgt_roll  = in.sticks.roll  * _max_roll;
    const float tgt_pitch = in.sticks.pitch * _max_pitch;

    // outer loop: angle error -> desired body rates
    float des_p, des_q;
    _att.update(tgt_roll, tgt_pitch, in.roll_rad, in.pitch_rad,
                in.airspeed_mps, des_p, des_q);
    const float des_r = in.sticks.yaw * _max_yaw_rate;

    // Ground / not-flying: freeze the rate-loop integrators. On the transition
    // back to enabled (e.g. arming), re-preset from the current outputs so
    // integration restarts bumplessly from a clean state instead of a wound rail.
    _rate.set_integrator_enabled(in.allow_integrators);
    if (in.allow_integrators && !_prev_allow) _need_preset = true;
    _prev_allow = in.allow_integrators;

    // bumpless: preload the rate integrators so the first output == entry pos
    if (_need_preset) {
        _rate.preset(des_p, des_q, des_r,
                     in.gyro_p_dps, in.gyro_q_dps, in.gyro_r_dps,
                     _entry_out.ch[0], _entry_out.ch[2], _entry_out.ch[4]);
        _need_preset = false;
    }

    // inner loop: rate error -> surface commands
    float r_cmd, p_cmd, y_cmd;
    _rate.update(des_p, des_q, des_r,
                 in.gyro_p_dps, in.gyro_q_dps, in.gyro_r_dps, in.dt_s,
                 r_cmd, p_cmd, y_cmd);

    // through the same mixer / output map as MANUAL; throttle is passthrough
    const control::Sticks mixed { r_cmd, p_cmd, y_cmd, in.sticks.throttle };
    control::mix_manual(mixed, mix, out);
}

} // namespace modes
