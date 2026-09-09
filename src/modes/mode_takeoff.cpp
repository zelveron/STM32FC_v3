#include "mode_takeoff.hpp"

namespace modes {
namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

void ModeTakeoff::configure(const control::PidGains& roll_pid, float sample_hz,
                            float roll_p_dps_per_rad, float max_roll_rate_dps,
                            float max_roll_rad)
{
    _roll_rate.configure(roll_pid, sample_hz);
    _roll_p       = roll_p_dps_per_rad;
    _max_rollrate = max_roll_rate_dps;
    _max_roll     = max_roll_rad;
}

void ModeTakeoff::enter(const control::Outputs& current)
{
    _entry_out   = current;
    _need_preset = true;
    _prev_allow  = true;
}

void ModeTakeoff::update(const ModeInput& in, control::Outputs& out)
{
    // roll: stick -> clamped bank target -> angle-P -> desired roll rate
    const float tgt_roll = clampf(in.sticks.roll * _max_roll, -_max_roll, _max_roll);
    const float des_p    = clampf(_roll_p * (tgt_roll - in.roll_rad),
                                  -_max_rollrate, _max_rollrate);

    // ground: freeze the integrator; re-preset bumplessly when it re-enables
    _roll_rate.set_integrator_enabled(in.allow_integrators);
    if (in.allow_integrators && !_prev_allow) _need_preset = true;
    _prev_allow = in.allow_integrators;

    if (_need_preset) {
        _roll_rate.preset_integrator(des_p, in.gyro_p_dps, _entry_out.ch[0]);
        _need_preset = false;
    }

    const float r_cmd = _roll_rate.update(des_p, in.gyro_p_dps, in.dt_s);

    // pitch / yaw / throttle: straight through, exactly as MANUAL
    const control::Sticks mixed { r_cmd, in.sticks.pitch, in.sticks.yaw,
                                  in.sticks.throttle };
    control::mix_manual(mixed, mix, out);
}

} // namespace modes
