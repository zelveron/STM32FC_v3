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
    _roll_rate.reset(); _seed=true;
    _transition.enter(current);
}

void ModeTakeoff::update(const ModeInput& in, control::Outputs& out)
{
    // roll: stick -> clamped bank target -> angle-P -> desired roll rate
    const float tgt_roll = clampf(in.sticks.roll * _max_roll, -_max_roll, _max_roll);
    const float error=std::atan2(std::sin(tgt_roll-in.roll_rad),std::cos(tgt_roll-in.roll_rad));
    const float heading=std::fabs(std::cos(in.pitch_rad))>.25f ?
        control::bound((in.gyro_q_dps*std::sin(in.roll_rad)+in.gyro_r_dps*std::cos(in.roll_rad))/
                       std::cos(in.pitch_rad),-90,90):0;
    const float des_p=clampf(_roll_p*error-heading*std::sin(in.pitch_rad),-_max_rollrate,_max_rollrate);

    // Freeze integration on the ground; surface slew bounds roll output changes.
    if(_seed) { _demand.reset(in.gyro_p_dps); _seed=false; }
    _dp=_demand.update(des_p,_tuning.roll_accel_dps2,in.dt_s);
    if(!in.allow_integrators) _roll_rate.clear_integrator();
    _roll_rate.set_integrator_enabled(in.allow_integrators);

    const float r_cmd = _roll_rate.update(_dp, in.gyro_p_dps, in.dt_s);

    // pitch / yaw / throttle: straight through, exactly as MANUAL
    const float scale=control::airspeed_scale(in.airspeed_mps,in.airspeed_valid,_tuning);
    const control::Sticks mixed { r_cmd*scale, in.sticks.pitch, in.sticks.yaw,
                                  in.sticks.throttle };
    control::mix_manual(mixed, mix, out);
    _transition.apply(out, in.dt_s, true);
    const float applied=std::fabs(mix.aileron_gain)>1e-6f?out.ch[0]/mix.aileron_gain/scale:0;
    _roll_rate.track_applied(r_cmd,applied);
}

} // namespace modes
