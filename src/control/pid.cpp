#include "pid.hpp"
#include <cmath>

namespace control {
namespace {
constexpr float kTwoPi = 6.28318530717958647692f;
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float lpf_alpha(float fc, float dt)
{
    if (fc <= 0.0f || dt <= 0.0f) return 1.0f;
    const float rc = 1.0f / (kTwoPi * fc);
    return dt / (rc + dt);
}
} // namespace

void Pid::configure(const PidGains& g, float sample_hz)
{
    _g = g;
    const float dt = (sample_hz > 0.0f) ? 1.0f / sample_hz : 0.0f;
    _d_alpha = lpf_alpha(g.d_lpf_hz, dt);
}

void Pid::reset()
{
    _i = _i_before = 0.0f;
    _prev_meas = 0.0f;
    _d_filt = 0.0f;
    _primed = false;
    _p = _d = 0.0f;
}

float Pid::update(float setpoint, float measurement, float dt_s)
{
    if (!std::isfinite(dt_s) || dt_s <= 0 || dt_s > 0.1f ||
        !std::isfinite(setpoint) || !std::isfinite(measurement)) { reset(); return 0; }
    _d_alpha = lpf_alpha(_g.d_lpf_hz,dt_s);
    _i_before = _i;
    const float err = setpoint - measurement;

    // --- D on measurement (filtered) ---
    if (!_primed) { _prev_meas = measurement; _d_filt = 0.0f; _primed = true; }
    const float d_raw = -(measurement - _prev_meas) / dt_s;
    _prev_meas = measurement;
    _d_filt += _d_alpha * (d_raw - _d_filt);
    _d = _g.kd * _d_filt;

    _p = _g.kp * err;
    const float ff = _g.kff * setpoint;

    // --- unsaturated output with the current integrator ---
    float out = ff + _p + _i + _d;

    // --- integrate with anti-windup (freeze while pushing further into a rail,
    //     or while integration is externally disabled e.g. on the ground) ---
    if (_integ_enabled && _g.i_max > 0.0f && _g.ki != 0.0f) {
        const bool sat_hi = out >= _g.out_max;
        const bool sat_lo = out <= _g.out_min;
        if (!((sat_hi && err > 0.0f) || (sat_lo && err < 0.0f))) {
            _i = clampf(_i + _g.ki * err * dt_s, -_g.i_max, _g.i_max);
            out = ff + _p + _i + _d;
        }
    }

    return clampf(out, _g.out_min, _g.out_max);
}

void Pid::track_applied(float requested,float applied)
{
    if(!std::isfinite(requested)||!std::isfinite(applied)) { _i=0; return; }
    if((requested-applied)*(_i-_i_before)>0) _i=_i_before;
}

void Pid::preset_integrator(float setpoint, float measurement, float desired_out)
{
    _primed = true;
    _prev_meas = measurement;
    _d_filt = 0.0f;
    _d = 0.0f;
    _p = _g.kp * (setpoint - measurement);
    const float ff = _g.kff * setpoint;
    float i = desired_out - ff - _p;             // d assumed ~0 at entry
    i = _g.i_max > 0.0f ? clampf(i, -_g.i_max, _g.i_max) : 0.0f;
    _i = i;
}

} // namespace control
