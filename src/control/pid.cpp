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
    _i = 0.0f;
    _prev_meas = 0.0f;
    _d_filt = 0.0f;
    _primed = false;
    _p = _d = 0.0f;
}

float Pid::update(float setpoint, float measurement, float dt_s)
{
    if (dt_s <= 0.0f) dt_s = 1e-3f;
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

void Pid::preset_integrator(float setpoint, float measurement, float desired_out)
{
    _primed = true;
    _prev_meas = measurement;
    _d_filt = 0.0f;
    _d = 0.0f;
    _p = _g.kp * (setpoint - measurement);
    const float ff = _g.kff * setpoint;
    float i = desired_out - ff - _p;             // d assumed ~0 at entry
    if (_g.i_max > 0.0f) i = clampf(i, -_g.i_max, _g.i_max);
    _i = i;
}

} // namespace control
