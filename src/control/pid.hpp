#pragma once
//
// pid.hpp -- PID with feedforward, anti-windup, and a filtered D term.
//
//   out = kff*setpoint + kp*error + I + kd * d(-measurement)/dt   (D-LPF'd)
//
// D is on the *measurement* (no derivative kick on a setpoint step). The
// integrator is clamped to +/-i_max and conditionally frozen while the output
// is saturated in the same direction (anti-windup). preset_integrator() forces
// I so the first update after a mode switch reproduces the current servo
// position -- bumpless entry.
//
// On a fixed wing FF dominates (surface deflection ~ desired rate): tune kff
// first, then kp, then ki (CLAUDE.md control notes).
//
// Portable.
//
namespace control {

struct PidGains {
    float kp       = 0.0f;
    float ki       = 0.0f;
    float kd       = 0.0f;
    float kff      = 0.0f;
    float i_max    = 0.0f;     // integrator clamp (abs). 0 -> I term disabled.
    float d_lpf_hz = 20.0f;    // D-term low-pass cutoff (<= 0 -> unfiltered)
    float out_min  = -1.0f;
    float out_max  =  1.0f;
};

class Pid {
public:
    void  configure(const PidGains& g, float sample_hz);
    float update(float setpoint, float measurement, float dt_s);

    // Freeze/thaw integration. While disabled the I term still contributes to
    // the output but stops accumulating -- used to stop windup against a stuck
    // plant (aircraft on the ground: a constant attitude error demands a body
    // rate that never happens, so I would otherwise pin to i_max).
    void  set_integrator_enabled(bool en) { _integ_enabled = en; }

    // Force I so that update(setpoint, measurement, dt) returns ~desired_out.
    void  preset_integrator(float setpoint, float measurement, float desired_out);
    void  reset();
    void clear_integrator() { _i=0; }
    // Undo integration which pushes further into an external actuator limit.
    // Called after mixing, transition blending and surface slew limiting.
    void track_applied(float requested,float applied);

    float p_term() const { return _p; }
    float i_term() const { return _i; }
    float d_term() const { return _d; }

private:
    PidGains _g;
    float _d_alpha   = 1.0f;
    float _i         = 0.0f;
    float _prev_meas = 0.0f;
    float _d_filt    = 0.0f;
    bool  _primed    = false;
    bool  _integ_enabled = true;
    float _p = 0.0f, _d = 0.0f, _i_before = 0.0f;
};

} // namespace control
