#pragma once
//
// rate_ctrl.hpp -- inner loop: desired body rate -> surface command.
//
// Three axis PIDs (roll/pitch/yaw), each running on the gyro. Output is a
// normalized surface command in [-1,+1] with the standard signs:
//   +roll  -> roll right   +pitch -> pitch up   +yaw -> yaw right
//
// FF dominates on a fixed wing (surface ~ desired rate): kff first.
//
// Portable.
//
#include "pid.hpp"

namespace control {

struct RateCtrlConfig {
    PidGains roll;
    PidGains pitch;
    PidGains yaw;
    float    sample_hz = 400.0f;
};

class RateController {
public:
    void configure(const RateCtrlConfig& c);
    void reset();

    // rates in deg/s; outputs in [-1,+1].
    void update(float des_p, float des_q, float des_r,
                float gyro_p, float gyro_q, float gyro_r, float dt_s,
                float& out_roll, float& out_pitch, float& out_yaw);

    // Bumpless entry: preset each integrator so the next update() reproduces
    // the given current surface commands.
    void preset(float des_p, float des_q, float des_r,
                float gyro_p, float gyro_q, float gyro_r,
                float cur_roll, float cur_pitch, float cur_yaw);

    const Pid& roll_pid()  const { return _roll; }
    const Pid& pitch_pid() const { return _pitch; }
    const Pid& yaw_pid()   const { return _yaw; }

private:
    Pid _roll, _pitch, _yaw;
};

} // namespace control
