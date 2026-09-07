#include "mixer.hpp"

namespace control {
namespace {
float clamp1 (float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }
float clamp01(float v) { return v <  0.0f ?  0.0f : (v > 1.0f ? 1.0f : v); }
}

void mix_manual(const Sticks& in, const MixParams& p, Outputs& out)
{
    const float ail = clamp1(in.roll  * p.aileron_gain);
    const float ele = clamp1(in.pitch * p.elevator_gain);
    const float rud = clamp1(in.yaw   * p.rudder_gain);

    out.ch[0] = ail;                              // aileron L
    out.ch[1] = ail;                              // aileron R
    out.ch[2] = ele;                              // elevator L
    out.ch[3] = ele;                              // elevator R
    out.ch[4] = rud;                              // rudder
    out.ch[5] = clamp1(in.yaw * p.nosewheel_gain);// nosewheel steering

    float dt = 0.0f;
    if (in.throttle > p.diff_thrust_min_throttle)
        dt = in.yaw * p.diff_thrust_gain;
    out.ch[6] = clamp01(in.throttle + dt);        // ESC L
    out.ch[7] = clamp01(in.throttle - dt);        // ESC R
}

} // namespace control
