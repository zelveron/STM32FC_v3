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

    out.ch[0] = ail;                              // aileron / physical SERVO1
    out.ch[1] = ele;                              // elevator / physical SERVO2
    out.ch[2] = clamp01(in.throttle);              // throttle / physical SERVO3
    out.ch[3] = rud;                              // rudder / physical SERVO4
    out.ch[4] = 0.0f;                             // unused SERVO5: center
    out.ch[5] = ail;                              // SERVO6: reversed at pulse conversion
    out.ch[6] = out.ch[7] = 0.0f;                 // unused SERVO8/9: ESC idle
}

} // namespace control
