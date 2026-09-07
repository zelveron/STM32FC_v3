#include "rc_channel.hpp"

namespace control {
namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}

float RcChannel::norm(uint16_t us) const
{
    int32_t d = (int32_t)us - (int32_t)center_us;
    if (d > -(int32_t)deadzone_us && d < (int32_t)deadzone_us) return 0.0f;
    d += (d < 0) ? (int32_t)deadzone_us : -(int32_t)deadzone_us;   // subtract the deadzone step

    const float span = (d < 0)
        ? (float)((int32_t)center_us - (int32_t)min_us - (int32_t)deadzone_us)
        : (float)((int32_t)max_us - (int32_t)center_us - (int32_t)deadzone_us);
    if (span <= 0.0f) return 0.0f;

    const float v = clampf((float)d / span, -1.0f, 1.0f);
    return reversed ? -v : v;
}

float RcChannel::unipolar(uint16_t us) const
{
    const float span = (float)((int32_t)max_us - (int32_t)min_us);
    if (span <= 0.0f) return 0.0f;
    const float v = clampf(((float)us - (float)min_us) / span, 0.0f, 1.0f);
    return reversed ? (1.0f - v) : v;
}

} // namespace control
