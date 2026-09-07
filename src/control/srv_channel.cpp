#include "srv_channel.hpp"

namespace control {
namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
uint16_t clampu(int32_t v, uint16_t lo, uint16_t hi)
{ return (v < (int32_t)lo) ? lo : ((v > (int32_t)hi) ? hi : (uint16_t)v); }
}

uint16_t SrvChannel::from_norm(float cmd) const
{
    if (reversed) cmd = -cmd;
    cmd = clampf(cmd, -1.0f, 1.0f);
    const float half = (cmd >= 0.0f) ? (float)((int32_t)max_us - (int32_t)center_us)
                                     : (float)((int32_t)center_us - (int32_t)min_us);
    const int32_t us = (int32_t)center_us +
                       (int32_t)(cmd * half + (cmd >= 0.0f ? 0.5f : -0.5f));
    return clampu(us, min_us, max_us);
}

uint16_t SrvChannel::from_unipolar(float thr) const
{
    if (reversed) thr = 1.0f - thr;
    thr = clampf(thr, 0.0f, 1.0f);
    const int32_t us = (int32_t)min_us +
                       (int32_t)(thr * (float)((int32_t)max_us - (int32_t)min_us) + 0.5f);
    return clampu(us, min_us, max_us);
}

} // namespace control
