#pragma once
//
// srv_channel.hpp -- normalized command -> servo / ESC pulse (microseconds).
//
// This is the ONLY place a normalized control value becomes microseconds
// (CLAUDE.md conventions). Portable.
//
#include <cstdint>

namespace control {

struct SrvChannel {
    uint16_t min_us    = 1000;
    uint16_t center_us = 1500;
    uint16_t max_us    = 2000;
    bool     reversed  = false;

    // cmd in [-1, +1]; 0 -> center_us. Clamped to [min_us, max_us].
    uint16_t from_norm(float cmd) const;

    // throttle in [0, 1]; 0 -> min_us.
    uint16_t from_unipolar(float thr) const;

    // Pulse for a disarmed / failsafe channel.
    uint16_t safe_us(bool is_throttle) const {
        return is_throttle ? min_us : center_us;
    }
};

} // namespace control
