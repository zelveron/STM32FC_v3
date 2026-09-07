#pragma once
//
// rc_channel.hpp -- raw RC pulse (microseconds) -> normalized pilot command.
//
// Portable. The inverse (normalized -> microseconds) is SrvChannel, and that
// is the ONLY place the conversion back to us happens (CLAUDE.md conventions).
//
#include <cstdint>

namespace control {

struct RcChannel {
    uint16_t min_us      = 1000;
    uint16_t center_us   = 1500;
    uint16_t max_us      = 2000;
    uint16_t deadzone_us = 8;      // applied around centre for norm()
    bool     reversed    = false;

    // Symmetric axis (roll / pitch / yaw): -1 .. +1, 0 at centre.
    float norm(uint16_t us) const;

    // Unidirectional (throttle): 0 .. 1.
    float unipolar(uint16_t us) const;
};

} // namespace control
