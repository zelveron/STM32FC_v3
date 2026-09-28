#pragma once
//
// failsafe.hpp -- layered failsafe.
//
// Tracks RC loss using the driver's channel-data freshness. Starts engaged
// and clears after 300 ms of usable reception. Application policy cuts motors
// and levels with healthy calibrated attitude, or centers surfaces otherwise.
// See docs/TX16S_SETUP.md for the user-selected automatic throttle recovery.
//
#include <cstdint>

namespace core {

enum class FailsafeLevel : uint8_t {
    none,      // all nominal
    rc_loss,   // no valid RC -- hold safe outputs, wait for a stable recovery
};

class Failsafe {
public:
    // Call every control tick. link_ok = a valid RC frame within the driver's
    // timeout. now_ms from hal::millis().
    void update(bool link_ok, uint32_t now_ms);

    FailsafeLevel level()  const { return _level; }
    bool          active() const { return _level != FailsafeLevel::none; }

private:
    FailsafeLevel _level    = FailsafeLevel::rc_loss;   // safe until RC proven
    uint32_t      _since_ms = 0;                        // when _link_ok changed
    bool          _link_ok  = false;
};

} // namespace core
