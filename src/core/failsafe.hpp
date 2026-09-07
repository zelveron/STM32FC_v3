#pragma once
//
// failsafe.hpp -- layered failsafe.
//
// !!! Do not modify this file as a side effect of another change (CLAUDE.md
// hard rule 7). A silent failsafe change is worse than none. !!!
//
// Today it tracks one thing: RC link loss. When engaged, the caller drives
// every output to its safe pulse (throttle min, surfaces centred). It starts
// engaged and only clears after the link has been stable for a recovery
// window. Debounced both ways.
//
// Later: sensor-fault demotion (AUTO -> ASSIST -> MANUAL), latched and
// announced over CRSF telemetry.
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
