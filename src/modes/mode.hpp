#pragma once
//
// mode.hpp -- flight mode interface. One transmitter channel, three positions:
// MANUAL / ASSIST / AUTO. Sensor faults demote downward only, never promote.
//
// Modes are static instances (no heap). update() maps pilot sticks to the 8
// normalized outputs; enter() preloads internal state for a bumpless switch.
//
// Portable.
//
#include <cstdint>
#include "../control/mixer.hpp"

namespace modes {

enum class Id : uint8_t { manual, assist, auto_ };

class Mode {
public:
    virtual ~Mode() = default;

    virtual Id          id()   const = 0;
    virtual const char* name() const = 0;

    // Called once when this mode becomes active. `current` is the last set of
    // outputs, so the mode can preload integrators for a bumpless transition.
    virtual void enter(const control::Outputs& current) = 0;

    // Per control tick.
    virtual void update(const control::Sticks& sticks, float dt_s,
                        control::Outputs& out) = 0;
};

} // namespace modes
