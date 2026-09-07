#pragma once
//
// mode.hpp -- flight mode interface. One transmitter channel, three positions:
// MANUAL / ASSIST / AUTO. Sensor faults demote downward only, never promote.
//
// Modes are static instances (no heap). update() maps the pilot input +
// estimator feedback to the 8 normalized outputs; enter() preloads internal
// state (PID integrators) for a bumpless switch.
//
// Portable.
//
#include <cstdint>
#include "../control/mixer.hpp"

namespace modes {

enum class Id : uint8_t { manual, assist, auto_ };

struct ModeInput {
    control::Sticks sticks;          // normalized pilot commands
    float dt_s = 0.0025f;

    // estimator feedback -- ASSIST / AUTO use it, MANUAL ignores it.
    float roll_rad = 0.0f, pitch_rad = 0.0f, yaw_rad = 0.0f;
    float gyro_p_dps = 0.0f, gyro_q_dps = 0.0f, gyro_r_dps = 0.0f;
    float airspeed_mps = 0.0f;       // <= 0 -> unknown (turn comp skipped)

    // false -> stabilizer integrators frozen (aircraft on the ground: a steady
    // attitude error would otherwise wind the rate-loop I term to its rail).
    // MANUAL ignores it.
    bool  allow_integrators = true;
};

class Mode {
public:
    virtual ~Mode() = default;

    virtual Id          id()   const = 0;
    virtual const char* name() const = 0;

    // Called once when this mode becomes active. `current` = the outputs the
    // previous mode was producing, so integrators can be preloaded for a
    // bumpless transition.
    virtual void enter(const control::Outputs& current) = 0;

    virtual void update(const ModeInput& in, control::Outputs& out) = 0;
};

} // namespace modes
