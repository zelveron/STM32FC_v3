#pragma once
//
// arming.hpp -- arm / disarm state machine.
//
// Disarmed by default. Arm only on a deliberate rising edge of the arm switch,
// with throttle at idle and no failsafe. Disarm immediately on switch-low or
// failsafe. While disarmed the ESC outputs are forced to min (motors off);
// control surfaces still follow the sticks for pre-flight checks -- that is
// the caller's job, using armed().
//
// Portable.
//
#include <cstdint>

namespace core {

enum class ArmState : uint8_t { disarmed, armed };

struct ArmInputs {
    bool  arm_switch      = false;   // aux channel above threshold
    float throttle        = 0.0f;    // [0,1]
    bool  failsafe_active = false;
};

class Arming {
public:
    void update(const ArmInputs& in);

    ArmState state() const { return _state; }
    bool     armed() const { return _state == ArmState::armed; }

private:
    ArmState _state       = ArmState::disarmed;
    bool     _prev_switch = false;
};

} // namespace core
