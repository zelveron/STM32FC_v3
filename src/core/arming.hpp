#pragma once
//
// arming.hpp -- arm / disarm state machine.
//
// Disarmed by default. Arm only on a deliberate rising edge of the arm switch,
// with throttle at idle and no failsafe. Disarm immediately on switch-low or
// failsafe. After stable RC recovery, prior arming authorization restores
// throttle at the current stick if CH5 remains high (user-selected policy).
// Cold boot cannot auto-arm. While disarmed ESC outputs are forced to min;
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
    bool  permitted       = true;
};

class Arming {
public:
    void update(const ArmInputs& in);

    ArmState state() const { return _state; }
    bool     armed() const { return _state == ArmState::armed; }

private:
    ArmState _state       = ArmState::disarmed;
    bool     _prev_switch = false;
    bool     _seen_low    = false;
    bool     _resume_after_loss = false;
};

} // namespace core
