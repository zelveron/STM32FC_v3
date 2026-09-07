#include "arming.hpp"

namespace core {
namespace {
constexpr float kArmThrottleMax = 0.05f;   // must be at idle to arm
}

void Arming::update(const ArmInputs& in)
{
    if (_state == ArmState::armed) {
        if (!in.arm_switch || in.failsafe_active)
            _state = ArmState::disarmed;
    } else {
        const bool rising = in.arm_switch && !_prev_switch;
        if (rising && in.throttle <= kArmThrottleMax && !in.failsafe_active)
            _state = ArmState::armed;
    }
    _prev_switch = in.arm_switch;
}

} // namespace core
