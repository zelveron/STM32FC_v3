#include "arming.hpp"
#include <cmath>

namespace core {
namespace {
constexpr float kArmThrottleMax = 0.05f;   // must be at idle to arm
}

void Arming::update(const ArmInputs& in)
{
    if (!in.permitted) {
        _state = ArmState::disarmed;
        _seen_low = false;
        _resume_after_loss = false;
        _prev_switch = in.arm_switch;
        return;
    }
    if (in.failsafe_active) {
        // User-selected automatic recovery. Retain only prior authorization;
        // a cold boot or an unarmed link outage can never acquire it.
        if (_state == ArmState::armed) _resume_after_loss = true;
        _state = ArmState::disarmed;
        _seen_low = false;
        _prev_switch = in.arm_switch;
        return;
    }
    if (_resume_after_loss) {
        _resume_after_loss = false;
        _state = in.arm_switch ? ArmState::armed : ArmState::disarmed;
        _prev_switch = in.arm_switch;
        _seen_low = !in.arm_switch;
        return;
    }
    if (!in.arm_switch) _seen_low = true;
    if (_state == ArmState::armed) {
        if (!in.arm_switch || in.failsafe_active)
            _state = ArmState::disarmed;
    } else {
        const bool rising = in.arm_switch && !_prev_switch;
        if (rising && _seen_low && std::isfinite(in.throttle) &&
            in.throttle >= 0 && in.throttle <= kArmThrottleMax)
            _state = ArmState::armed;
    }
    _prev_switch = in.arm_switch;
}

} // namespace core
