#include "failsafe.hpp"

namespace core {
namespace {
constexpr uint32_t kEngageMs  = 200;   // link must be lost this long to engage
constexpr uint32_t kRecoverMs = 300;   // ...and stable this long to clear
}

void Failsafe::update(bool link_ok, uint32_t now_ms)
{
    if (link_ok != _link_ok) {
        _link_ok  = link_ok;
        _since_ms = now_ms;
    }
    const uint32_t stable = now_ms - _since_ms;

    if (_link_ok) {
        if (_level == FailsafeLevel::rc_loss && stable >= kRecoverMs)
            _level = FailsafeLevel::none;
    } else {
        if (_level == FailsafeLevel::none && stable >= kEngageMs)
            _level = FailsafeLevel::rc_loss;
    }
}

} // namespace core
