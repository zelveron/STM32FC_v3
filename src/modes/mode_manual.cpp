#include "mode_manual.hpp"

namespace modes {

void ModeManual::update(const control::Sticks& sticks, float /*dt_s*/,
                        control::Outputs& out)
{
    control::mix_manual(sticks, params, out);
}

} // namespace modes
