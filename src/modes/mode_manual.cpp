#include "mode_manual.hpp"

namespace modes {

void ModeManual::update(const ModeInput& in, control::Outputs& out)
{
    control::mix_manual(in.sticks, params, out);
}

} // namespace modes
