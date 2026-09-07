#include "baro_alt.hpp"
#include <cmath>

namespace estimation {

void BaroAlt::set_ground(float press_pa)
{
    if (press_pa > 1000.0f) _p0 = press_pa;
}

float BaroAlt::alt_m(float press_pa) const
{
    if (_p0 <= 0.0f || press_pa <= 0.0f) return 0.0f;
    return 44330.0f * (1.0f - std::pow(press_pa / _p0, 0.190263f));
}

} // namespace estimation
