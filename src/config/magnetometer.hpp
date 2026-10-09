#pragma once
#include "../estimation/mag_heading.hpp"
namespace config {
// Replaced by tools/calibrate_mag.py from a recorded 3D rotation session.
// No BMM350 mounting/calibration has been measured in this checkout yet.
// Diagnostics stay available while magnetic yaw/heading hold await setup.
inline constexpr estimation::MagHeadingConfig magnetic_calibration{};
}
