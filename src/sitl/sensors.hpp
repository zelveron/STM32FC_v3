#pragma once
//
// sensors.hpp -- synthesize IMU / baro / GPS readings from the SITL truth
// state, in the same conventions the real drivers produce (level -> az = +1 g,
// nose-up -> ax negative, etc.). Noise-free by default.
//
#include "aircraft.hpp"

namespace sitl {

struct Sensors {
    float  ax_g,  ay_g,  az_g;
    float  gx_dps, gy_dps, gz_dps;
    float  press_hpa, temp_c;
    double lat_deg, lon_deg, alt_m;
    float  gspeed_kmh;
};

Sensors synth(const State& s, double home_lat = 39.9, double home_lon = 32.8);

} // namespace sitl
