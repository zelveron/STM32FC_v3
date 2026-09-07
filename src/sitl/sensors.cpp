#include "sensors.hpp"
#include <cmath>

namespace sitl {

Sensors synth(const State& s, double home_lat, double home_lon)
{
    Sensors o{};
    constexpr double g   = 9.80665;
    constexpr double r2d = 57.29577951308232;

    // Model accel_body is specific force with gravity removed and z DOWN; the
    // real BMI323 reads az = +1 g when level, ax negative for nose-up. Negate.
    o.ax_g = (float)(-s.accel_body.x / g);
    o.ay_g = (float)(-s.accel_body.y / g);
    o.az_g = (float)(-s.accel_body.z / g);

    o.gx_dps = (float)(s.omega_body.x * r2d);
    o.gy_dps = (float)(s.omega_body.y * r2d);
    o.gz_dps = (float)(s.omega_body.z * r2d);

    const double alt = -s.pos_ned.z;
    const double p_pa = 101325.0 * std::pow(1.0 - alt / 44330.0, 1.0 / 0.1902632);
    o.press_hpa = (float)(p_pa / 100.0);
    o.temp_c    = 15.0f;

    const double kLatM = 111320.0;
    o.lat_deg = home_lat + s.pos_ned.x / kLatM;
    o.lon_deg = home_lon + s.pos_ned.y / (kLatM * std::cos(home_lat * M_PI / 180.0));
    o.alt_m   = alt;
    o.gspeed_kmh = (float)(std::sqrt(s.vel_ned.x * s.vel_ned.x +
                                     s.vel_ned.y * s.vel_ned.y) * 3.6);
    return o;
}

} // namespace sitl
