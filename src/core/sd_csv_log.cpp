#include "sd_csv_log.hpp"

// BENCH SCAFFOLDING -- see sd_csv_log.hpp. Arduino / STM32SD only, excluded from native.
#include <Arduino.h>
#include <STM32SD.h>

namespace sd_csv_log {
namespace {

File     s_file;
bool     s_ok = false;
char     s_name[16] = "";
uint32_t s_last_sync_ms = 0;

} // namespace

Result begin()
{
    if (!SD.begin()) { s_ok = false; return Result::begin_failed; }

    int n = 0;
    do { snprintf(s_name, sizeof(s_name), "FLT%05d.CSV", n++); }
    while (SD.exists(s_name));

    s_file = SD.open(s_name, FILE_WRITE);
    if (!s_file) { s_ok = false; return Result::open_failed; }

    s_file.println(F("t_ms,ax,ay,az,gx,gy,gz,roll,pitch,yaw,press_hPa,temp_c,alt_m,gps_time,gps_sats,gps_speed_kmh"));
    s_file.flush();
    s_ok = true;
    return Result::ok;
}

bool        ok()   { return s_ok; }
const char* name() { return s_name; }

void write_row(const Row& r)
{
    if (!s_ok) return;

    s_file.print(r.t_ms);
    s_file.print(',');
    s_file.print(r.ax_g, 4); s_file.print(',');
    s_file.print(r.ay_g, 4); s_file.print(',');
    s_file.print(r.az_g, 4); s_file.print(',');
    s_file.print(r.gx_dps, 2); s_file.print(',');
    s_file.print(r.gy_dps, 2); s_file.print(',');
    s_file.print(r.gz_dps, 2); s_file.print(',');
    s_file.print(r.roll_deg, 1);  s_file.print(',');
    s_file.print(r.pitch_deg, 1); s_file.print(',');
    s_file.print(r.yaw_deg, 1);   s_file.print(',');
    s_file.print(r.press_hpa, 3); s_file.print(',');
    s_file.print(r.temp_c, 2);    s_file.print(',');
    s_file.print(r.alt_m, 2);     s_file.print(',');
    s_file.print(r.gps_time);     s_file.print(',');
    s_file.print(r.gps_sats);     s_file.print(',');
    s_file.println(r.gps_speed_kmh, 1);

    if ((millis() - s_last_sync_ms) >= 1000) {
        s_last_sync_ms = millis();
        s_file.flush();
    }
}

} // namespace sd_csv_log
