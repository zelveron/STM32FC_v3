#pragma once
//
// sd_csv_log.hpp -- FLTxxxxx.CSV data logger over SDIO / FatFs.  BENCH SCAFFOLDING.
//
// *** Sits OUTSIDE the hal boundary. ***  It drives the vendored Arduino
// STM32SD / FatFs stack directly (README known issue #2: those trees are
// patched and must not be regenerated, and hal::blk_* is not wired yet). Does
// not compile for [env:native]; excluded there.
//
// This CSV format and the sprintf-of-floats cost are exactly what the binary
// ring-buffer logger (Phase 0 step 7) replaces. Kept only so the bench GUI
// keeps working during the restructure.
//
#include <cstdint>

namespace sd_csv_log {

struct Row {
    uint32_t t_ms;
    float    ax_g, ay_g, az_g;
    float    gx_dps, gy_dps, gz_dps;
    float    roll_deg, pitch_deg, yaw_deg;
    float    press_hpa, temp_c, alt_m;
    const char* gps_time;
    int      gps_sats;
    float    gps_speed_kmh;
};

enum class Result { ok, begin_failed, open_failed };

// Mount the card, open the next free FLTxxxxx.CSV, write the header row.
Result begin();

bool        ok();
const char* name();

// Append one row. Flushes to the card at most once per second.
void write_row(const Row& r);

} // namespace sd_csv_log
