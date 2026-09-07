//
// main_stm32.cpp -- STM32F407 application entry (Arduino setup/loop).
//
// Composition root: brings up hal + drivers + estimation, registers the bench
// tasks with the cooperative scheduler, then runs it. loop() is unused.
//
// Arduino-coupled (setup/loop symbols, F() strings). Excluded from [env:native].
//
// BEHAVIOUR CONTRACT: this still emits the same tagged-CSV stream as the flat
// streamer, at the same cadences, plus a new SCHED,* line at 1 Hz (per-task
// DWT timing + overrun counts -- the GUI ignores unknown tags). If you change
// what it emits, that is a feature change -- do it deliberately.
//
#include <Arduino.h>
#include <math.h>

#include "hal/hal.hpp"
#include "core/scheduler.hpp"
#include "drivers/bmi323.hpp"
#include "drivers/bmp581.hpp"
#include "drivers/ublox.hpp"
#include "estimation/ahrs.hpp"
#include "core/usb_stream.hpp"
#include "core/sd_csv_log.hpp"

namespace {

constexpr uint32_t kGpsBootBaud = 9600;
constexpr float    kRadToDeg    = 57.2957795130823f;

// Latest sensor values, mirrored for the SD row (as in the old g_* globals).
float g_ax = 0, g_ay = 0, g_az = 0, g_gx = 0, g_gy = 0, g_gz = 0;
float g_press_hpa = 0, g_temp_c = 0, g_alt_m = 0;

bool     s_bmi_ready   = false;
uint32_t s_att_last_us = 0;
Print*   s_log = nullptr;   // -> usb_stream::log(), set in setup()

inline Print& L() { return *s_log; }

// ---------------------------------------------------------------------------
// Scheduler tasks (one per old loop() block; timing gates are now the
// scheduler's job, so each body assumes it is being called at its rate).
// ---------------------------------------------------------------------------

void task_gps()   // 50 Hz -- drain UART, parse, emit position / status
{
    const uint8_t ev = ublox::poll();

    if (ev & ublox::EV_GGA) {
        static uint32_t last = 0;
        if ((millis() - last) >= 1000) {      // GPS_STAT <= 1 Hz
            last = millis();
            L().print(F("GPS_STAT,"));
            L().print(ublox::fix());      L().print(',');
            L().print(ublox::sats());     L().print(',');
            L().print(ublox::time_str()); L().print(',');
            L().println(ublox::speed_kmh(), 1);
        }
    }
    if (ev & ublox::EV_FIX) {
        L().print(F("GPS,"));
        L().print(ublox::lat_deg(), 6); L().print(',');
        L().print(ublox::lon_deg(), 6); L().print(',');
        L().print(ublox::alt_m(), 1);   L().print(',');
        L().print(ublox::sats());       L().print(',');
        L().print(ublox::fix());        L().print(',');
        L().print(ublox::time_str());   L().print(',');
        L().println(ublox::speed_kmh(), 1);
    }
}

void task_bmi_retry()   // 1 Hz -- only acts while the IMU is not up
{
    if (s_bmi_ready) return;
    if (bmi323::begin()) {
        s_bmi_ready = true;
        L().println(F("BMI_STATUS,1"));
    } else {
        L().println(F("BMI_STATUS,0"));
        L().print(F("BMI_RAW,0x"));
        L().println(bmi323::raw_chip_id(), HEX);
    }
}

void task_bmi()   // 100 Hz -- read IMU, run AHRS, stream BMI line
{
    if (!s_bmi_ready) return;

    bmi323::Sample s;
    const bmi323::Result r = bmi323::read(s);
    if (r == bmi323::Result::comm_error) {
        s_bmi_ready = false;              // task_bmi_retry picks it up
        L().println(F("BMI_STATUS,0"));
        return;
    }
    if (r != bmi323::Result::ok) return;  // no_data -> keep last values

    const uint32_t now_us = micros();
    const float dt_s = (s_att_last_us == 0) ? 0.01f
                                            : (float)(now_us - s_att_last_us) * 1e-6f;
    s_att_last_us = now_us;
    ahrs::update(s.ax_g, s.ay_g, s.az_g, s.gx_dps, s.gy_dps, s.gz_dps, dt_s);

    g_ax = s.ax_g;   g_ay = s.ay_g;   g_az = s.az_g;
    g_gx = s.gx_dps; g_gy = s.gy_dps; g_gz = s.gz_dps;

    L().print(F("BMI,"));
    L().print(s.ax_g, 4);   L().print(',');
    L().print(s.ay_g, 4);   L().print(',');
    L().print(s.az_g, 4);   L().print(',');
    L().print(s.gx_dps, 2); L().print(',');
    L().print(s.gy_dps, 2); L().print(',');
    L().println(s.gz_dps, 2);
}

void task_att_out()   // 20 Hz
{
    if (!s_bmi_ready) return;
    L().print(F("ATT,"));
    L().print(ahrs::roll_rad()  * kRadToDeg, 1); L().print(',');
    L().print(ahrs::pitch_rad() * kRadToDeg, 1); L().print(',');
    L().println(ahrs::yaw_rad() * kRadToDeg, 1);
}

void task_bmp()   // 50 Hz -- driver self-gates the ~10 Hz trigger/collect
{
    bmp581::Sample bs;
    if (!bmp581::poll(bs)) return;

    const float p_pa = bs.pressure_pa;
    const float alt  = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1902632f));
    g_press_hpa = p_pa / 100.0f;
    g_temp_c    = bs.temp_c;
    g_alt_m     = alt;

    L().print(F("BMP,"));
    L().print(p_pa / 100.0f, 3); L().print(',');
    L().print(bs.temp_c, 2);     L().print(',');
    L().println(alt, 2);
}

void task_sd_log()   // 50 Hz
{
    if (!sd_csv_log::ok() || !s_bmi_ready) return;
    const sd_csv_log::Row row {
        millis(),
        g_ax, g_ay, g_az, g_gx, g_gy, g_gz,
        ahrs::roll_rad()  * kRadToDeg,
        ahrs::pitch_rad() * kRadToDeg,
        ahrs::yaw_rad()   * kRadToDeg,
        g_press_hpa, g_temp_c, g_alt_m,
        ublox::time_str(), ublox::sats(), ublox::speed_kmh(),
    };
    sd_csv_log::write_row(row);
}

void task_debug()   // 2 Hz -- the low-rate GPS/SD debug lines keep their gates
{
    const uint32_t now = millis();

    static uint32_t l_dbg = 0;
    if ((now - l_dbg) >= 2000) {
        l_dbg = now;
        L().print(F("GPS_DBG,"));
        L().print(ublox::rx_bytes());     L().print(',');
        L().print(ublox::current_baud()); L().print(',');
        L().println(ublox::locked() ? 1 : 0);
    }

    static uint32_t l_first = 0;
    if ((now - l_first) >= 3000) {
        l_first = now;
        const uint8_t* boot = nullptr;
        const uint16_t boot_len = ublox::boot_capture(boot);
        L().print(F("GPS_FIRST,"));
        L().print(boot_len);
        L().print(',');
        for (uint16_t i = 0; i < boot_len && i < 64; i++) {
            if (boot[i] < 0x10) L().print('0');
            L().print(boot[i], HEX);
        }
        L().println();
    }

    static uint32_t l_raw = 0;
    if (ublox::nmea_valid() && (now - l_raw) >= 1000) {
        l_raw = now;
        L().print(F("GPS_RAW,"));
        L().println(ublox::last_nmea());
    }

    static uint32_t l_sd = 0;
    if ((now - l_sd) >= 5000) {
        l_sd = now;
        L().print(F("SD_DBG,"));
        L().print(sd_csv_log::ok() ? 1 : 0); L().print(F(","));
        L().print(sd_csv_log::name());       L().print(F(","));
        L().print(usb_stream::drops());
        L().println();
    }
}

void task_sched_report()   // 1 Hz -- per-task DWT timing + overruns
{
    sched::TaskStats st;
    for (size_t i = 0; i < sched::task_count(); i++) {
        if (!sched::get_stats(i, st)) continue;
        L().print(F("SCHED,"));      L().print(st.name);
        L().print(F(",hz="));        L().print(st.rate_hz);
        L().print(F(",min_us="));    L().print(st.min_us);
        L().print(F(",mean_us="));   L().print(st.mean_us);
        L().print(F(",max_us="));    L().print(st.max_us);
        L().print(F(",overruns=")); L().print(st.overruns);
        L().println();
    }
    L().print(F("SCHED,_loop,passes="));   L().print(sched::loop_count());
    L().print(F(",worst_pass_us="));       L().println(sched::worst_pass_us());
}

} // namespace

void setup()
{
    hal::init();

    usb_stream::begin();
    ublox::begin(kGpsBootBaud);

    // Bounded wait for the USB host, draining the u-blox boot burst meanwhile.
    for (int i = 0; i < 300 && !usb_stream::host_ready(); i++) {
        ublox::drain_rx();
        hal::delay_ms(10);
    }
    hal::delay_ms(100);
    ublox::drain_rx();

    s_log = &usb_stream::log();
    L().println(F("boot: BMP581 + BMI323 + uBlox + SD streamer (scheduled)"));

    const bool bmp_ok = bmp581::begin();     // init once, not retried
    L().println(bmp_ok ? F("BMP_STATUS,1") : F("BMP_STATUS,0"));
    ublox::drain_rx();

    switch (sd_csv_log::begin()) {
        case sd_csv_log::Result::ok:
            L().print(F("SD_STATUS,1,")); L().println(sd_csv_log::name()); break;
        case sd_csv_log::Result::open_failed:
            L().println(F("SD_STATUS,0,open_failed")); break;
        case sd_csv_log::Result::begin_failed:
            L().println(F("SD_STATUS,0,begin_failed")); break;
    }

    // Register order matters: producers before consumers within a pass
    // (bmi writes g_*/ahrs, then att_out and sd_log read them).
    sched::add("gps",       50, task_gps);
    sched::add("bmi_retry",  1, task_bmi_retry);
    sched::add("bmi",      100, task_bmi);
    sched::add("att",       20, task_att_out);
    sched::add("bmp",       50, task_bmp);
    sched::add("sd_log",    50, task_sd_log);
    sched::add("debug",      2, task_debug);
    sched::add("sched",      1, task_sched_report);

    // IWDG stays off until MANUAL exists to fall back into on a watchdog reset.
    // sched::set_watchdog_ms(1000);

    sched::run();   // never returns
}

void loop() { }   // unused -- sched::run() owns the main loop
