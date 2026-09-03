//
// main_stm32.cpp -- STM32F407 application entry (Arduino setup/loop).
//
// Composition root: wires hal + drivers + estimation + the bench USB/SD
// scaffolding together and runs the temporary super-loop. This is the only
// place scheduling lives until the cooperative scheduler is built.
//
// Arduino-coupled (setup/loop symbols, F() strings). Excluded from [env:native].
//
// BEHAVIOUR CONTRACT: this file is a pure refactor of the old single-file
// streamer. Same tagged-CSV output, same cadences, same SD columns. If you
// change what it emits, that is a feature change -- do it deliberately.
//
#include <Arduino.h>
#include <math.h>

#include "hal/hal.hpp"
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

bool s_bmi_ready = false;

Print* s_log = nullptr;   // -> usb_stream::log(), set in setup()

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
    Print& Log = *s_log;
    Log.println(F("boot: BMP581 + BMI323 + uBlox + SD streamer"));

    // BMP581 is initialised once here, not retried from loop().
    const bool bmp_ok = bmp581::begin();
    Log.println(bmp_ok ? F("BMP_STATUS,1") : F("BMP_STATUS,0"));
    ublox::drain_rx();

    switch (sd_csv_log::begin()) {
        case sd_csv_log::Result::ok:
            Log.print(F("SD_STATUS,1,"));
            Log.println(sd_csv_log::name());
            break;
        case sd_csv_log::Result::open_failed:
            Log.println(F("SD_STATUS,0,open_failed"));
            break;
        case sd_csv_log::Result::begin_failed:
            Log.println(F("SD_STATUS,0,begin_failed"));
            break;
    }
}

void loop()
{
    Print& Log = *s_log;

    // --- u-blox GNSS: drain + parse, auto-baud, UBX poll ------------------
    const uint8_t gps_ev = ublox::poll();

    // GPS_STAT: fix quality + sat count, <=1 Hz, even with no position.
    if (gps_ev & ublox::EV_GGA) {
        static uint32_t last_gps_stat = 0;
        if ((millis() - last_gps_stat) >= 1000) {
            last_gps_stat = millis();
            Log.print(F("GPS_STAT,"));
            Log.print(ublox::fix());       Log.print(',');
            Log.print(ublox::sats());      Log.print(',');
            Log.print(ublox::time_str()); Log.print(',');
            Log.println(ublox::speed_kmh(), 1);
        }
    }

    // GPS: full position line, on every fix.
    if (gps_ev & ublox::EV_FIX) {
        Log.print(F("GPS,"));
        Log.print(ublox::lat_deg(), 6);  Log.print(',');
        Log.print(ublox::lon_deg(), 6);  Log.print(',');
        Log.print(ublox::alt_m(), 1);    Log.print(',');
        Log.print(ublox::sats());        Log.print(',');
        Log.print(ublox::fix());         Log.print(',');
        Log.print(ublox::time_str());    Log.print(',');
        Log.println(ublox::speed_kmh(), 1);
    }

    // Debug: rx byte count / baud / lock state every 2 s.
    static uint32_t last_gps_dbg = 0;
    if ((millis() - last_gps_dbg) >= 2000) {
        last_gps_dbg = millis();
        Log.print(F("GPS_DBG,"));
        Log.print(ublox::rx_bytes());      Log.print(',');
        Log.print(ublox::current_baud());  Log.print(',');
        Log.println(ublox::locked() ? 1 : 0);
    }

    // Captured u-blox boot bytes (hex) every 3 s.
    static uint32_t last_gps_first = 0;
    if ((millis() - last_gps_first) >= 3000) {
        last_gps_first = millis();
        const uint8_t* boot = nullptr;
        const uint16_t boot_len = ublox::boot_capture(boot);
        Log.print(F("GPS_FIRST,"));
        Log.print(boot_len);
        Log.print(',');
        for (uint16_t i = 0; i < boot_len && i < 64; i++) {
            if (boot[i] < 0x10) Log.print('0');
            Log.print(boot[i], HEX);
        }
        Log.println();
    }

    // Echo the most recent valid NMEA sentence (1 Hz).
    static uint32_t last_nmea = 0;
    if (ublox::nmea_valid() && (millis() - last_nmea) >= 1000) {
        last_nmea = millis();
        Log.print(F("GPS_RAW,"));
        Log.println(ublox::last_nmea());
    }

    // --- BMI323 presence detection (retry every 1 s until found) ---------
    static uint32_t last_bmi_retry = 0;
    if (!s_bmi_ready && (millis() - last_bmi_retry) >= 1000) {
        last_bmi_retry = millis();
        if (bmi323::begin()) {
            s_bmi_ready = true;
            Log.println(F("BMI_STATUS,1"));
        } else {
            Log.println(F("BMI_STATUS,0"));
            Log.print(F("BMI_RAW,0x"));
            Log.println(bmi323::raw_chip_id(), HEX);
        }
    }

    // --- BMI323 streaming at ~100 Hz -----------------------------------
    static uint32_t last_bmi = 0;
    static uint32_t att_last_us = 0;
    if (s_bmi_ready && (millis() - last_bmi) >= 10) {
        last_bmi = millis();

        bmi323::Sample s;
        const bmi323::Result r = bmi323::read(s);
        if (r == bmi323::Result::comm_error) {
            s_bmi_ready = false;   // re-initialise next pass
            Log.println(F("BMI_STATUS,0"));
        } else if (r == bmi323::Result::ok) {
            const uint32_t now_us = micros();
            const float dt_s = (att_last_us == 0) ? 0.01f
                                                  : (float)(now_us - att_last_us) * 1e-6f;
            att_last_us = now_us;
            ahrs::update(s.ax_g, s.ay_g, s.az_g, s.gx_dps, s.gy_dps, s.gz_dps, dt_s);

            g_ax = s.ax_g; g_ay = s.ay_g; g_az = s.az_g;
            g_gx = s.gx_dps; g_gy = s.gy_dps; g_gz = s.gz_dps;

            Log.print(F("BMI,"));
            Log.print(s.ax_g, 4);   Log.print(',');
            Log.print(s.ay_g, 4);   Log.print(',');
            Log.print(s.az_g, 4);   Log.print(',');
            Log.print(s.gx_dps, 2); Log.print(',');
            Log.print(s.gy_dps, 2); Log.print(',');
            Log.println(s.gz_dps, 2);
        }
        // Result::no_data -> nothing fresh this tick, keep last values
    }

    // --- Attitude output at ~20 Hz -----------------------------------
    static uint32_t last_att_out = 0;
    if (s_bmi_ready && (millis() - last_att_out) >= 50) {
        last_att_out = millis();
        Log.print(F("ATT,"));
        Log.print(ahrs::roll_rad()  * kRadToDeg, 1); Log.print(',');
        Log.print(ahrs::pitch_rad() * kRadToDeg, 1); Log.print(',');
        Log.println(ahrs::yaw_rad() * kRadToDeg, 1);
    }

    // --- BMP581 forced mode, non-blocking ----------------------------
    bmp581::Sample bs;
    if (bmp581::poll(bs)) {
        const float p_pa = bs.pressure_pa;
        const float alt  = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1902632f));

        g_press_hpa = p_pa / 100.0f;
        g_temp_c    = bs.temp_c;
        g_alt_m     = alt;

        Log.print(F("BMP,"));
        Log.print(p_pa / 100.0f, 3); Log.print(',');
        Log.print(bs.temp_c, 2);     Log.print(',');
        Log.println(alt, 2);
    }

    // --- SD data log at ~50 Hz -------------------------------------
    static uint32_t last_sd_log = 0;
    if (sd_csv_log::ok() && s_bmi_ready && (millis() - last_sd_log) >= 20) {
        last_sd_log = millis();
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

    // --- SD status debug every 5 s -------------------------------
    static uint32_t last_sd_dbg = 0;
    if ((millis() - last_sd_dbg) >= 5000) {
        last_sd_dbg = millis();
        Log.print(F("SD_DBG,"));
        Log.print(sd_csv_log::ok() ? 1 : 0);   Log.print(F(","));
        Log.print(sd_csv_log::name());         Log.print(F(","));
        Log.print(usb_stream::drops());
        Log.println();
    }
}
