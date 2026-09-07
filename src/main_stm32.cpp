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
#include <string.h>

#include "hal/hal.hpp"
#include "core/scheduler.hpp"
#include "core/failsafe.hpp"
#include "core/arming.hpp"
#include "drivers/bmi323.hpp"
#include "drivers/bmp581.hpp"
#include "drivers/ublox.hpp"
#include "drivers/crsf.hpp"
#include "estimation/ahrs.hpp"
#include "estimation/imu_prep.hpp"
#include "estimation/baro_alt.hpp"
#include "control/rc_channel.hpp"
#include "control/srv_channel.hpp"
#include "control/mixer.hpp"
#include "control/surface_test.hpp"
#include "modes/mode_manual.hpp"
#include "modes/mode_assist.hpp"
#include "core/usb_stream.hpp"
#include "core/log_ring.hpp"
#include "core/log_frame.hpp"
#include "core/sd_bin_log.hpp"

namespace {

constexpr uint32_t kGpsBootBaud = 9600;
constexpr float    kRadToDeg    = 57.2957795130823f;

// --- CRSF channel assignment (AETR; index 0-based). Set kArmCh to whichever
//     switch you map on the TX16S. ---
constexpr int      kRollCh  = 0, kPitchCh = 1, kThrCh = 2, kYawCh = 3;
constexpr int      kArmCh   = 4;      // ch5: 2-pos arm switch
constexpr int      kModeCh  = 6;      // ch7: 3-pos mode select (low/mid/high)
constexpr uint16_t kArmHi   = 1700;  // arm switch "on" threshold, us
constexpr uint32_t kServoHz = 333;

// ch7 -> requested mode. low = MANUAL, mid = ASSIST. AUTO (high) is not built
// yet -- a high request resolves to MANUAL (see resolve_mode()); the MODE line
// still reports req=AUTO so the mismatch is visible, never silent.
inline modes::Id mode_from_ch(uint16_t us)
{
    if (us < 1333) return modes::Id::manual;
    if (us < 1667) return modes::Id::assist;
    return modes::Id::auto_;
}
constexpr float kAirspeedFloorMps = 10.0f;   // GPS-speed proxy floor for the
                                             // angle loop (no pitot; unreliable
                                             // in wind -- see DECISIONS.md)
inline const char* mode_name(modes::Id id)
{
    return (id == modes::Id::manual) ? "MANUAL"
         : (id == modes::Id::assist) ? "ASSIST" : "AUTO";
}

// Latest sensor values, mirrored for the SD row (as in the old g_* globals).
float g_ax = 0, g_ay = 0, g_az = 0, g_gx = 0, g_gy = 0, g_gz = 0;
float g_press_hpa = 0, g_temp_c = 0, g_alt_m = 0;

bool     s_bmi_ready   = false;
uint32_t s_att_last_us = 0;
Print*   s_log = nullptr;   // -> usb_stream::log(), set in setup()

inline Print& L() { return *s_log; }

// --- control chain ---------------------------------------------------------
control::RcChannel  s_rc_roll, s_rc_pitch, s_rc_yaw;   // symmetric, 1000/1500/2000
control::RcChannel  s_rc_thr;                          // unipolar throttle
control::SrvChannel s_srv[8];                          // per-output us mapping
modes::ModeManual   s_mode_manual;
modes::ModeAssist   s_mode_assist;
modes::Mode*        s_mode_active = &s_mode_manual;    // current mode object
modes::Id           s_mode_cur    = modes::Id::manual; // its id, for the log frame
bool               s_assist_lockout = false;          // latched: IMU faulted in ASSIST
bool               s_flying         = false;          // latched: armed + spooled/rolling
control::SurfaceTest s_surface_test;                   // gyro-cal-done surface sweep
core::Failsafe      s_failsafe;
core::Arming        s_arming;
control::Outputs    s_out;                             // last outputs (bumpless)
uint16_t            s_out_us[8] = { 0,0,0,0,0,0,0,0 }; // last pulses, for OUT,
modes::Id           s_mode_req  = modes::Id::manual;   // requested via ch7
core::LogRing       s_log_ring;                        // binary log producers -> SD
estimation::ImuPrep s_imu_prep;                        // gyro-bias cal + LPF
estimation::BaroAlt s_baro_alt;                        // ground-referenced altitude
float               g_alt_agl_m = 0.0f;

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

void task_crsf()   // 100 Hz -- drain USART3, parse CRSF, stream channels / link
{
    const uint8_t ev = crsf::poll();

    static uint32_t last_rc = 0;
    if ((ev & crsf::EV_RC) && (millis() - last_rc) >= 50) {   // 20 Hz to USB
        last_rc = millis();
        const crsf::Channels& ch = crsf::channels();
        L().print(F("RC"));
        for (int i = 0; i < 16; i++) { L().print(','); L().print(ch.us[i]); }
        L().println();
    }

    static uint32_t last_link = 0;
    if ((ev & crsf::EV_LINK) && (millis() - last_link) >= 200) {   // 5 Hz
        last_link = millis();
        const crsf::LinkStats& lk = crsf::link();
        L().print(F("LINK,up_rssi_dbm=")); L().print(lk.up_rssi_dbm);
        L().print(F(",up_lq="));           L().print(lk.up_lq);
        L().print(F(",up_snr="));          L().print(lk.up_snr);
        L().print(F(",rf_mode="));         L().println(lk.rf_mode);
    }
}

// Which mode may actually run this tick. Demotion is downward only and, for an
// IMU fault, latched (CLAUDE.md "Flight modes"): once ASSIST has lost the IMU it
// stays locked out until reboot. ASSIST also requires the gyro-bias cal to have
// completed (unbiased rates) -- that gate is not latched, it just waits.
modes::Id resolve_mode(modes::Id req, bool imu_ok, bool bias_ready, bool failsafe)
{
    if (failsafe) return modes::Id::manual;              // fall back, don't latch
    if (!imu_ok)  { s_assist_lockout = true; return modes::Id::manual; }
    if (req == modes::Id::assist && !s_assist_lockout && bias_ready)
        return modes::Id::assist;
    return modes::Id::manual;                            // AUTO request lands here too
}

// Switch the active mode, preloading it from the last outputs so the first
// command equals the current servo position (bumpless -- a snap at speed loses
// the airframe).
void set_mode(modes::Id id)
{
    if (id == s_mode_cur) return;
    modes::Mode* next = (id == modes::Id::assist)
                      ? static_cast<modes::Mode*>(&s_mode_assist)
                      : static_cast<modes::Mode*>(&s_mode_manual);
    next->enter(s_out);
    s_mode_active = next;
    s_mode_cur    = id;
    L().print(F("MODE_CHANGE,")); L().println(next->name());
}

void task_control()   // 400 Hz -- CRSF -> arming/failsafe -> mode -> mixer -> PWM
{
    static uint32_t last_us = 0;
    const uint32_t now_us = micros();
    const float dt_s = (last_us == 0) ? 0.0025f : (float)(now_us - last_us) * 1e-6f;
    last_us = now_us;

    const crsf::Channels& ch = crsf::channels();

    control::Sticks sticks;
    sticks.roll     = s_rc_roll.norm(ch.us[kRollCh]);
    sticks.pitch    = s_rc_pitch.norm(ch.us[kPitchCh]);
    sticks.yaw      = s_rc_yaw.norm(ch.us[kYawCh]);
    sticks.throttle = s_rc_thr.unipolar(ch.us[kThrCh]);

    s_failsafe.update(crsf::receiving(), millis());
    s_mode_req = mode_from_ch(ch.us[kModeCh]);

    core::ArmInputs ai;
    ai.arm_switch      = ch.us[kArmCh] > kArmHi;
    ai.throttle        = sticks.throttle;
    ai.failsafe_active = s_failsafe.active();
    s_arming.update(ai);

    const bool armed = s_arming.armed();

    // "flying" latch: set once armed AND (throttle clearly applied OR moving),
    // held until disarm. Gates the stabilizer integrators so they never wind
    // against a stationary airframe (armed on the bench = frozen; the moment
    // you spool up or roll, they arm and re-preset bumplessly).
    if (!armed) {
        s_flying = false;
    } else if (sticks.throttle > 0.25f || (ublox::speed_kmh() * (1.0f / 3.6f)) > 3.0f) {
        s_flying = true;
    }

    // --- mode manager: resolve request -> permitted mode, switch bumplessly ---
    set_mode(resolve_mode(s_mode_req, s_bmi_ready, s_imu_prep.bias_ready(),
                          s_failsafe.active()));

    modes::ModeInput mi;
    mi.sticks       = sticks;
    mi.dt_s         = dt_s;
    mi.roll_rad     = ahrs::roll_rad();
    mi.pitch_rad    = ahrs::pitch_rad();
    mi.yaw_rad      = ahrs::yaw_rad();
    mi.gyro_p_dps   = g_gx;   // bias-corrected + LPF'd body rates from task_bmi
    mi.gyro_q_dps   = g_gy;
    mi.gyro_r_dps   = g_gz;
    mi.airspeed_mps = fmaxf(ublox::speed_kmh() / 3.6f, kAirspeedFloorMps);
    mi.allow_integrators = s_flying;
    s_mode_active->update(mi, s_out);

    // "gyro-bias cal complete" signal: on the rising edge of bias_ready, while
    // disarmed, sweep aileron -> elevator -> rudder once so the pilot sees it.
    static bool prev_bias_ready = false;
    const bool bias_ready = s_imu_prep.bias_ready();
    if (bias_ready && !prev_bias_ready) {
        L().println(F("CAL_DONE,gyro_bias"));            // one-shot marker
        if (!armed) s_surface_test.start();
    }
    prev_bias_ready = bias_ready;
    if (armed) s_surface_test.cancel();

    float sw_r, sw_p, sw_y;
    const bool sweeping = s_surface_test.step(dt_s, sw_r, sw_p, sw_y);

    const bool fs = s_failsafe.active();
    for (int i = 0; i < 8; i++) {
        const bool is_thr = (i == 6 || i == 7);
        uint16_t us;
        if (sweeping && !is_thr) {                      // surface sweep overrides (disarmed)
            const float d = (i <= 1) ? sw_r : (i <= 3) ? sw_p : sw_y;
            us = s_srv[i].from_norm(d);
        }
        else if (fs)                us = s_srv[i].safe_us(is_thr);
        else if (is_thr && !armed)  us = s_srv[i].safe_us(true);          // motors off
        else if (is_thr)            us = s_srv[i].from_unipolar(s_out.ch[i]);
        else                        us = s_srv[i].from_norm(s_out.ch[i]);
        s_out_us[i] = us;
        if (i < 4) hal::pwm_write_us(hal::PwmGroup::out_1_4, (uint8_t)i,       us);
        else       hal::pwm_write_us(hal::PwmGroup::out_5_8, (uint8_t)(i - 4), us);
    }

    static uint32_t last_out = 0;
    if ((millis() - last_out) >= 50) {   // 20 Hz to USB
        last_out = millis();
        L().print(F("OUT"));
        for (int i = 0; i < 8; i++) { L().print(','); L().print(s_out_us[i]); }
        L().println();
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

    // bias-correct + low-pass, then run the gated AHRS
    const estimation::ImuSample p = s_imu_prep.process(
        s.gx_dps, s.gy_dps, s.gz_dps, s.ax_g, s.ay_g, s.az_g, dt_s);
    ahrs::update(p.ax_g, p.ay_g, p.az_g, p.gx_dps, p.gy_dps, p.gz_dps, dt_s);

    g_ax = p.ax_g;   g_ay = p.ay_g;   g_az = p.az_g;
    g_gx = p.gx_dps; g_gy = p.gy_dps; g_gz = p.gz_dps;

    L().print(F("BMI,"));
    L().print(p.ax_g, 4);   L().print(',');
    L().print(p.ay_g, 4);   L().print(',');
    L().print(p.az_g, 4);   L().print(',');
    L().print(p.gx_dps, 2); L().print(',');
    L().print(p.gy_dps, 2); L().print(',');
    L().println(p.gz_dps, 2);
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
    const float alt  = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1902632f));   // absolute (ISA)
    g_press_hpa = p_pa / 100.0f;
    g_temp_c    = bs.temp_c;
    g_alt_m     = alt;

    // ground reference: track the latest pressure while disarmed, freeze on arm.
    if (!s_arming.armed()) s_baro_alt.set_ground(p_pa);
    g_alt_agl_m = s_baro_alt.alt_m(p_pa);

    L().print(F("BMP,"));
    L().print(p_pa / 100.0f, 3); L().print(',');
    L().print(bs.temp_c, 2);     L().print(',');
    L().println(alt, 2);
}

void task_log()   // 50 Hz -- pack one binary frame into the ring (non-blocking)
{
    core::LogFrame f;
    memset(&f, 0, sizeof(f));
    f.t_ms = millis();

    f.acc[0] = (int16_t)(g_ax * core::kLogAccScale);
    f.acc[1] = (int16_t)(g_ay * core::kLogAccScale);
    f.acc[2] = (int16_t)(g_az * core::kLogAccScale);
    f.gyr[0] = (int16_t)(g_gx * core::kLogGyrScale);
    f.gyr[1] = (int16_t)(g_gy * core::kLogGyrScale);
    f.gyr[2] = (int16_t)(g_gz * core::kLogGyrScale);
    f.att[0] = (int16_t)(ahrs::roll_rad()  * kRadToDeg * core::kLogAngScale);
    f.att[1] = (int16_t)(ahrs::pitch_rad() * kRadToDeg * core::kLogAngScale);
    f.att[2] = (int16_t)(ahrs::yaw_rad()   * kRadToDeg * core::kLogAngScale);

    f.press_pa      = g_press_hpa * 100.0f;
    f.temp_dC       = (int16_t)(g_temp_c * 10.0f);
    f.alt_mm        = (int32_t)(g_alt_agl_m * 1000.0f);   // AGL vs ground ref
    f.lat_1e7       = (int32_t)(ublox::lat_deg() * 1e7);
    f.lon_1e7       = (int32_t)(ublox::lon_deg() * 1e7);
    f.gps_alt_mm    = (int32_t)(ublox::alt_m() * 1000.0f);
    f.gps_speed_cms = (uint16_t)(ublox::speed_kmh() * (100.0f / 3.6f));
    f.gps_sats      = (uint8_t)ublox::sats();
    f.gps_fix       = (uint8_t)ublox::fix();

    const crsf::Channels& ch = crsf::channels();
    for (int i = 0; i < 8; i++) { f.rc_us[i] = ch.us[i]; f.out_us[i] = s_out_us[i]; }

    f.mode  = (uint8_t)s_mode_cur;
    f.flags = (uint8_t)((s_arming.armed()      ? core::LOG_ARMED    : 0) |
                        (s_failsafe.active()   ? core::LOG_FAILSAFE : 0) |
                        ((uint8_t)s_mode_req << core::LOG_REQMODE_SHIFT));

    core::log_frame_finalize(f);
    s_log_ring.push(&f, sizeof(f));   // drops + counts if the ring is full
}

void task_log_flush()   // 25 Hz -- drain 512-byte sectors to SD (blocking lives here)
{
    sd_bin_log::flush_step();
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
        sd_bin_log::sync();                          // commit FAT/dir, ~5 s cadence
        L().print(F("SD_DBG,"));
        L().print(sd_bin_log::ok() ? 1 : 0);         L().print(F(","));
        L().print(sd_bin_log::name());               L().print(F(",bytes="));
        L().print(sd_bin_log::bytes_written());      L().print(F(",log_drops="));
        L().print(s_log_ring.drops());               L().print(F(",usb_drops="));
        L().print(usb_stream::drops());
        L().println();
    }

    static uint32_t l_est = 0;
    if ((now - l_est) >= 1000) {
        l_est = now;
        float bx, by, bz; ahrs::gyro_bias_dps(bx, by, bz);
        L().print(F("EST,bias_ready="));   L().print(s_imu_prep.bias_ready() ? 1 : 0);
        L().print(F(",gbias="));           L().print(bx, 2); L().print('/');
        L().print(by, 2); L().print('/');  L().print(bz, 2);
        L().print(F(",acc_trust="));       L().print(ahrs::acc_trust(), 2);
        L().print(F(",agl_m="));           L().print(g_alt_agl_m, 2);
        L().print(F(",baro_ref="));        L().print(s_baro_alt.referenced() ? 1 : 0);
        L().println();
    }

    static uint32_t l_crsf = 0;
    if ((now - l_crsf) >= 1000) {
        l_crsf = now;
        L().print(F("CRSF_STAT,receiving=")); L().print(crsf::receiving() ? 1 : 0);
        L().print(F(",frames_ok="));          L().print(crsf::frames_ok());
        L().print(F(",crc_err="));            L().print(crsf::crc_errors());
        L().print(F(",resync="));             L().print(crsf::resyncs());
        L().println();
    }

    static uint32_t l_mode = 0;
    if ((now - l_mode) >= 500) {
        l_mode = now;
        L().print(F("MODE,active="));  L().print(s_mode_active->name());
        L().print(F(",req="));         L().print(mode_name(s_mode_req));
        L().print(F(",armed="));       L().print(s_arming.armed() ? 1 : 0);
        L().print(F(",failsafe="));    L().print((int)s_failsafe.level());
        L().print(F(",assist_lockout=")); L().print(s_assist_lockout ? 1 : 0);
        L().print(F(",flying="));          L().print(s_flying ? 1 : 0);
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
    crsf::begin(420000);   // ER8 on USART3 (PB11 rx / PB10 tx), 420000 8N1

    // Bounded wait for the USB host, draining the u-blox boot burst meanwhile.
    for (int i = 0; i < 300 && !usb_stream::host_ready(); i++) {
        ublox::drain_rx();
        hal::delay_ms(10);
    }
    hal::delay_ms(100);
    ublox::drain_rx();

    s_log = &usb_stream::log();
    L().println(F("boot: BMP581 + BMI323 + uBlox + CRSF + SD (scheduled) -- MANUAL"));

    // IMU preconditioning: 100 Hz sample, gentle LPF (Nyquist is 50 Hz until
    // the FIFO/1 kHz path lands). Gyro bias calibrates on the first stationary
    // window; keep the board still for the first few seconds after boot.
    s_imu_prep.configure(100.0f, 30.0f, 15.0f);
    ahrs::reset();

    // ASSIST (FBWA): stick -> clamped attitude angle -> rate loop -> mixer.
    // Rough first gains, verified only in SITL (--assist-check) -- FF-dominant
    // per CLAUDE.md, small P, light I. Tune in flight. See docs/DECISIONS.md.
    {
        control::AttitudeCtrlConfig att;                 // defaults (110 dps/rad, turn-comp on)
        control::RateCtrlConfig rate;
        rate.sample_hz = 400.0f;                         // == task_control rate
        rate.roll.kff = 0.006f; rate.roll.kp = 0.010f; rate.roll.ki = 0.02f;
        rate.roll.i_max = 0.4f; rate.roll.d_lpf_hz = 25.0f;
        rate.pitch = rate.roll; rate.pitch.kff = 0.010f; rate.pitch.kp = 0.020f;
        rate.yaw   = rate.roll; rate.yaw.kff = 0.004f; rate.yaw.kp = 0.006f; rate.yaw.ki = 0.0f;
        s_mode_assist.configure(att, rate, 0.70f, 0.45f, 80.0f);  // max roll/pitch rad, max yaw-rate dps
    }

    // Servo / ESC PWM. SrvChannel defaults (1000/1500/2000) suit surfaces and,
    // via from_unipolar(), the ESCs (min 1000 = off). Per-airframe trim /
    // reverse is set later.
    hal::pwm_config(hal::PwmGroup::out_1_4, kServoHz);
    hal::pwm_config(hal::PwmGroup::out_5_8, kServoHz);

    const bool bmp_ok = bmp581::begin();     // init once, not retried
    L().println(bmp_ok ? F("BMP_STATUS,1") : F("BMP_STATUS,0"));
    ublox::drain_rx();

    switch (sd_bin_log::begin(s_log_ring)) {
        case sd_bin_log::Result::ok:
            L().print(F("SD_STATUS,1,")); L().println(sd_bin_log::name()); break;
        case sd_bin_log::Result::open_failed:
            L().println(F("SD_STATUS,0,open_failed")); break;
        case sd_bin_log::Result::begin_failed:
            L().println(F("SD_STATUS,0,begin_failed")); break;
    }

    // Register order matters: producers before consumers within a pass
    // (bmi writes g_*/ahrs, then att_out and sd_log read them).
    sched::add("gps",       50, task_gps);
    sched::add("crsf",     100, task_crsf);
    sched::add("bmi_retry",  1, task_bmi_retry);
    sched::add("bmi",      100, task_bmi);
    sched::add("control",  400, task_control);
    sched::add("att",       20, task_att_out);
    sched::add("bmp",       50, task_bmp);
    sched::add("log",       50, task_log);        // pack frame -> ring (fast)
    sched::add("log_flush", 25, task_log_flush);  // ring -> SD (blocking, isolated)
    sched::add("debug",      2, task_debug);
    sched::add("sched",      1, task_sched_report);

    // IWDG stays off until MANUAL exists to fall back into on a watchdog reset.
    // sched::set_watchdog_ms(1000);

    sched::run();   // never returns
}

void loop() { }   // unused -- sched::run() owns the main loop
