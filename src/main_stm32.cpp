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
#include "core/flight_state.hpp"
#include "core/arming.hpp"
#include "drivers/imu_v2.hpp"
#include "drivers/bmm350.hpp"
#include "config/airframe.hpp"
#include "drivers/bmp581.hpp"
#include "drivers/ublox.hpp"
#include "drivers/crsf.hpp"
#include "estimation/ahrs.hpp"
#include "estimation/imu_prep.hpp"
#include "estimation/baro_alt.hpp"
#include "control/rc_channel.hpp"
#include "control/srv_channel.hpp"
#include "control/mixer.hpp"
#include "modes/mode_manual.hpp"
#include "modes/mode_assist.hpp"
#include "modes/mode_takeoff.hpp"
#include "core/usb_stream.hpp"
#include "core/log_ring.hpp"
#include "core/log_frame.hpp"
#include "core/diagnostic_log.hpp"
#include "core/sd_bin_log.hpp"

namespace {

constexpr uint32_t kGpsBootBaud = 38400;
constexpr float    kRadToDeg    = 57.2957795130823f;

// --- CRSF channel assignment (AETR; index 0-based). Set kArmCh to whichever
//     switch you map on the TX16S. ---
constexpr int      kRollCh  = 0, kPitchCh = 1, kThrCh = 2, kYawCh = 3;
constexpr int      kArmCh   = 4;      // ch5: 2-pos arm switch
constexpr int      kModeCh  = 6;      // ch7: 3-pos mode select (low/mid/high)
constexpr uint16_t kArmHi   = 1700;  // arm switch "on" threshold, us


// ch7 -> requested mode. low = MANUAL, mid = ASSIST, high = TKOFF (roll
// wing-leveller, pitch/yaw manual -- fly the takeoff here, then drop to ASSIST
// once settled in the climb). AUTO is not built yet and has no switch slot.
inline modes::Id mode_from_ch(uint16_t us)
{
    if (us < 1333) return modes::Id::manual;
    if (us < 1667) return modes::Id::assist;
    return modes::Id::takeoff;
}

// Integration eligibility is maintained independently from the flight-session latch.
inline const char* mode_name(modes::Id id)
{
    switch (id) {
        case modes::Id::assist:  return "ASSIST";
        case modes::Id::takeoff: return "TKOFF";
        case modes::Id::auto_:   return "AUTO";
        default:                 return "MANUAL";
    }
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
modes::ModeTakeoff  s_mode_takeoff;                    // roll wing-leveller for the takeoff
modes::Mode*        s_mode_active = &s_mode_manual;    // current mode object
modes::Id           s_mode_cur    = modes::Id::manual; // its id, for the log frame
bool               s_assist_lockout = false;          // latched: IMU faulted in ASSIST
bool               s_flying         = false;          // current integration eligibility
bool               s_flight_session = false;
bool               s_control_fault = false;
bool               s_output_ready = false;
core::Failsafe      s_failsafe;
core::Arming        s_arming;
core::FlightState   s_flight_state;
core::TelemetrySchedule s_telemetry;
control::Outputs    s_out;                             // last outputs (bumpless)
uint16_t            s_out_us[8] = { 0,0,0,0,0,0,0,0 }; // last pulses, for OUT,
modes::Id           s_mode_req  = modes::Id::manual;   // requested via ch7
core::LogRing       s_log_ring;                        // binary log producers -> SD
estimation::BaroAlt s_baro_alt;                        // ground-referenced altitude
estimation::MagHeading s_heading{config::mag_heading_config()};

bool magnetic_driver_healthy() {
    return config::enable_magnetometer&&mag350::healthy()&&mag350::diagnostics().stage==0;
}
bool heading_attitude_valid() { return imu_v2::healthy()&&imu_v2::bias_ready(); }
float heading_degrees(float radians) {
    float deg=fmodf(radians*kRadToDeg,360.0f);
    return deg<0?deg+360.0f:deg;
}
float               g_alt_agl_m  = 0.0f;
float               g_climb_mps  = 0.0f;               // filtered dAGL/dt, for the CRSF vario

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

void task_crsf()   // 400 Hz -- drain UART4, parse CRSF (RC / link stats)
{
    crsf::poll();   // USB echo of RC / LINK is in task_stream / task_debug
}

// Which mode may actually run this tick. Demotion is downward only and, for an
// IMU fault, latched: once a stabilised mode has lost
// the IMU it stays locked out until reboot. The stabilised modes (ASSIST,
// TKOFF) also require the gyro-bias cal to have completed (unbiased rates) --
// that gate is not latched, it just waits.
modes::Id resolve_mode(modes::Id req, bool imu_ok, bool bias_ready, bool failsafe)
{
    if (!imu_ok && (s_mode_cur==modes::Id::assist || s_mode_cur==modes::Id::takeoff))
        s_assist_lockout=true;
    const bool stab_ok=imu_ok && !s_assist_lockout && bias_ready;
    if (failsafe) return stab_ok ? modes::Id::assist : modes::Id::manual;
    if (req==modes::Id::assist && stab_ok) return modes::Id::assist;
    if (req==modes::Id::takeoff && stab_ok) return modes::Id::takeoff;
    return modes::Id::manual;
}

// Start stabilized surface slew from the last outputs; MANUAL is immediate.
void set_mode(modes::Id id)
{
    if (id == s_mode_cur) return;
    modes::Mode* next;
    switch (id) {
        case modes::Id::assist:  next = &s_mode_assist;  break;
        case modes::Id::takeoff: next = &s_mode_takeoff; break;
        default:                 next = &s_mode_manual;  break;
    }
    next->enter(s_out);
    s_mode_active = next;
    s_mode_cur    = id;
    L().print(F("MODE_CHANGE,")); L().println(next->name());
}

// Flight-mode text for CRSF. Markers are our labels, not guaranteed alarms.
// Configure radio alarms separately; telemetry can be stale during RF loss.
const char* fm_string()
{
    if (s_control_fault) return "!TIMING";
    if (s_failsafe.active()) return imu_v2::healthy() && imu_v2::bias_ready() && !s_assist_lockout ? "!FS-LVL" : "!FS-CTR";
    if (s_assist_lockout && s_mode_cur != modes::Id::assist
                         && s_mode_cur != modes::Id::takeoff) return "!LOCK";
    if (!config::flight_enabled) {
        switch(s_mode_cur) {
            case modes::Id::assist: return "B-ASSIST";
            case modes::Id::takeoff: return "B-TKOFF";
            default: return "B-MANUAL";
        }
    }
    const bool armed = s_arming.armed();
    switch (s_mode_cur) {
        case modes::Id::assist:  return armed ? "ASSIST" : "ASSIST*";
        case modes::Id::takeoff: return armed ? "TKOFF"  : "TKOFF*";
        case modes::Id::auto_:   return armed ? "AUTO"   : "AUTO*";
        default:                 return armed ? "MANUAL" : "MANUAL*";
    }
}

void task_crsf_tx()   // 100 Hz service; per-type due times and byte budget
{
    const uint32_t now=millis(),seq=ublox::fix_sequence();
    const char* mode=fm_string();
    const auto kind=s_telemetry.choose(now,imu_v2::healthy(),bmp581::healthy(),
                                      ublox::locked(),seq,mode,crsf::receiving());
    bool sent=false;
    switch(kind) {
        case core::TelemKind::attitude:
            sent=crsf::send_attitude(ahrs::pitch_rad(),ahrs::roll_rad(),ahrs::yaw_rad()); break;
        case core::TelemKind::vario: sent=crsf::send_vario(g_climb_mps); break;
        case core::TelemKind::gps: {
            crsf::GpsTelem g;
            g.lat_1e7=int32_t(ublox::lat_deg()*1e7); g.lon_1e7=int32_t(ublox::lon_deg()*1e7);
            g.ground_mps=ublox::speed_kmh()/3.6f; g.heading_deg=ublox::course_deg();
            g.altitude_m=ublox::alt_m(); g.sats=uint8_t(ublox::sats());
            sent=crsf::send_gps(g); break;
        }
        case core::TelemKind::mode: sent=crsf::send_flight_mode(mode); break;
        default: return;
    }
    // Only scheduling state is retained. A retry always builds CURRENT values.
    s_telemetry.result(kind,sent,now,seq,mode);
}

void task_control()   // 400 Hz -- CRSF -> arming/failsafe -> mode -> mixer -> PWM
{
    static uint32_t last_us = 0;
    const uint32_t now_us = micros();
    const float dt_s = (last_us == 0) ? 0.0025f : (float)(now_us - last_us) * 1e-6f;
    if (last_us && dt_s > 0.02f && s_flight_session) s_control_fault=true;
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
    ai.permitted = config::flight_enabled && s_output_ready && !s_control_fault;
    s_arming.update(ai);

    const bool armed = s_arming.armed();

    // Debounced integration eligibility; explicit disarm/confirmed landing
    // clear it. Session pressure reference and fault latches are independent.
    if (armed) s_flight_session=true;
    s_flying=s_flight_state.update(armed,s_failsafe.active(),sticks.throttle,
        ublox::speed_valid(),ublox::speed_kmh()/3.6f,bmp581::healthy(),g_climb_mps,
        sqrtf(g_gx*g_gx+g_gy*g_gy+g_gz*g_gz),millis());
    // Session/ground datum stay latched across airborne disarm and RC loss.
    const bool imu_ok=imu_v2::healthy();
    s_bmi_ready=imu_ok;
    if (s_failsafe.active()) sticks={}; // level roll/pitch, zero yaw rate, idle

    // --- mode manager: resolve request -> permitted mode, switch bumplessly ---
    set_mode(resolve_mode(s_mode_req, imu_ok, imu_v2::bias_ready(),
                          s_failsafe.active()));

    modes::ModeInput mi;
    mi.sticks       = sticks;
    mi.dt_s         = fminf(fmaxf(dt_s,0.0005f),0.01f);
    mi.roll_rad     = ahrs::roll_rad();
    mi.pitch_rad    = ahrs::pitch_rad();
    mi.yaw_rad      = ahrs::yaw_rad();
    mi.gyro_p_dps   = g_gx;   // bias-corrected + LPF'd body rates from task_bmi
    mi.gyro_q_dps   = g_gy;
    mi.gyro_r_dps   = g_gz;
    mi.airspeed_mps = 0.0f; mi.airspeed_valid=false; // no pitot fitted
    mi.heading_valid=s_heading.heading_valid(millis(),heading_attitude_valid(),magnetic_driver_healthy());
    mi.allow_heading_hold=armed&&s_flying&&!s_failsafe.active();
    mi.allow_integrators = s_flying;
    s_mode_active->update(mi, s_out);

    // No automatic surface sweep at boot/calibration completion.

    const bool fs = s_failsafe.active();
    for (int i = 0; i < 8; i++) {
        const bool is_thr = control::is_motor_output(i);
        uint16_t us;
        if (is_thr && (fs || !armed)) us=s_srv[i].safe_us(true);
        else if (fs && (!imu_ok || s_assist_lockout || !imu_v2::bias_ready())) us=s_srv[i].safe_us(false);
        else if (is_thr) us=s_srv[i].from_unipolar(s_out.ch[i]);
        else us=s_srv[i].from_norm(s_out.ch[i]);
        s_out_us[i] = us;
        const auto group=i<2?hal::PwmGroup::ailerons:i<6?hal::PwmGroup::tail:hal::PwmGroup::motors;
        const uint8_t channel=i<2?i:i<6?i-2:i-6;
        if(hal::pwm_write_us(group,channel,us)!=hal::Status::ok) s_output_ready=false;
    }
}   // OUT USB line is emitted from task_stream

void task_bmi() // 400 Hz FIFO service, both BMI270s
{
    imu_v2::poll(!s_flight_session);
    const auto& p=imu_v2::latest();
    g_ax=p.ax_g; g_ay=p.ay_g; g_az=p.az_g;
    g_gx=p.gx_dps; g_gy=p.gy_dps; g_gz=p.gz_dps;
    if(imu_v2::healthy()) s_att_last_us=micros();
    if(!heading_attitude_valid()) s_heading.invalidate_attitude();
}
void task_mag()
{
    if(!config::enable_magnetometer) return;
    mag350::Sample m;
    if(mag350::poll(m)) {
        const float correction=s_heading.observe(m.x_ut,m.y_ut,m.z_ut,
            ahrs::roll_rad(),ahrs::pitch_rad(),ahrs::yaw_rad(),heading_attitude_valid(),
            !s_flight_session,millis());
        ahrs::correct_yaw(correction);
        L().print(F("MAG,")); L().print(m.x_ut,2); L().print(',');
        L().print(m.y_ut,2); L().print(','); L().println(m.z_ut,2);
    } else if(!magnetic_driver_healthy()) s_heading.driver_failed();
}

void task_stream()   // 20 Hz -- all the high-rate USB echo, off the control path
{
    if(imu_v2::healthy()) {
        // IMU (bias-corrected, filtered)
        L().print(F("BMI,"));
        L().print(g_ax, 4);   L().print(',');
        L().print(g_ay, 4);   L().print(',');
        L().print(g_az, 4);   L().print(',');
        L().print(g_gx, 2);   L().print(',');
        L().print(g_gy, 2);   L().print(',');
        L().println(g_gz, 2);

        // attitude
        L().print(F("ATT,"));
        L().print(ahrs::roll_rad()  * kRadToDeg, 1); L().print(',');
        L().print(ahrs::pitch_rad() * kRadToDeg, 1); L().print(',');
        L().println(ahrs::yaw_rad() * kRadToDeg, 1);

    } // Suppress normal attitude samples when unhealthy; GUI expires old data.

    // servo / ESC outputs
    L().print(F("OUT"));
    for (int i = 0; i < 8; i++) { L().print(','); L().print(s_out_us[i]); }
    L().println();

    // RC channels (10 Hz -- every other call)
    static bool rc_toggle = false;
    rc_toggle = !rc_toggle;
    if (rc_toggle && crsf::receiving()) {
        const crsf::Channels& ch = crsf::channels();
        L().print(F("RC"));
        for (int i = 0; i < 16; i++) { L().print(','); L().print(ch.us[i]); }
        L().println();
    }
}

void task_bmp()   // 100 Hz poll, consume fresh normal-mode 50 Hz pressure samples
{
    bmp581::Sample bs;
    if (!bmp581::poll(bs)) return;

    const float p_pa = bs.pressure_pa;
    const float alt  = 44330.0f * (1.0f - powf(p_pa / 101325.0f, 0.1902632f));   // absolute (ISA)
    g_press_hpa = p_pa / 100.0f;
    g_temp_c    = bs.temp_c;
    g_alt_m     = alt;

    // ground reference: track the latest pressure while disarmed, freeze on arm.
    if (!s_flight_session) s_baro_alt.set_ground(p_pa);
    g_alt_agl_m = s_baro_alt.alt_m(p_pa);

    // climb rate: filtered derivative of AGL (feeds the CRSF vario)
    static uint32_t s_climb_ms  = 0;
    static float    s_climb_agl = 0.0f;
    const uint32_t now_ms = millis();
    if (s_climb_ms != 0) {
        const float dt = (float)(now_ms - s_climb_ms) * 1e-3f;
        if (dt > 1e-3f) {
            const float raw = (g_alt_agl_m - s_climb_agl) / dt;
            g_climb_mps += (dt / (0.2274f + dt)) * (raw - g_climb_mps);   // ~0.7 Hz LPF at 50 Hz
        }
    }
    s_climb_ms  = now_ms;
    s_climb_agl = g_alt_agl_m;

    L().print(F("BMP,"));
    L().print(p_pa / 100.0f, 3); L().print(',');
    L().print(bs.temp_c, 2);     L().print(',');
    L().println(alt, 2);
}

void imu_log(unsigned i,uint32_t stamp,uint32_t seq,uint32_t clock,const float* raw,const estimation::ImuSample& p)
{
    if(!sd_bin_log::ok()) return;
    core::ImuLogFrame f; f.host_us=stamp; f.sequence=seq; f.sensor_time=clock; f.sensor=uint8_t(i); f.flags=p.bias_ready?1:0;
    const float filtered[6]={p.gx_dps,p.gy_dps,p.gz_dps,p.ax_g,p.ay_g,p.az_g};
    for(unsigned axis=0;axis<6;++axis) {
        const float scale=axis<3?core::kLogGyrScale:core::kLogAccScale;
        f.raw[axis]=core::log_i16(raw[axis],scale); f.filtered[axis]=core::log_i16(filtered[axis],scale);
    }
    core::finish_diagnostic(f); s_log_ring.push(&f,sizeof(f));
}

void task_control_log() // 100 Hz setpoint/output diagnostics
{
    if(!sd_bin_log::ok()) return;
    core::ControlLogFrame f; f.host_us=micros();
    f.demand[0]=core::log_i16(s_mode_cur==modes::Id::assist?s_mode_assist.demand_p():s_mode_cur==modes::Id::takeoff?s_mode_takeoff.demand_p():0,16);
    f.demand[1]=core::log_i16(s_mode_cur==modes::Id::assist?s_mode_assist.demand_q():0,16);
    f.measured[0]=core::log_i16(g_gx,16); f.measured[1]=core::log_i16(g_gy,16); f.measured[2]=core::log_i16(g_gz,16);
    for(unsigned axis=0;axis<3;++axis) f.surface[axis]=core::log_i16(s_out.ch[control::kAxisOutputs[axis]],10000);
    f.throttle=uint16_t(s_out.ch[control::kThrottleOutput]*10000); f.mode=uint8_t(s_mode_cur);
    f.flags=(s_arming.armed()?1:0)|(s_failsafe.active()?2:0)|(s_flying?4:0);
    core::finish_diagnostic(f); s_log_ring.push(&f,sizeof(f));
}

void task_log()   // 50 Hz -- pack one binary frame into the ring (non-blocking)
{
    if(!sd_bin_log::ok()) return;
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

void task_log_flush()   // 4 kHz -- bounded asynchronous SD service
{
    if(config::enable_sd_logging) sd_bin_log::flush_step(512);
}

void task_debug()   // 2 Hz -- low-rate status lines (no blocking calls here)
{
    const uint32_t now = millis();
    L().print(F("TELEM,"));
    for(unsigned i=0;i<4;++i) { L().print(s_telemetry.sent[i]); L().print(','); }
    L().print(s_telemetry.deferred); L().print(','); L().println(s_telemetry.queue_failures);
    const bool bringup = now < 25000;   // GPS bring-up dumps only for the first 25 s

    static uint32_t l_dbg = 0;
    if (bringup && (now - l_dbg) >= 2000) {
        l_dbg = now;
        L().print(F("GPS_DBG,"));
        L().print(ublox::rx_bytes());     L().print(',');
        L().print(ublox::current_baud()); L().print(',');
        L().println(ublox::locked() ? 1 : 0);
    }

    static uint32_t l_first = 0;
    if (bringup && (now - l_first) >= 3000) {
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
    if (bringup && ublox::nmea_valid() && (now - l_raw) >= 1000) {
        l_raw = now;
        L().print(F("GPS_RAW,"));
        L().println(ublox::last_nmea());
    }

    static uint32_t l_link = 0;
    if (crsf::receiving() && (now - l_link) >= 500) {
        l_link = now;
        const crsf::LinkStats& lk = crsf::link();
        L().print(F("LINK,up_rssi_dbm=")); L().print(lk.up_rssi_dbm);
        L().print(F(",up_lq="));           L().print(lk.up_lq);
        L().print(F(",up_snr="));          L().print(lk.up_snr);
        L().print(F(",rf_mode="));         L().println(lk.rf_mode);
    }

    static uint32_t l_sd = 0;
    if ((now - l_sd) >= 5000) {
        l_sd = now;                                  // SD runtime service is asynchronous
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
        L().print(F("IMU_CONFIG,model=")); L().print(config::imu_model);
        L().print(F(",mag=")); L().print(config::enable_magnetometer?1:0);
        L().print(F(",error0=")); L().print(imu_v2::driver_error(0));
        L().print(F(",error1=")); L().print(imu_v2::driver_error(1));
        L().print(F(",regs0=")); L().print(imu_v2::driver_health_registers(0),HEX);
        L().print(F(",regs1=")); L().println(imu_v2::driver_health_registers(1),HEX);
        L().print(F("BMP_HEALTH,valid=")); L().print(bmp581::healthy()?1:0);
        L().print(F(",error=")); L().println(bmp581::error());
        const auto& md=mag350::diagnostics();
        L().print(F("MAG_HEALTH,enabled=")); L().print(config::enable_magnetometer?1:0);
        L().print(F(",initialized=")); L().print(md.initialized?1:0);
        L().print(F(",healthy=")); L().print(mag350::healthy()?1:0);
        L().print(F(",communicating=")); L().print(mag350::communicating()?1:0);
        L().print(F(",stage=")); L().print(md.stage);
        L().print(F(",result=")); L().print(int(md.result));
        L().print(F(",chip_id=")); L().print(md.chip_id);
        L().print(F(",id14=")); L().print(md.id14);
        L().print(F(",id15=")); L().print(md.id15);
        L().print(F(",bus_errors=")); L().print(md.bus_errors);
        L().print(F(",last_reg=")); L().print(md.last_error_register);
        L().print(F(",status=")); L().print(md.status);
        L().print(F(",samples=")); L().print(md.samples);
        L().print(F(",reads=")); L().print(md.reads);
        L().print(F(",invalid=")); L().print(md.invalid_samples);
        L().print(F(",bus_status=")); L().print(int(md.last_bus_status));
        L().print(F(",consecutive=")); L().print(md.consecutive_errors);
        L().print(F(",recoveries=")); L().print(md.recoveries);
        L().print(F(",otp_error=")); L().print(md.otp_error);
        L().print(F(",self_test=")); L().println(md.self_test_ok?1:0);
        L().print(F("MAG_OTP_STATUS,passes=")); L().print(md.otp_passes);
        L().print(F(",mismatch=")); L().println(md.otp_mismatch);
        // One short saved-diagnostic chunk per status tick. This does not
        // access the sensor, and late USB joins can collect a complete cycle.
        static unsigned mag_dump_chunk=0;
        constexpr unsigned otp_chunks=mag350::otp_word_count/8;
        constexpr unsigned trim_chunks=(mag350::trim_count+7)/8;
        if(mag_dump_chunk<mag350::otp_pass_count*otp_chunks) {
            const unsigned pass=mag_dump_chunk/otp_chunks,offset=(mag_dump_chunk%otp_chunks)*8;
            L().print(F("MAG_OTP,pass=")); L().print(pass);
            L().print(F(",offset=")); L().print(offset);
            L().print(F(",words="));
            for(unsigned i=0;i<8;++i) { if(i) L().print('/'); L().print(md.otp[pass][offset+i],HEX); }
        } else {
            const unsigned offset=(mag_dump_chunk-mag350::otp_pass_count*otp_chunks)*8;
            L().print(F("MAG_TRIM,offset=")); L().print(offset);
            L().print(F(",bits="));
            for(unsigned i=offset;i<offset+8&&i<mag350::trim_count;++i) {
                uint32_t bits; std::memcpy(&bits,&md.trim[i],sizeof(bits));
                if(i!=offset) L().print('/'); L().print(bits,HEX);
            }
        }
        L().println();
        mag_dump_chunk=(mag_dump_chunk+1)%(mag350::otp_pass_count*otp_chunks+trim_chunks);
        L().print(F("MAG_CHECK,x=")); L().print(md.self_test_x,2);
        L().print(F(",y=")); L().print(md.self_test_y,2);
        L().print(F(",err=")); L().print(md.error_reg);
        L().print(F(",pmu=")); L().print(md.pmu);
        L().print(F(",aggr=")); L().print(md.aggr);
        L().print(F(",axes=")); L().print(md.axes);
        L().print(F(",st=")); L().println(md.self_test_reg);
        // Diagnostic values may be invalid; only MAG contains accepted data.
        L().print(F("MAG_DATA,x=")); L().print(md.last_sample.x_ut,2);
        L().print(F(",y=")); L().print(md.last_sample.y_ut,2);
        L().print(F(",z=")); L().print(md.last_sample.z_ut,2);
        L().print(F(",temp=")); L().print(md.last_sample.temp_c,2);
        L().print(F(",raw="));
        for(unsigned i=0;i<4;++i) { if(i) L().print('/'); L().print(md.raw[i]); }
        L().println();
        const bool attitude_ok=heading_attitude_valid(),mag_ok=magnetic_driver_healthy();
        const bool mag_aiding=s_heading.aiding(now,attitude_ok,mag_ok);
        const bool heading_ok=s_heading.heading_valid(now,attitude_ok,mag_ok);
        const bool hold=s_mode_cur==modes::Id::assist&&s_mode_assist.heading_hold()&&heading_ok;
        L().print(F("YAW_STATUS,source=")); L().print(!attitude_ok?"NONE":mag_aiding?"BMM350":"GYRO");
        L().print(F(",valid=")); L().print(heading_ok?1:0);
        L().print(F(",reason=")); L().print(estimation::MagHeading::state_name(s_heading.state(now,attitude_ok,mag_ok)));
        L().print(F(",heading=")); L().print(heading_degrees(ahrs::yaw_rad()),1);
        L().print(F(",mag_heading=")); L().print(heading_degrees(s_heading.measured_rad()),1);
        L().print(F(",field=")); L().print(s_heading.field_ut(),2);
        L().print(F(",innovation=")); L().print(s_heading.innovation_rad()*kRadToDeg,1);
        L().print(F(",configured=")); L().print(s_heading.configured()?1:0);
        L().print(F(",hold=")); L().print(hold?1:0);
        L().print(F(",target=")); L().println(hold?heading_degrees(s_mode_assist.heading_target_rad()):0,1);
        // Continuous health even with no fix/no receiver and for late GUI joins.
        L().print(F("GPS_HEALTH,rx=")); L().print(ublox::receiving()?1:0);
        L().print(F(",nmea=")); L().print(ublox::nmea_valid()?1:0);
        L().print(F(",fix=")); L().print(ublox::fix());
        L().print(F(",used=")); L().print(ublox::satellites_used());
        L().print(F(",visible=")); L().print(ublox::satellites_visible());
        L().print(F(",baud=")); L().print(ublox::current_baud());
        L().print(F(",bytes=")); L().print(ublox::rx_bytes());
        L().print(F(",messages=")); L().println(ublox::valid_messages());
        float bx, by, bz; ahrs::gyro_bias_dps(bx, by, bz);
        L().print(F("IMU_HEALTH,")); L().print(imu_v2::sensor_healthy(0)); L().print(',');
        L().print(imu_v2::sensor_healthy(1)); L().print(','); L().print(imu_v2::active()); L().print(','); L().print(imu_v2::ambiguous());
        L().print(','); L().println(imu_v2::healthy());
        L().print(F("EST,bias_ready="));   L().print(imu_v2::bias_ready() ? 1 : 0);
        L().print(F(",gbias="));           L().print(bx, 2); L().print('/');
        L().print(by, 2); L().print('/');  L().print(bz, 2);
        L().print(F(",acc_trust="));       L().print(ahrs::acc_trust(), 2);
        L().print(F(",agl_m="));           L().print(g_alt_agl_m, 2);
        L().print(F(",climb_mps="));       L().print(g_climb_mps, 2);
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
        L().print(F(",telem_tx="));           L().print(crsf::telem_frames_tx());
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
        L().print(F(",flight_enabled=")); L().print(config::flight_enabled ? 1 : 0);
        L().print(F(",timing_fault=")); L().print(s_control_fault ? 1 : 0);
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
        L().print(st.critical ? F("*,hz=") : F(",hz="));
        L().print(st.rate_hz);
        L().print(F(",min_us="));    L().print(st.min_us);
        L().print(F(",mean_us="));   L().print(st.mean_us);
        L().print(F(",max_us="));    L().print(st.max_us);
        L().print(F(",overruns=")); L().print(st.overruns);
        L().println();
    }
    L().print(F("SCHED,_loop,passes="));   L().print(sched::loop_count());
    L().print(F(",worst_pass_us="));       L().println(sched::worst_pass_us());
}

// Explicit USB maintenance command, for a landed/disarmed aircraft. This is
// not a ground detector. Fresh RC must show CH5 off and throttle <= 5%; after
// first arming, stale RC/failsafe must never authorize a reboot.
bool dfu_allowed()
{
    if (s_arming.armed() || s_flying) return false;
    const bool linked = crsf::receiving();
    if (s_flight_session && (!linked || s_failsafe.active())) return false;
    const auto& ch = crsf::channels();
    return !linked || (ch.us[kArmCh] <= kArmHi && ch.us[kThrCh] <= 1050);
}

// Bench command reader over USB CDC. Newline-terminated tokens:
//   RESET_STATS   -- zero the scheduler counters for a clean measurement window
//   dfu          -- reset into STM32 ROM DFU, without BOOT/RESET buttons
//   REBOOT_BL    -- compatibility alias for dfu
void task_cmd()   // 10 Hz
{
    static char buf[24];
    static uint8_t len = 0;
    static bool overflow=false;
    for (int guard = 0; guard < 128; guard++) {   // bounded drain
        const int c = usb_stream::read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            if(overflow) { overflow=false; len=0; L().println(F("NAK,OVERFLOW")); continue; }
            buf[len] = 0;
            if      (!strcmp(buf, "RESET_STATS")) { sched::reset_stats(); L().println(F("ACK,RESET_STATS")); }
            else if (!strcmp(buf, "dfu") || !strcmp(buf, "REBOOT_BL")) {
                if (dfu_allowed()) {
                    len=0; // clear before the noreturn call (also used by mocks)
                    L().println(F("ACK,DFU")); // best effort; reset may precede USB TX
                    hal::jump_to_bootloader();
                }
                else L().println(F("NAK,DFU,DISARM_IDLE_REQUIRED"));
            }
            else if (len)                         { L().print(F("NAK,")); L().println(buf); }
            len = 0;
        } else if (overflow) {
            continue;
        } else if (c < 0x20 || c > 0x7e) {
            len = 0; overflow=true; // binary/NUL cannot turn a prefix into a command
        } else if (len < sizeof(buf) - 1) {
            buf[len++] = (char)c;
        } else {
            len = 0; overflow=true;
        }
    }
}

void initialize_outputs()
{
    // Define outputs and idle before sensor/storage bring-up, without a USB wait.
    for(unsigned i=0;i<8;++i) {
        s_srv[i].min_us=config::servo_min[i]; s_srv[i].center_us=config::servo_center[i];
        s_srv[i].max_us=config::servo_max[i]; s_srv[i].reversed=config::servo_reverse[i];
    }
    s_output_ready=hal::pwm_config(hal::PwmGroup::ailerons,config::pwm_hz_ailerons)==hal::Status::ok;
    s_output_ready=(hal::pwm_config(hal::PwmGroup::tail,config::pwm_hz_tail)==hal::Status::ok)&&s_output_ready;
    s_output_ready=(hal::pwm_config(hal::PwmGroup::motors,config::pwm_hz_motors)==hal::Status::ok)&&s_output_ready;
    for(unsigned i=0;i<8;++i) {
        const auto group=i<2?hal::PwmGroup::ailerons:i<6?hal::PwmGroup::tail:hal::PwmGroup::motors;
        s_output_ready=(hal::pwm_write_us(group,i<2?i:i<6?i-2:i-6,s_srv[i].safe_us(control::is_motor_output(i)))==hal::Status::ok)&&s_output_ready;
    }
}

} // namespace

void setup()
{
    hal::init();
    const auto reset=hal::reset_cause();
    const bool watchdog_reset=reset==hal::ResetCause::iwdg || reset==hal::ResetCause::wwdg;
    if(watchdog_reset) { s_control_fault=true; s_assist_lockout=true; hal::watchdog_kick(); }

    usb_stream::begin();
    ublox::begin(kGpsBootBaud);
    crsf::begin(420000);   // ER8 on J2 / UART4 (PA1 rx / PA0 tx), 420000 8N1

    s_log=&usb_stream::log();
    L().print(F("boot: v2.2 dual ")); L().print(config::imu_model);
    L().println(F("/BMP581/SAM-M10Q; development build"));
    L().println(config::flight_enabled?F("FLIGHT_GATE,enabled"):F("FLIGHT_GATE,bench_motor_inhibit"));
    initialize_outputs();
    hal::delay_ms(5); // sensor power rails settle, startup only
    s_bmi_ready=!watchdog_reset && imu_v2::begin();
    L().println(s_bmi_ready?F("BMI_STATUS,1"):F("BMI_STATUS,0"));

    // ASSIST (FBWA): stick -> clamped attitude angle -> rate loop -> mixer.
    // Rough first gains, verified only in SITL (--assist-check) -- FF-dominant
    // small P, light I. Actual airframe tuning/qualification remains outstanding.
    {
        control::AttitudeCtrlConfig att; // body-rate turn geometry; no GPS-as-airspeed
        control::RateCtrlConfig rate;
        rate.sample_hz = 400.0f;                         // == task_control rate
        rate.roll.kff = 0.006f; rate.roll.kp = 0.010f; rate.roll.ki = 0.02f;
        rate.roll.i_max = 0.4f; rate.roll.d_lpf_hz = 25.0f;
        rate.pitch = rate.roll; rate.pitch.kff = 0.010f; rate.pitch.kp = 0.020f;
        rate.yaw   = rate.roll; rate.yaw.kff = 0.004f; rate.yaw.kp = 0.006f; rate.yaw.ki = 0.0f;
        s_mode_assist.set_tuning(config::assist_tuning);
        s_mode_takeoff.set_tuning(config::assist_tuning);
        s_mode_assist.configure(att, rate, 0.70f, 0.45f);  // roll/pitch limits, radians

        // TKOFF: roll wing-leveller only (pitch/yaw/throttle manual). Reuse the
        // ASSIST roll rate gains; small bank authority and a gentle rate demand
        // near the ground.
        control::PidGains tk_roll;
        tk_roll.kff = 0.006f; tk_roll.kp = 0.010f; tk_roll.ki = 0.02f;
        tk_roll.i_max = 0.4f; tk_roll.d_lpf_hz = 25.0f;
        s_mode_takeoff.configure(tk_roll, 400.0f,
                                 110.0f,    // angle-P, dps per rad
                                 120.0f,    // max roll rate, dps (inactive at this bank cap)
                                 0.175f);   // max bank, rad (~10 deg) -- crosswind wing-low
                                            // only; you keep the stick centred on takeoff
    }

    const bool bmp_ok = !watchdog_reset && bmp581::begin();
    L().println(bmp_ok ? F("BMP_STATUS,1") : F("BMP_STATUS,0"));
    ublox::drain_rx();

    const bool mag_ok=config::enable_magnetometer && !watchdog_reset && mag350::begin();
    L().println(!config::enable_magnetometer?F("MAG_STATUS,disabled"):mag_ok?F("MAG_STATUS,1"):F("MAG_STATUS,0"));
    if(config::enable_sd_logging && !watchdog_reset) switch (sd_bin_log::begin(s_log_ring)) {
        case sd_bin_log::Result::ok:
            L().print(F("SD_STATUS,1,")); L().println(sd_bin_log::name()); break;
        case sd_bin_log::Result::open_failed:
            L().println(F("SD_STATUS,0,open_failed")); break;
        case sd_bin_log::Result::begin_failed:
            L().println(F("SD_STATUS,0,begin_failed")); break;
    }

    imu_v2::set_observer(imu_log);
    s_telemetry.configure(config::telemetry);

    // Critical chain first (marked *): it runs at the top of every pass and is
    // re-serviced after slow non-critical tasks. This is cooperative, not
    // preemptive: blocking I/O can still stall everything. SD runtime service is asynchronous.
    // BMI writes g_*/ahrs before control reads them.
    sched::add("crsf",     400, task_crsf, true);
    sched::add("bmi",      400, task_bmi,     /*critical=*/true);
    sched::add("control",  400, task_control, /*critical=*/true);

    sched::add("gps",       50, task_gps);
    sched::add("crsf_tx",  100, task_crsf_tx);    // FC -> handset telemetry
    if(config::enable_magnetometer) sched::add("mag",50,task_mag);
    sched::add("bmp",      100, task_bmp);
    sched::add("ctl_log",  100, task_control_log);
    sched::add("log",       50, task_log);        // pack frame -> ring (fast)
    sched::add("log_flush",4000,task_log_flush); // no filesystem calls, no polling waits
    sched::add("stream",    20, task_stream);     // BMI/ATT/OUT/RC USB echo
    sched::add("cmd",       10, task_cmd);        // USB bench commands
    sched::add("debug",      2, task_debug);      // low-rate status lines
    sched::add("sched",      1, task_sched_report);

    // A stuck cooperative task cannot leave the last PWM command forever.
    // Watchdog recovery skips slow sensor/storage initialization and keeps
    // motors inhibited; RC surface passthrough remains available.
    sched::set_watchdog_ms(100);

    sched::run();   // never returns
}

void loop() { }   // unused -- sched::run() owns the main loop
