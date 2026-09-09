//
// main_sitl.cpp -- [env:sitl] desktop simulator.
//
// Runs pilot stick input through the REAL control chain (RcChannel -> mode ->
// mixer -> SrvChannel), feeds the surface/throttle commands to the 6DOF model,
// synthesizes IMU/baro/GPS, runs the AHRS on them, and prints a state CSV to
// stdout. Lets control-logic changes be checked without hardware.
//
//   pio run -e sitl && .pio/build/sitl/program              # built-in maneuver
//   .pio/build/sitl/program --secs 30 --hz 400 > run.csv
//   .pio/build/sitl/program --script sticks.csv > run.csv   # t,roll,pitch,yaw,thr
//
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>

#include "sitl/aircraft.hpp"
#include "sitl/sensors.hpp"
#include "control/rc_channel.hpp"
#include "control/srv_channel.hpp"
#include "modes/mode_manual.hpp"
#include "modes/mode_assist.hpp"
#include "modes/mode_takeoff.hpp"
#include "estimation/ahrs.hpp"

namespace {

constexpr double kRad2Deg = 57.29577951308232;

struct StickCmd { double roll, pitch, yaw, thr; };

// Built-in maneuver: hold, then roll / pitch / yaw doublets, then a throttle nudge.
// `thr0` is the trim throttle; doublets are small so the aircraft stays flyable.
StickCmd builtin(double t, double thr0)
{
    StickCmd c { 0.0, 0.0, 0.0, thr0 };
    auto doublet = [&](double t0, double amp, double& axis) {
        if (t >= t0 && t < t0 + 0.4)             axis =  amp;
        else if (t >= t0 + 0.4 && t < t0 + 0.8)  axis = -amp;
    };
    doublet(2.0, 0.30, c.roll);
    doublet(6.0, 0.20, c.pitch);
    doublet(10.0, 0.25, c.yaw);
    if (t >= 14.0 && t < 16.0) c.thr = thr0 + 0.2;
    return c;
}

std::vector<std::pair<double, StickCmd>> load_script(const char* path)
{
    std::vector<std::pair<double, StickCmd>> v;
    FILE* f = std::fopen(path, "r");
    if (!f) { std::fprintf(stderr, "cannot open %s\n", path); std::exit(1); }
    char line[256];
    while (std::fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == 't') continue;
        double t, r, p, y, th;
        if (std::sscanf(line, "%lf,%lf,%lf,%lf,%lf", &t, &r, &p, &y, &th) == 5)
            v.push_back({ t, { r, p, y, th } });
    }
    std::fclose(f);
    return v;
}

} // namespace

int main(int argc, char** argv)
{
    double secs = 20.0, hz = 400.0;
    const char* script = nullptr;
    bool check = false, assist = false, takeoff = false;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--secs")  && i + 1 < argc) secs = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hz") && i + 1 < argc) hz = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--script") && i + 1 < argc) script = argv[++i];
        else if (!std::strcmp(argv[i], "--check")) { check = true; script = nullptr; secs = 20.0; }
        else if (!std::strcmp(argv[i], "--assist")) assist = true;
        else if (!std::strcmp(argv[i], "--assist-check")) { check = true; assist = true; script = nullptr; secs = 14.0; }
        else if (!std::strcmp(argv[i], "--takeoff")) takeoff = true;
    }
    const double dt = 1.0 / hz;

    auto scripted = script ? load_script(script) : std::vector<std::pair<double, StickCmd>>{};

    sitl::Aircraft ac;
    ac.reset(18.0, 100.0);
    ahrs::reset();
    const double de_trim  = ac.trim_elevator();
    const double thr_trim = ac.trim_throttle();
    std::fprintf(stderr, "trim: elevator=%.3f throttle=%.3f\n", de_trim, thr_trim);

    control::RcChannel rc_roll, rc_pitch, rc_yaw, rc_thr;
    modes::ModeManual  mode_manual;
    modes::ModeAssist  mode_assist;
    modes::ModeTakeoff mode_takeoff;
    {
        control::AttitudeCtrlConfig ac;   // defaults
        control::RateCtrlConfig rc;
        rc.roll.kff  = 0.006f; rc.roll.kp = 0.010f; rc.roll.ki = 0.02f;
        rc.roll.i_max = 0.4f;  rc.roll.d_lpf_hz = 25.0f;
        rc.pitch = rc.roll;    rc.pitch.kff = 0.010f; rc.pitch.kp = 0.020f;
        rc.yaw   = rc.roll;    rc.yaw.kff  = 0.004f;  rc.yaw.kp  = 0.006f; rc.yaw.ki = 0.0f;
        mode_assist.configure(ac, rc, 0.70f, 0.45f, 80.0f);
        mode_takeoff.configure(rc.roll, rc.sample_hz, 110.0f, 120.0f, 0.35f);
    }
    modes::Mode* mode = takeoff ? static_cast<modes::Mode*>(&mode_takeoff)
                      : assist  ? static_cast<modes::Mode*>(&mode_assist)
                                : static_cast<modes::Mode*>(&mode_manual);
    mode->enter(control::Outputs{});

    if (!check)
        std::printf("t,N,E,D,VN,VE,VD,roll_deg,pitch_deg,yaw_deg,p_dps,q_dps,r_dps,"
                    "ahrs_roll,ahrs_pitch,ahrs_yaw,ail,ele,rud,thr,airspeed\n");

    // --check accumulators
    double trim_roll_max = 0, trim_pitch_max = 0, trim_V_err = 0;
    double roll_doublet_peak = 0;   // max roll during t in [2.0, 2.8]
    double end_roll = 0, end_pitch = 0, end_V = 0;
    double ahrs_roll_err_max = 0, ahrs_pitch_err_max = 0;
    // --assist-check accumulators
    double as_hold_bank_min = 999, as_hold_bank_max = -999;   // t in [3.5, 6]
    double as_rollrate_abs_max = 0;                           // t in [3.5, 6]
    double as_return_bank_abs = 999;                          // min |roll| in t [8.5, 10]

    sitl::Sensors sn_prev = sitl::synth(ac.state());
    size_t si = 0;
    for (double t = 0.0; t < secs; t += dt) {
        StickCmd sc;
        if (script) {
            while (si + 1 < scripted.size() && scripted[si + 1].first <= t) si++;
            sc = scripted.empty() ? StickCmd{ 0, 0, 0, thr_trim } : scripted[si].second;
        } else if (assist) {
            // ASSIST maneuver: roll stick to +0.7 (t 1-7, target ~0.49 rad),
            // release to level (t 7-14). Pitch nudge t 10-12.
            sc = StickCmd{ 0.0, 0.0, 0.0, thr_trim };
            if (t >= 1.0 && t < 7.0)  sc.roll  = 0.7;
            if (t >= 10.0 && t < 12.0) sc.pitch = 0.4;
        } else {
            sc = builtin(t, thr_trim);
        }

        // --- real control chain ---
        const uint16_t rc_us[4] = {
            (uint16_t)(1500 + sc.roll  * 500),
            (uint16_t)(1500 + sc.pitch * 500),
            (uint16_t)(1000 + sc.thr   * 1000),
            (uint16_t)(1500 + sc.yaw   * 500),
        };
        control::Sticks st;
        st.roll     = rc_roll.norm(rc_us[0]);
        st.pitch    = rc_pitch.norm(rc_us[1]);
        st.yaw      = rc_yaw.norm(rc_us[3]);
        st.throttle = rc_thr.unipolar(rc_us[2]);

        modes::ModeInput mi;
        mi.sticks       = st;
        mi.dt_s         = (float)dt;
        mi.roll_rad     = ahrs::roll_rad();
        mi.pitch_rad    = ahrs::pitch_rad();
        mi.yaw_rad      = ahrs::yaw_rad();
        mi.gyro_p_dps   = sn_prev.gx_dps;
        mi.gyro_q_dps   = sn_prev.gy_dps;
        mi.gyro_r_dps   = sn_prev.gz_dps;
        mi.airspeed_mps = (float)ac.airspeed_mps();   // SITL has "airspeed"; on HW use GPS speed

        control::Outputs o;
        mode->update(mi, o);
        sitl::Controls u {
            o.ch[0],                          // aileron  (both sides same)
            o.ch[2] + de_trim,                // elevator + airframe trim
            o.ch[4],                          // rudder
            0.5 * (o.ch[6] + o.ch[7]),        // throttle (mean of the two ESCs)
        };

        ac.step(u, dt);
        const sitl::State& s = ac.state();
        const sitl::Sensors sn = sitl::synth(s);
        sn_prev = sn;
        ahrs::update(sn.ax_g, sn.ay_g, sn.az_g, sn.gx_dps, sn.gy_dps, sn.gz_dps, (float)dt);

        double roll, pitch, yaw;
        ac.euler(roll, pitch, yaw);
        const double rd = roll * kRad2Deg, pd = pitch * kRad2Deg;

        if (check && assist) {
            const double bank = ahrs::roll_rad();   // ASSIST controls the AHRS estimate
            if (t >= 3.5 && t < 6.5) {
                if (bank < as_hold_bank_min) as_hold_bank_min = bank;
                if (bank > as_hold_bank_max) as_hold_bank_max = bank;
                const double rr = std::fabs(sn.gx_dps);
                if (rr > as_rollrate_abs_max) as_rollrate_abs_max = rr;
            }
            if (t >= 9.0 && t < 10.0 && std::fabs(bank) < as_return_bank_abs)
                as_return_bank_abs = std::fabs(bank);
            end_V = ac.airspeed_mps();
            continue;
        }
        if (check) {
            if (t < 1.9) {
                if (std::fabs(rd) > trim_roll_max)  trim_roll_max = std::fabs(rd);
                if (std::fabs(pd) > trim_pitch_max) trim_pitch_max = std::fabs(pd);
                const double e = std::fabs(ac.airspeed_mps() - 18.0);
                if (e > trim_V_err) trim_V_err = e;
            }
            if (t >= 2.0 && t < 2.8 && rd > roll_doublet_peak) roll_doublet_peak = rd;
            end_roll = rd; end_pitch = pd; end_V = ac.airspeed_mps();
            if (t > 0.5) {   // let the filter settle
                const double er = std::fabs(ahrs::roll_rad()  * kRad2Deg - rd);
                const double ep = std::fabs(ahrs::pitch_rad() * kRad2Deg - pd);
                if (er > ahrs_roll_err_max)  ahrs_roll_err_max  = er;
                if (ep > ahrs_pitch_err_max) ahrs_pitch_err_max = ep;
            }
            continue;
        }

        std::printf("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                    "%.3f,%.3f,%.3f,%.3f,%.2f\n",
                    t, s.pos_ned.x, s.pos_ned.y, s.pos_ned.z,
                    s.vel_ned.x, s.vel_ned.y, s.vel_ned.z,
                    roll * kRad2Deg, pitch * kRad2Deg, yaw * kRad2Deg,
                    sn.gx_dps, sn.gy_dps, sn.gz_dps,
                    ahrs::roll_rad() * kRad2Deg, ahrs::pitch_rad() * kRad2Deg,
                    ahrs::yaw_rad() * kRad2Deg,
                    u.aileron, u.elevator, u.rudder, u.throttle, ac.airspeed_mps());
    }

    if (check) {
        int fails = 0;
        auto ck = [&](const char* what, bool ok) {
            std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
            if (!ok) fails++;
        };

        if (assist) {
            const double tgt = 0.7 * 0.70;   // stick * max_roll_rad
            const double hold_mid = 0.5 * (as_hold_bank_min + as_hold_bank_max);
            std::fprintf(stderr,
                "ASSIST: hold bank %.2f..%.2f rad (target %.2f), roll-rate max %.0f dps, "
                "|bank| after release %.3f rad, V_end %.1f\n",
                as_hold_bank_min, as_hold_bank_max, tgt, as_rollrate_abs_max,
                as_return_bank_abs, end_V);
            ck("ASSIST holds commanded bank (within 0.15 rad of target)",
               std::fabs(hold_mid - tgt) < 0.15);
            ck("ASSIST bank does not oscillate (roll-rate max < 120 dps)",
               as_rollrate_abs_max < 120.0);
            ck("ASSIST returns to ~level after stick release (|bank| < 0.12 rad)",
               as_return_bank_abs < 0.12);
            ck("ASSIST: no departure (airspeed_end in [10,30])",
               end_V > 10.0 && end_V < 30.0);
            std::printf(fails ? "\nRESULT: %d FAIL\n" : "\nRESULT: all pass\n", fails);
            return fails;
        }

        ck("trim holds: |roll|<2 deg over first 1.9 s",        trim_roll_max  < 2.0);
        ck("trim holds: |pitch|<6 deg",                        trim_pitch_max < 6.0);
        ck("trim holds: airspeed within 3 m/s of 18",          trim_V_err     < 3.0);
        ck("aileron-right -> roll-right (doublet peak > 5 deg)", roll_doublet_peak > 5.0);
        ck("no departure: |roll_end|<30",                      std::fabs(end_roll)  < 30.0);
        ck("no departure: |pitch_end|<30",                     std::fabs(end_pitch) < 30.0);
        ck("no departure: airspeed_end in [10,30]",            end_V > 10.0 && end_V < 30.0);
        std::fprintf(stderr, "ahrs err max: roll %.1f deg, pitch %.1f deg\n",
                     ahrs_roll_err_max, ahrs_pitch_err_max);
        ck("gated AHRS: max roll error < 8 deg (incl. throttle bump)",  ahrs_roll_err_max  < 8.0);
        ck("gated AHRS: max pitch error < 8 deg (incl. throttle bump)", ahrs_pitch_err_max < 8.0);
        std::printf(fails ? "\nRESULT: %d FAIL\n" : "\nRESULT: all pass\n", fails);
        return fails;
    }
    return 0;
}
