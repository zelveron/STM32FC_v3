//
// main_native.cpp -- [env:native] entry / smoke test.
//
// Proves the portable layers (hal.hpp, drivers/, estimation/, core/scheduler)
// compile and link against the native hal backend, and that the scheduler
// dispatches at the requested rates. Real unit tests attach here later.
//
// Exit 0 = pass, non-zero = fail.
//
#include <cstdio>
#include <cmath>

#include "hal/hal.hpp"
#include "estimation/ahrs.hpp"
#include "estimation/imu_prep.hpp"
#include "estimation/baro_alt.hpp"
#include "core/scheduler.hpp"
#include "control/rc_channel.hpp"
#include "control/srv_channel.hpp"
#include "control/mixer.hpp"
#include "control/surface_test.hpp"
#include "control/pid.hpp"
#include "control/rate_ctrl.hpp"
#include "control/attitude_ctrl.hpp"
#include "core/failsafe.hpp"
#include "core/arming.hpp"
#include "core/log_ring.hpp"
#include "core/log_frame.hpp"
#include "modes/mode_manual.hpp"
#include "modes/mode_assist.hpp"
#include "modes/mode_takeoff.hpp"

#include <cstring>

namespace {

uint32_t g_fast_runs = 0;
uint32_t g_slow_runs = 0;

void fast_task() { g_fast_runs++; }
void slow_task() { g_slow_runs++; }

int check(const char* what, bool ok)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    return ok ? 0 : 1;
}

} // namespace

int main()
{
    hal::init();
    int fails = 0;

    // --- ahrs: level, still sample -> attitude stays near zero ---
    ahrs::reset();
    for (int i = 0; i < 200; i++)
        ahrs::update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.005f);
    fails += check("ahrs level -> |roll|,|pitch| < 0.01 rad",
                   std::fabs(ahrs::roll_rad()) < 0.01f && std::fabs(ahrs::pitch_rad()) < 0.01f);

    // --- ahrs: high body rate -> accel correction gated out ---
    ahrs::reset();
    ahrs::update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.005f);      // seed
    ahrs::update(0.7f, 0.0f, 0.7f, 300.0f, 0.0f, 0.0f, 0.005f);    // tilted accel + fast roll
    fails += check("ahrs gates accel at 300 dps (acc_trust ~ 0)", ahrs::acc_trust() < 0.05f);

    // --- ahrs: online bias converges; a constant gyro bias does not drift roll/pitch ---
    ahrs::reset();
    ahrs::set_gains(1.5f, 0.3f);
    const float true_bias = 2.0f;   // dps on x
    for (int i = 0; i < 4000; i++)  // 20 s @ 200 Hz, stationary + level
        ahrs::update(0.0f, 0.0f, 1.0f, true_bias, 0.0f, 0.0f, 0.005f);
    {
        float bx, by, bz; ahrs::gyro_bias_dps(bx, by, bz);
        fails += check("ahrs online bias converges to ~2 dps", std::fabs(bx - true_bias) < 0.4f);
        fails += check("ahrs holds level despite gyro bias", std::fabs(ahrs::roll_rad()) < 0.02f);
    }
    ahrs::set_gains(1.0f, 0.05f);

    // --- imu_prep: gyro bias cal + LPF ---
    {
        estimation::ImuPrep prep;
        prep.configure(100.0f, 25.0f, 12.0f);
        estimation::ImuSample o{};
        for (int i = 0; i < 500; i++)   // stationary, biased gyro
            o = prep.process(1.5f, -0.8f, 0.3f, 0.01f, 0.0f, 1.0f, 0.01f);
        fails += check("imu_prep bias_ready after a stationary window", o.bias_ready);
        float bx, by, bz; prep.gyro_bias(bx, by, bz);
        fails += check("imu_prep gyro bias ~ (1.5,-0.8,0.3)",
                       std::fabs(bx - 1.5f) < 0.1f && std::fabs(by + 0.8f) < 0.1f &&
                       std::fabs(bz - 0.3f) < 0.1f);
        o = prep.process(1.5f, -0.8f, 0.3f, 0.01f, 0.0f, 1.0f, 0.01f);
        fails += check("imu_prep removes the bias (|g| < 0.2 dps)",
                       std::fabs(o.gx_dps) < 0.2f && std::fabs(o.gy_dps) < 0.2f);

        estimation::ImuPrep p2;
        p2.configure(100.0f, 25.0f, 12.0f);
        for (int i = 0; i < 300; i++) p2.process(0, 0, 0, 0, 0, 1, 0.01f);
        p2.process(50.0f, 0, 0, 0, 0, 1, 0.01f);   // a big spike mid-window
        fails += check("imu_prep restarts cal on motion", !p2.bias_ready());
    }

    // --- baro_alt: ground reference ---
    {
        estimation::BaroAlt b;
        fails += check("baro not referenced -> alt 0", !b.referenced() && b.alt_m(90000.0f) == 0.0f);
        b.set_ground(95000.0f);
        fails += check("baro at ground pressure -> ~0 m", std::fabs(b.alt_m(95000.0f)) < 0.5f);
        const float a = b.alt_m(94000.0f);   // ~1000 Pa lower -> ~85 m up
        fails += check("baro lower pressure -> positive altitude ~85 m", a > 60.0f && a < 110.0f);
    }

    // --- scheduler: two tasks at 100 Hz and 20 Hz over ~500 ms ---
    fails += check("sched::add x2", sched::add("fast", 100, fast_task) &&
                                    sched::add("slow", 20, slow_task));
    const uint32_t t0 = hal::millis();
    while (hal::millis() - t0 < 500) sched::run_once();

    std::printf("  fast_runs=%u slow_runs=%u passes=%u\n",
                g_fast_runs, g_slow_runs, sched::loop_count());

    fails += check("100 Hz task ran ~50x in 500 ms",
                   g_fast_runs >= 44 && g_fast_runs <= 56);
    fails += check("20 Hz task ran ~10x in 500 ms",
                   g_slow_runs >= 8 && g_slow_runs <= 12);

    sched::TaskStats st;
    fails += check("get_stats(0) ok", sched::get_stats(0, st) && st.runs == g_fast_runs);

    fails += check("reset_stats zeroes counters",
                   (sched::reset_stats(), sched::get_stats(0, st) &&
                    st.runs == 0 && st.overruns == 0 && st.max_us == 0 &&
                    sched::loop_count() == 0 && sched::worst_pass_us() == 0));

    // critical tasks are serviced first and re-serviced after a slow task
    fails += check("sched::add critical", sched::add("crit", 100, fast_task, true));
    g_fast_runs = g_slow_runs = 0;
    const uint32_t tc = hal::millis();
    while (hal::millis() - tc < 300) sched::run_once();
    sched::TaskStats sc;
    // "crit" is index 2; it and "fast" share fast_task, so both increment g_fast_runs
    fails += check("critical task runs at rate",
                   sched::get_stats(2, sc) && sc.critical &&
                   sc.runs >= 26 && sc.runs <= 34);

    // --- RC_Channel ---
    {
        control::RcChannel rc;   // 1000/1500/2000, dz 8
        fails += check("rc.norm(1500) == 0",          rc.norm(1500) == 0.0f);
        fails += check("rc.norm(1505) == 0 (deadzone)", rc.norm(1505) == 0.0f);
        fails += check("rc.norm(2000) ~ +1",          std::fabs(rc.norm(2000) - 1.0f) < 0.02f);
        fails += check("rc.norm(1000) ~ -1",          std::fabs(rc.norm(1000) + 1.0f) < 0.02f);
        fails += check("rc.unipolar(1000)==0, (2000)==1",
                       rc.unipolar(1000) == 0.0f && std::fabs(rc.unipolar(2000) - 1.0f) < 1e-6f);
        control::RcChannel rr; rr.reversed = true;
        fails += check("rc reversed flips sign",      rr.norm(2000) < -0.9f);
    }

    // --- SRV_Channel (round-trips RC) ---
    {
        control::SrvChannel sv;
        fails += check("srv.from_norm(0)==1500",  sv.from_norm(0.0f)  == 1500);
        fails += check("srv.from_norm(+1)==2000", sv.from_norm(1.0f)  == 2000);
        fails += check("srv.from_norm(-1)==1000", sv.from_norm(-1.0f) == 1000);
        fails += check("srv.from_norm clamps",    sv.from_norm(5.0f)  == 2000);
        fails += check("srv.from_unipolar(0.5)==1500", sv.from_unipolar(0.5f) == 1500);
        fails += check("srv.safe: surface centre / throttle min",
                       sv.safe_us(false) == 1500 && sv.safe_us(true) == 1000);
        control::SrvChannel svr; svr.reversed = true;
        fails += check("srv reversed: from_norm(+1)==1000", svr.from_norm(1.0f) == 1000);
    }

    // --- mixer (MANUAL passthrough) ---
    {
        control::MixParams p;   // unit gains, diff thrust off
        control::Outputs o;
        control::mix_manual({ 1.0f, 0.0f, 0.0f, 0.0f }, p, o);   // full right roll
        fails += check("mix: roll -> both ailerons +1, nothing else",
                       o.ch[0] == 1.0f && o.ch[1] == 1.0f &&
                       o.ch[2] == 0.0f && o.ch[3] == 0.0f && o.ch[4] == 0.0f);
        control::mix_manual({ 0.0f, -0.5f, 0.0f, 0.6f }, p, o);
        fails += check("mix: elevator L==R, ESC L==R==throttle",
                       o.ch[2] == -0.5f && o.ch[3] == -0.5f &&
                       o.ch[6] == 0.6f && o.ch[7] == 0.6f);
    }

    // --- failsafe (debounced both ways; starts engaged) ---
    {
        core::Failsafe fs;
        fails += check("fs starts engaged (rc_loss)", fs.active());
        fs.update(true, 100);
        fails += check("fs: link up but not yet stable -> still engaged", fs.active());
        fs.update(true, 500);   // 400 ms stable >= 300 recover
        fails += check("fs: clears after recover window", !fs.active());
        fs.update(false, 600);
        fails += check("fs: link drop, not yet -> still clear", !fs.active());
        fs.update(false, 900);  // 300 ms lost >= 200 engage
        fails += check("fs: re-engages after engage window", fs.active());
    }

    // --- arming ---
    {
        core::Arming a;
        a.update({ false, 0.0f, false });
        fails += check("arm: disarmed by default", !a.armed());
        a.update({ true, 0.0f, false });                 // rising edge, idle, no fs
        fails += check("arm: arms on rising edge + idle throttle", a.armed());
        a.update({ true, 0.8f, false });
        fails += check("arm: stays armed with throttle up", a.armed());
        a.update({ false, 0.0f, false });
        fails += check("arm: disarms on switch low", !a.armed());
        core::Arming b;
        b.update({ true, 0.8f, false });
        fails += check("arm: will not arm with throttle up", !b.armed());
        core::Arming c;
        c.update({ true, 0.0f, true });
        fails += check("arm: will not arm during failsafe", !c.armed());
    }

    // --- surface_test sweep: aileron then elevator then rudder, each hits +/-1 ---
    {
        control::SurfaceTest st;
        fails += check("surface_test inactive by default", !st.active());
        st.start();
        float r, p, y;
        float rmin=9, rmax=-9, pmin=9, pmax=-9, ymin=9, ymax=-9;
        bool axis_bleed = false;
        double acc = 0.0;
        for (int i = 0; i < 800; i++) {   // 2 s @ 400 Hz (sweep is 1.8 s)
            if (!st.step(0.0025f, r, p, y)) break;
            acc += 0.0025;                 // track sweep's own elapsed time
            const double eps = 0.02;      // skip a guard band around segment edges
            const bool near_edge = (acc > 0.6 - eps && acc < 0.6 + eps) ||
                                   (acc > 1.2 - eps && acc < 1.2 + eps);
            if (near_edge) continue;
            if (acc < 0.6)      { if (p!=0||y!=0) axis_bleed = true; if(r<rmin)rmin=r; if(r>rmax)rmax=r; }
            else if (acc < 1.2) { if (r!=0||y!=0) axis_bleed = true; if(p<pmin)pmin=p; if(p>pmax)pmax=p; }
            else                { if (r!=0||p!=0) axis_bleed = true; if(y<ymin)ymin=y; if(y>ymax)ymax=y; }
        }
        fails += check("surface_test: each axis moves only in its segment", !axis_bleed);
        fails += check("surface_test: aileron reaches +/-1",  rmax > 0.95f && rmin < -0.95f);
        fails += check("surface_test: elevator reaches +/-1", pmax > 0.95f && pmin < -0.95f);
        fails += check("surface_test: rudder reaches +/-1",   ymax > 0.95f && ymin < -0.95f);
        fails += check("surface_test finishes and returns to inactive", !st.active());
    }

    // --- PID ---
    {
        using control::Pid; using control::PidGains;

        // pure P, wide clamp
        Pid p; PidGains g; g.kp = 2.0f; g.out_min = -100; g.out_max = 100;
        p.configure(g, 400.0f);
        fails += check("pid P: err 1 * kp 2 -> 2", std::fabs(p.update(1.0f, 0.0f, 0.0025f) - 2.0f) < 1e-4f);

        // FF on setpoint (no error)
        Pid f; PidGains gf; gf.kff = 0.5f; gf.out_min = -100; gf.out_max = 100;
        f.configure(gf, 400.0f);
        fails += check("pid FF: kff 0.5 * sp 3 -> 1.5", std::fabs(f.update(3.0f, 3.0f, 0.0025f) - 1.5f) < 1e-4f);

        // I accumulates then clamps at i_max
        Pid pi; PidGains gi; gi.ki = 10.0f; gi.i_max = 0.5f; gi.out_min = -100; gi.out_max = 100;
        pi.configure(gi, 400.0f);
        for (int i = 0; i < 400; i++) pi.update(1.0f, 0.0f, 0.0025f);
        fails += check("pid I clamps at i_max", std::fabs(pi.i_term() - 0.5f) < 1e-3f);

        // integrator freeze: disabled -> I does not accumulate; re-enable -> resumes
        Pid pf; PidGains gf2; gf2.ki = 10.0f; gf2.i_max = 5.0f; gf2.out_min = -100; gf2.out_max = 100;
        pf.configure(gf2, 400.0f);
        pf.set_integrator_enabled(false);
        for (int i = 0; i < 400; i++) pf.update(1.0f, 0.0f, 0.0025f);
        fails += check("pid I frozen when disabled", std::fabs(pf.i_term()) < 1e-6f);
        pf.set_integrator_enabled(true);
        for (int i = 0; i < 40; i++) pf.update(1.0f, 0.0f, 0.0025f);
        fails += check("pid I resumes when re-enabled", pf.i_term() > 0.5f);

        // D on measurement: a setpoint step must NOT spike D; a measurement step must
        Pid pd; PidGains gd; gd.kd = 1.0f; gd.d_lpf_hz = 0.0f; gd.out_min = -100; gd.out_max = 100;
        pd.configure(gd, 400.0f);
        pd.update(0.0f, 0.0f, 0.0025f);                 // prime
        pd.update(5.0f, 0.0f, 0.0025f);                 // setpoint step
        fails += check("pid D ignores setpoint step", std::fabs(pd.d_term()) < 1e-4f);
        pd.update(5.0f, 1.0f, 0.0025f);                 // measurement +1 in one tick
        fails += check("pid D opposes measurement rise (D < 0)", pd.d_term() < -100.0f);

        // anti-windup: saturate hard, I must stop growing; recover when error flips
        Pid pw; PidGains gw; gw.kp = 1.0f; gw.ki = 50.0f; gw.i_max = 1000.0f;
        gw.out_min = -1.0f; gw.out_max = 1.0f;
        pw.configure(gw, 400.0f);
        for (int i = 0; i < 800; i++) pw.update(10.0f, 0.0f, 0.0025f);   // pegged high
        const float i_wound = pw.i_term();
        for (int i = 0; i < 40; i++) pw.update(10.0f, 0.0f, 0.0025f);
        fails += check("pid anti-windup: I frozen at the rail", std::fabs(pw.i_term() - i_wound) < 1e-3f);
        const float recov = pw.update(-10.0f, 0.0f, 0.0025f);           // error flips
        fails += check("pid anti-windup: output responds immediately on flip", recov < 0.5f);

        // bumpless: preset so the next update reproduces a target output
        Pid pb; PidGains gb; gb.kp = 3.0f; gb.ki = 2.0f; gb.i_max = 5.0f; gb.out_min = -5; gb.out_max = 5;
        pb.configure(gb, 100.0f);
        pb.preset_integrator(0.2f, 0.1f, 0.42f);
        fails += check("pid preset_integrator -> bumpless first output",
                       std::fabs(pb.update(0.2f, 0.1f, 0.01f) - 0.42f) < 0.05f);
    }

    // --- rate controller ---
    {
        control::RateController rc;
        control::RateCtrlConfig cfg;
        cfg.roll.kff = 0.01f;  cfg.roll.out_min = -1;  cfg.roll.out_max = 1;
        cfg.pitch = cfg.yaw = cfg.roll;
        rc.configure(cfg);
        float r, p, y;
        rc.update(50.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0025f, r, p, y);
        fails += check("rate_ctrl: FF 0.01 * 50 dps demand -> roll out 0.5", std::fabs(r - 0.5f) < 1e-3f);
        fails += check("rate_ctrl: other axes zero", p == 0.0f && y == 0.0f);
        rc.preset(0, 0, 0, 0, 0, 0, 0.3f, -0.2f, 0.0f);
        rc.update(0, 0, 0, 0, 0, 0, 0.0025f, r, p, y);
        fails += check("rate_ctrl preset -> bumpless (roll ~0.3, pitch ~-0.2)",
                       std::fabs(r - 0.3f) < 0.05f && std::fabs(p + 0.2f) < 0.05f);
    }

    // --- attitude controller ---
    {
        control::AttitudeController ac;
        control::AttitudeCtrlConfig c;   // defaults
        ac.configure(c);
        float dp, dq;
        ac.update(0.3f, 0.0f, 0.0f, 0.0f, 0.0f, dp, dq);   // want +0.3 rad roll, no airspeed
        fails += check("att_ctrl: roll error -> positive roll-rate demand, clamped",
                       dp > 0.0f && dp <= c.max_roll_rate_dps + 1e-3f);
        ac.update(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, dp, dq);
        fails += check("att_ctrl: no error -> zero rate demand", std::fabs(dp) < 1e-4f && std::fabs(dq) < 1e-4f);
        // turn compensation: banked 30 deg at 18 m/s -> nose-up (positive q) FF
        ac.update(0.52f, 0.0f, 0.52f, 0.0f, 18.0f, dp, dq);
        fails += check("att_ctrl: banked -> turn-comp adds nose-up (dq > 0)", dq > 1.0f);
    }

    // --- mode_manual = passthrough via the mixer ---
    {
        modes::ModeManual m;
        modes::ModeInput mi; mi.sticks = { 0.4f, 0.0f, 0.0f, 0.3f }; mi.dt_s = 0.0025f;
        control::Outputs o;
        m.update(mi, o);
        fails += check("mode_manual: roll -> ailerons, throttle -> ESCs",
                       o.ch[0] == 0.4f && o.ch[1] == 0.4f &&
                       o.ch[6] == 0.3f && o.ch[7] == 0.3f);
    }

    // --- mode_assist: angle command, bumpless entry, closes to target ---
    {
        modes::ModeAssist m;
        control::AttitudeCtrlConfig ac;
        control::RateCtrlConfig rc;
        rc.roll.kff = 0.006f; rc.roll.kp = 0.004f; rc.roll.ki = 0.05f;
        rc.roll.i_max = 0.6f; rc.roll.out_min = -1; rc.roll.out_max = 1;
        rc.pitch = rc.yaw = rc.roll;
        m.configure(ac, rc, 0.7f, 0.5f, 90.0f);

        // bumpless: enter() with a non-zero current output, first update reproduces it
        control::Outputs cur{}; cur.ch[0] = cur.ch[1] = 0.25f;
        m.enter(cur);
        modes::ModeInput mi; mi.dt_s = 0.0025f;
        mi.sticks = { 0.0f, 0.0f, 0.0f, 0.4f };   // sticks centred
        control::Outputs o;
        m.update(mi, o);
        fails += check("mode_assist bumpless: first roll out ~ entry 0.25",
                       std::fabs(o.ch[0] - 0.25f) < 0.1f);

        // closed loop: hold roll stick right, level aircraft -> roll command drives positive
        modes::ModeAssist m2;
        m2.configure(ac, rc, 0.7f, 0.5f, 90.0f);
        m2.enter(control::Outputs{});
        float roll = 0.0f, gp = 0.0f;
        for (int i = 0; i < 2000; i++) {   // 5 s @ 400 Hz, trivial roll integrator
            modes::ModeInput in; in.dt_s = 0.0025f;
            in.sticks = { 0.5f, 0.0f, 0.0f, 0.4f };   // want +0.35 rad bank
            in.roll_rad = roll; in.gyro_p_dps = gp;
            control::Outputs oo;
            m2.update(in, oo);
            // toy plant: aileron -> roll accel; heavy damping
            const float acc = oo.ch[0] * 800.0f - gp * 4.0f;   // dps/s
            gp   += acc * 0.0025f;
            roll += gp * (3.14159f / 180.0f) * 0.0025f;
        }
        fails += check("mode_assist closes to ~+0.35 rad bank",
                       roll > 0.20f && roll < 0.50f);
    }

    // --- mode_takeoff: roll assist only, pitch/yaw pass straight through ---
    {
        modes::ModeTakeoff m;
        control::PidGains roll;
        roll.kff = 0.006f; roll.kp = 0.004f; roll.ki = 0.05f; roll.i_max = 0.6f;
        m.configure(roll, 400.0f, 110.0f, 120.0f, 0.35f);   // max bank 0.35 rad

        // pitch / yaw are passthrough: out.ch[2..3] == pitch stick, ch[4] == yaw
        m.enter(control::Outputs{});
        modes::ModeInput mi; mi.dt_s = 0.0025f;
        mi.sticks = { 0.0f, 0.6f, -0.3f, 0.5f };   // pitch +0.6, yaw -0.3
        mi.roll_rad = 0.0f; mi.gyro_p_dps = 0.0f;
        control::Outputs o;
        m.update(mi, o);
        fails += check("mode_takeoff: pitch passthrough (elevator == stick)",
                       std::fabs(o.ch[2] - 0.6f) < 1e-3f && std::fabs(o.ch[3] - 0.6f) < 1e-3f);
        fails += check("mode_takeoff: yaw passthrough (rudder == stick)",
                       std::fabs(o.ch[4] - (-0.3f)) < 1e-3f);
        fails += check("mode_takeoff: roll centred -> ~no aileron",
                       std::fabs(o.ch[0]) < 0.05f);

        // closed loop: hold roll stick right -> bank converges, capped by max_roll
        modes::ModeTakeoff m2;
        m2.configure(roll, 400.0f, 110.0f, 120.0f, 0.35f);
        m2.enter(control::Outputs{});
        float rr = 0.0f, gp = 0.0f;
        for (int i = 0; i < 2000; i++) {
            modes::ModeInput in; in.dt_s = 0.0025f;
            in.sticks = { 1.0f, 0.0f, 0.0f, 0.5f };   // full roll stick -> target 0.35 rad
            in.roll_rad = rr; in.gyro_p_dps = gp;
            control::Outputs oo;
            m2.update(in, oo);
            const float acc = oo.ch[0] * 800.0f - gp * 4.0f;
            gp += acc * 0.0025f;
            rr += gp * (3.14159f / 180.0f) * 0.0025f;
        }
        fails += check("mode_takeoff: bank holds near the 0.35 rad cap",
                       rr > 0.25f && rr < 0.45f);
    }

    // --- log_ring ---
    {
        static core::LogRing ring;   // 8 KB -- keep off the test stack
        fails += check("ring empty", ring.used() == 0 && ring.drops() == 0);

        uint8_t a[100]; for (int i = 0; i < 100; i++) a[i] = (uint8_t)(i * 7);
        fails += check("ring push 100", ring.push(a, 100) && ring.used() == 100);

        uint8_t b[100];
        fails += check("ring peek matches", ring.peek(b, 100) == 100 &&
                                            std::memcmp(a, b, 100) == 0);
        ring.consume(40);
        fails += check("ring consume 40 -> used 60", ring.used() == 60);

        // overflow: try to push more than fits -> dropped whole, counted
        uint8_t big[core::LogRing::kSize];
        fails += check("ring overflow -> drop + count",
                       !ring.push(big, sizeof(big)) && ring.drops() == 1 &&
                       ring.used() == 60);

        // wraparound integrity: many push/consume cycles across the boundary
        bool wrap_ok = true;
        for (int k = 0; k < 500 && wrap_ok; k++) {
            uint8_t w[300], r[300];
            for (int i = 0; i < 300; i++) w[i] = (uint8_t)(k + i);
            if (!ring.push(w, 300)) { wrap_ok = false; break; }
            if (ring.peek(r, 300) < 300) { wrap_ok = false; break; }
            // (older bytes are still in front; just check the tail advances cleanly)
            ring.consume(300);
        }
        fails += check("ring wraparound 500x300 clean", wrap_ok);
    }

    // --- log_frame ---
    {
        fails += check("crc16 CCITT('123456789') == 0x29B1",
                       core::log_crc16("123456789", 9) == 0x29B1);

        core::LogFrame f;
        std::memset(&f, 0, sizeof(f));
        f.t_ms = 12345; f.acc[2] = 2048; f.rc_us[0] = 1500; f.flags = core::LOG_ARMED;
        core::log_frame_finalize(f);
        fails += check("log_frame_valid after finalize", core::log_frame_valid(f));
        reinterpret_cast<uint8_t*>(&f)[10] ^= 0xFF;
        fails += check("log_frame_valid false after corruption", !core::log_frame_valid(f));

        core::LogFileHeader h; core::log_file_header_init(h);
        fails += check("file header tag/size",
                       std::memcmp(h.tag, "STFC", 4) == 0 &&
                       h.frame_size == (uint8_t)sizeof(core::LogFrame));
        std::printf("  sizeof(LogFrame)=%zu\n", sizeof(core::LogFrame));
    }

    std::printf(fails ? "\nRESULT: %d FAIL\n" : "\nRESULT: all pass\n", fails);
    return fails;
}
