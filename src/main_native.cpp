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
#include "core/scheduler.hpp"
#include "control/rc_channel.hpp"
#include "control/srv_channel.hpp"
#include "control/mixer.hpp"
#include "core/failsafe.hpp"
#include "core/arming.hpp"
#include "modes/mode_manual.hpp"

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
    for (int i = 0; i < 20; i++)
        ahrs::update(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.005f);
    fails += check("ahrs level -> |roll|,|pitch| < 0.01 rad",
                   ahrs::roll_rad() < 0.01f && ahrs::roll_rad() > -0.01f &&
                   ahrs::pitch_rad() < 0.01f && ahrs::pitch_rad() > -0.01f);

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

    // --- mode_manual = passthrough via the mixer ---
    {
        modes::ModeManual m;
        control::Outputs o;
        m.update({ 0.4f, 0.0f, 0.0f, 0.3f }, 0.0025f, o);
        fails += check("mode_manual: roll -> ailerons, throttle -> ESCs",
                       o.ch[0] == 0.4f && o.ch[1] == 0.4f &&
                       o.ch[6] == 0.3f && o.ch[7] == 0.3f);
    }

    std::printf(fails ? "\nRESULT: %d FAIL\n" : "\nRESULT: all pass\n", fails);
    return fails;
}
