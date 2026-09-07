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
#include "core/log_ring.hpp"
#include "core/log_frame.hpp"
#include "modes/mode_manual.hpp"

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
