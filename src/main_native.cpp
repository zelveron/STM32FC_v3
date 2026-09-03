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

#include "hal/hal.hpp"
#include "estimation/ahrs.hpp"
#include "core/scheduler.hpp"

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

    std::printf(fails ? "\nRESULT: %d FAIL\n" : "\nRESULT: all pass\n", fails);
    return fails;
}
