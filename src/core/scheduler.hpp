#pragma once
//
// scheduler.hpp -- single-threaded cooperative scheduler.  No RTOS.
//
// The main loop. Tasks are fixed-rate free functions registered before run().
// Each dispatch pass:
//   1. runs every *critical* task that is due (the flight-control chain);
//   2. runs every non-critical task that is due, once -- and after any
//      non-critical task that ran longer than kYieldUs, re-services the
//      critical tasks so a slow logger / debug print cannot stall the rate
//      loop for more than one such task.
//
// Each dispatch is profiled with the CPU cycle counter (hal::cycles). A task
// that overruns its budget is a bug, not a tuning issue -- the per-task
// overrun count surfaces it. reset_stats() zeroes the counters so a clean
// measurement window can be taken after boot settles.
//
// Portable: hal only. Compiles and is tested on [env:native].
//
// Rules honoured: no dynamic allocation, no exceptions, fixed-size table.
// A non-critical task runs at most once per pass; a critical task may also be
// serviced once more mid-pass if a slow non-critical task made it fall due.
//
#include <cstddef>
#include <cstdint>

namespace sched {

using TaskFn = void (*)();

struct TaskStats {
    const char* name;
    uint32_t    rate_hz;
    bool        critical;
    uint32_t    runs;
    uint32_t    overruns;   // dispatched late (>half a period past due) OR ran longer than its period
    uint32_t    last_us;
    uint32_t    min_us;
    uint32_t    max_us;
    uint32_t    mean_us;
};

constexpr size_t kMaxTasks = 16;

// Register a periodic task. Call before run(). rate_hz > 0, fn != null.
// `critical` tasks (IMU read, rate/mixer/output) run first each pass and are
// re-serviced after any slow non-critical task. false if the table is full or
// an argument is invalid.
bool add(const char* name, uint32_t rate_hz, TaskFn fn, bool critical = false);

// Optional IWDG for run(): 0 (default) disables it. Started on the first
// run()/run_once() and kicked once per pass.
void set_watchdog_ms(uint32_t ms);

// One dispatch pass (see file header). Cheap when nothing is due. Exposed so a
// test / SITL harness can step the scheduler.
void run_once();

// Cooperative dispatch loop. Never returns.
[[noreturn]] void run();

// --- introspection (safe from inside a task) ---
size_t   task_count();
bool     get_stats(size_t i, TaskStats& out);
uint32_t loop_count();       // dispatch passes since start / since reset_stats()
uint32_t worst_pass_us();    // longest single pass since start / since reset_stats()
void     reset_stats();      // zero every per-task counter + worst_pass + loop_count

} // namespace sched
