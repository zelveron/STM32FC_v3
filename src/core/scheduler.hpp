#pragma once
//
// scheduler.hpp -- single-threaded cooperative scheduler.  No RTOS.
//
// The main loop. Tasks are fixed-rate free functions registered before run().
// Each dispatch pass walks the table, runs whatever is due, and profiles it
// with the CPU cycle counter (hal::cycles). A task that overruns its budget is
// a bug, not a tuning issue -- the per-task overrun count surfaces it.
//
// Portable: hal only. Compiles and is tested on [env:native].
//
// Rules honoured: no dynamic allocation, no exceptions, fixed-size table,
// bounded work per pass (each task runs at most once per pass).
//
#include <cstddef>
#include <cstdint>

namespace sched {

using TaskFn = void (*)();

struct TaskStats {
    const char* name;
    uint32_t    rate_hz;
    uint32_t    runs;
    uint32_t    overruns;   // dispatched late (>half a period past due) OR ran longer than its period
    uint32_t    last_us;
    uint32_t    min_us;
    uint32_t    max_us;
    uint32_t    mean_us;
};

constexpr size_t kMaxTasks = 16;

// Register a periodic task. Call before run(). rate_hz > 0, fn != null.
// false if the table is full or an argument is invalid.
bool add(const char* name, uint32_t rate_hz, TaskFn fn);

// Optional IWDG for run(): 0 (default) disables it. Started on the first
// run()/run_once() and kicked once per pass.
void set_watchdog_ms(uint32_t ms);

// One dispatch pass: run every task that is due, once. Cheap when nothing is
// due. Exposed so a test / SITL harness can step the scheduler.
void run_once();

// Cooperative dispatch loop. Never returns.
[[noreturn]] void run();

// --- introspection (safe from inside a task) ---
size_t   task_count();
bool     get_stats(size_t i, TaskStats& out);
uint32_t loop_count();       // dispatch passes since start
uint32_t worst_pass_us();    // longest single pass

} // namespace sched
