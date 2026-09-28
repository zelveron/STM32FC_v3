#include "scheduler.hpp"
#include "../hal/hal.hpp"

namespace sched {
namespace {

struct Task {
    const char* name;
    TaskFn      fn;
    uint32_t    rate_hz;
    uint32_t    period_us;
    uint32_t    next_us;        // due time, hal::micros() timebase
    bool        critical;
    // stats
    uint32_t    runs;
    uint32_t    overruns;
    uint32_t    last_us;
    uint32_t    min_us;
    uint32_t    max_us;
    uint64_t    sum_us;
    uint32_t    completed_us;
};

Task     s_task[kMaxTasks];
size_t   s_n = 0;
uint32_t s_loops = 0;
uint32_t s_worst_pass_us = 0;
uint32_t s_wdt_ms = 0;
bool     s_wdt_started = false;

// After a non-critical task runs longer than this, re-service the critical
// tasks before continuing the pass.
constexpr uint32_t kYieldUs = 500;

inline uint32_t cyc_to_us(uint32_t c)
{
    return (uint32_t)((uint64_t)c * 1000000ull / hal::cpu_hz());
}

// Run one due task and fold in its timing. `now` is the dispatch pass time.
void run_task(Task& t, uint32_t now)
{
    const bool late = (int32_t)(now - t.next_us) > (int32_t)(t.period_us / 2);

    const uint32_t c0 = hal::cycles();
    t.fn();
    const uint32_t us = cyc_to_us(hal::cycles() - c0);

    t.runs++;
    t.last_us = us;
    if (us < t.min_us) t.min_us = us;
    if (us > t.max_us) t.max_us = us;
    t.sum_us += us;
    t.completed_us = hal::micros();
    if (late || us > t.period_us) t.overruns++;

    // next slot; resync if a whole period behind (no catch-up storm)
    t.next_us += t.period_us;
    const uint32_t after = hal::micros();
    if ((int32_t)(after - t.next_us) >= 0) t.next_us = after + t.period_us;
}

// Run every critical task that is currently due. Called at the top of a pass
// and again after any slow non-critical task.
void service_critical()
{
    for (size_t i = 0; i < s_n; i++) {
        Task& t = s_task[i];
        if (!t.critical) continue;
        const uint32_t now = hal::micros();
        if ((int32_t)(now - t.next_us) < 0) continue;
        run_task(t, now);
    }
}

} // namespace

bool add(const char* name, uint32_t rate_hz, TaskFn fn, bool critical)
{
    if (s_n >= kMaxTasks || rate_hz == 0 || fn == nullptr) return false;
    Task& t = s_task[s_n++];
    t.name      = name;
    t.fn        = fn;
    t.rate_hz   = rate_hz;
    t.period_us = 1000000u / rate_hz;
    t.next_us   = hal::micros();
    t.critical  = critical;
    t.runs = t.overruns = t.last_us = t.max_us = 0;
    t.min_us = 0xFFFFFFFFu;
    t.sum_us = 0;
    return true;
}

void set_watchdog_ms(uint32_t ms) { s_wdt_ms = ms; }

void run_once()
{
    if (s_wdt_ms && !s_wdt_started) { hal::watchdog_start(s_wdt_ms); s_wdt_started = true; }

    const uint32_t pass_c0 = hal::cycles();

    service_critical();

    for (size_t i = 0; i < s_n; i++) {
        Task& t = s_task[i];
        if (t.critical) continue;
        const uint32_t now = hal::micros();
        if ((int32_t)(now - t.next_us) < 0) continue;
        run_task(t, now);
        if (t.last_us > kYieldUs) service_critical();
    }

    if (s_wdt_started) {
        bool progressed=true;
        for(size_t i=0;i<s_n;++i) if(s_task[i].critical) {
            const auto& t=s_task[i];
            if(!t.runs || uint32_t(hal::micros()-t.completed_us)>20000) progressed=false;
        }
        if(progressed) hal::watchdog_kick();
    }

    const uint32_t pass_us = cyc_to_us(hal::cycles() - pass_c0);
    if (pass_us > s_worst_pass_us) s_worst_pass_us = pass_us;
    s_loops++;
}

void run()
{
    for (;;) run_once();
}

size_t task_count() { return s_n; }

bool get_stats(size_t i, TaskStats& o)
{
    if (i >= s_n) return false;
    const Task& t = s_task[i];
    o.name     = t.name;
    o.rate_hz  = t.rate_hz;
    o.critical = t.critical;
    o.runs     = t.runs;
    o.overruns = t.overruns;
    o.last_us  = t.last_us;
    o.min_us   = (t.min_us == 0xFFFFFFFFu) ? 0u : t.min_us;
    o.max_us   = t.max_us;
    o.mean_us  = t.runs ? (uint32_t)(t.sum_us / t.runs) : 0u;
    return true;
}

uint32_t loop_count()    { return s_loops; }
uint32_t worst_pass_us() { return s_worst_pass_us; }

void reset_stats()
{
    for (size_t i = 0; i < s_n; i++) {
        Task& t = s_task[i];
        t.runs = t.overruns = t.last_us = t.max_us = 0;
        t.min_us = 0xFFFFFFFFu;
        t.sum_us = 0;
    }
    s_worst_pass_us = 0;
    s_loops = 0;
}

} // namespace sched
