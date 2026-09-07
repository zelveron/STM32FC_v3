#pragma once
//
// sd_bin_log.hpp -- binary flight log to SD (FLTxxxxx.BIN).  BENCH-adjacent:
// drives the vendored Arduino STM32SD / FatFs stack directly (README known
// issue #2), so it is STM32-only and excluded from [env:native]. Replaces the
// CSV logger.
//
// The producers push core::LogFrame records into a LogRing (fast, non-blocking).
// flush_step() -- called from its own low-rate task -- drains 512-byte-aligned
// chunks to the card. That is where the SD write latency lives, off the
// control loop. sync() (f_sync -> FAT/dir update) is called sparingly.
//
#include <cstdint>
#include "log_ring.hpp"

namespace sd_bin_log {

enum class Result { ok, begin_failed, open_failed };

// Mount the card, open the next free FLTxxxxx.BIN, write the file header, and
// remember the ring to drain.
Result begin(core::LogRing& ring);

bool        ok();
const char* name();
uint32_t    bytes_written();

// Write whole 512-byte sectors from the ring, up to budget_bytes. Returns the
// number of bytes written this call. No f_sync here.
uint32_t flush_step(uint32_t budget_bytes = 4096);

// f_sync: commit the FAT + directory entry. ~10-70 ms, card dependent. Call
// every few seconds and on disarm, never from the control loop.
void sync();

} // namespace sd_bin_log
