#pragma once
#include <cstdint>
#include "log_ring.hpp"
namespace sd_bin_log {
enum class Result { ok, begin_failed, open_failed };
// Boot-only FAT32 preallocation. See docs/LOGGING.md for recovery/file format.
Result begin(core::LogRing& ring);
bool ok();
bool full();
const char* name();
uint32_t bytes_written();
uint32_t sectors_written();
// One bounded state-machine step; never waits for a card or filesystem.
// Returns newly committed payload bytes, NOT requested transfer bytes.
uint32_t flush_step(uint32_t budget_bytes=512);
// Request partial-tail drain. This is asynchronous; it is NOT fsync.
void sync();
}
