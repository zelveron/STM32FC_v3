#include "sd_bin_log.hpp"
#include "log_frame.hpp"

// STM32SD / FatFs, Arduino -- excluded from [env:native].
#include <Arduino.h>
#include <STM32SD.h>

namespace sd_bin_log {
namespace {

File           s_file;
bool           s_ok = false;
char           s_name[16] = "";
uint32_t       s_bytes = 0;
core::LogRing*  s_ring = nullptr;

} // namespace

Result begin(core::LogRing& ring)
{
    s_ring  = &ring;
    s_ok    = false;
    s_bytes = 0;

    if (!SD.begin()) return Result::begin_failed;

    int n = 0;
    do { snprintf(s_name, sizeof(s_name), "FLT%05d.BIN", n++); }
    while (SD.exists(s_name));

    s_file = SD.open(s_name, FILE_WRITE);
    if (!s_file) return Result::open_failed;

    core::LogFileHeader h;
    core::log_file_header_init(h);
    s_file.write(reinterpret_cast<const uint8_t*>(&h), sizeof(h));
    s_file.flush();

    s_ok = true;
    return Result::ok;
}

bool        ok()            { return s_ok; }
const char* name()          { return s_name; }
uint32_t    bytes_written() { return s_bytes; }

uint32_t flush_step(uint32_t budget_bytes)
{
    if (!s_ok || s_ring == nullptr) return 0;

    uint32_t written = 0;
    uint8_t  sector[512];
    while (s_ring->used() >= sizeof(sector) && written < budget_bytes) {
        s_ring->peek(sector, sizeof(sector));
        if (s_file.write(sector, sizeof(sector)) != sizeof(sector)) {
            s_ok = false;                 // card fault -- stop, keep the ring
            break;
        }
        s_ring->consume(sizeof(sector));
        written  += sizeof(sector);
        s_bytes  += sizeof(sector);
    }
    return written;
}

void sync()
{
    if (s_ok) s_file.flush();
}

} // namespace sd_bin_log
