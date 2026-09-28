#include "sd_bin_log.hpp"
#include "sd_storage.hpp"
#include "log_sector.hpp"
#include <cstring>
namespace sd_bin_log {
namespace {
core::LogRing* ring=nullptr;
storage::Extent extent;
alignas(4) core::LogSector sector;
char filename[16]{};
bool good=false,exhausted=false,pending=false,header=true,tail=false,started=false;
uint32_t sequence=0,bytes=0,last_commit=0;
}
Result begin(core::LogRing& r) {
    if(started) return Result::open_failed;
    started=true;
    if(!storage::prepare(extent,filename)) return Result::begin_failed;
    if(!extent.sectors||!extent.session) return Result::open_failed;
    ring=&r; good=true; last_commit=storage::now_ms(); return Result::ok;
}
bool ok() { return good; }
bool full() { return exhausted; }
const char* name() { return filename; }
uint32_t bytes_written() { return bytes; }
uint32_t sectors_written() { return sequence; }
void sync() { tail=true; }
uint32_t flush_step(uint32_t budget) {
    if(!good||budget<512) return 0;
    if(pending) {
        const auto p=storage::poll_sector();
        if(p==storage::Progress::error) { good=false; return 0; }
        if(p!=storage::Progress::complete) return 0;
        const uint32_t n=sector.used;
        if(!header) ring->consume(n);
        header=false; pending=false; ++sequence; bytes+=n; last_commit=storage::now_ms();
        if(sequence==extent.sectors) { exhausted=true; good=false; }
        return n;
    }
    if(!header&&ring->used()<sizeof(sector.payload)&&!tail&&
       uint32_t(storage::now_ms()-last_commit)<100) return 0;
    if(!header&&!ring->used()) { tail=false; return 0; }
    sector={}; sector.session=extent.session; sector.sequence=sequence;
    if(header) {
        core::LogFileHeader h; core::log_file_header_init(h);
        std::memcpy(sector.payload,&h,sizeof(h)); sector.used=sizeof(h);
    } else sector.used=uint16_t(ring->peek(sector.payload,sizeof(sector.payload)));
    core::finalize(sector);
    if(!storage::start_sector(extent.first_sector+sequence,reinterpret_cast<const uint8_t*>(&sector))) {
        good=false; return 0;
    }
    pending=true; tail=false; return 0;
}
}
