#pragma once
#include <cstdint>
#include "sd_async_card.hpp"
namespace storage {
struct Extent { uint32_t first_sector=0,sectors=0; uint64_t session=0; };
struct Diagnostics {
    const char* stage="not_started";
    uint32_t card_sectors=0;
    uint32_t hw_error=0;
    uint32_t response=0,dma_status=0;
    uint32_t dma_remaining=0,data_control=0;
    uint32_t fifo_warning_transfers=0; // transfers with FEIF, not failed writes
    uint8_t command=0;
    int fatfs=0;
    uint8_t filesystem=0;
};
const Diagnostics& diagnostics();
// Boot only: allocate and close a new contiguous FAT32 file. All FAT metadata
// is committed before raw asynchronous writes begin. No runtime filesystem IO.
bool prepare(Extent& extent,char (&name)[16]);
bool start_sector(uint32_t lba,const uint8_t* aligned512);
Progress poll_sector();
uint32_t now_ms();
}
