#pragma once
#include <cstdint>
#include "sd_async_card.hpp"
namespace storage {
struct Extent { uint32_t first_sector=0,sectors=0; uint64_t session=0; };
// Boot only: allocate and close a new contiguous FAT32 file. All FAT metadata
// is committed before raw asynchronous writes begin. No runtime filesystem IO.
bool prepare(Extent& extent,char (&name)[16]);
bool start_sector(uint32_t lba,const uint8_t* aligned512);
Progress poll_sector();
uint32_t now_ms();
}
