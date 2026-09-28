#pragma once
#include <cstdint>
#include <cstring>
#include "log_frame.hpp"
namespace core {
#pragma pack(push,1)
struct LogSector {
    char tag[4]; // FCS2; versioned sector envelope, legacy STFC stream inside
    uint64_t session;
    uint32_t sequence;
    uint16_t used;
    uint8_t payload[492];
    uint16_t crc;
};
#pragma pack(pop)
static_assert(sizeof(LogSector)==512,"SD sector layout");
inline void finalize(LogSector& s) {
    std::memcpy(s.tag,"FCS2",4); s.crc=log_crc16(&s,510);
}
inline bool valid(const LogSector& s) {
    return !std::memcmp(s.tag,"FCS2",4)&&s.used<=492&&s.crc==log_crc16(&s,510);
}
}
