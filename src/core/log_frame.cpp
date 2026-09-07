#include "log_frame.hpp"
#include <cstddef>
#include <cstring>

namespace core {

uint16_t log_crc16(const void* data, size_t len)
{
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

void log_frame_finalize(LogFrame& f)
{
    f.magic = kLogMagic;
    f.crc   = log_crc16(&f, offsetof(LogFrame, crc));
}

bool log_frame_valid(const LogFrame& f)
{
    return f.magic == kLogMagic &&
           f.crc   == log_crc16(&f, offsetof(LogFrame, crc));
}

void log_file_header_init(LogFileHeader& h)
{
    std::memcpy(h.tag, "STFC", 4);
    h.version    = kLogVersion;
    h.frame_size = (uint8_t)sizeof(LogFrame);
    h.magic      = kLogMagic;
    h.acc_scale  = kLogAccScale;
    h.gyr_scale  = kLogGyrScale;
    h.ang_scale  = kLogAngScale;
}

} // namespace core
