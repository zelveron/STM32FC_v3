#pragma once
//
// log_frame.hpp -- the binary flight-log record.
//
// Fixed-size, packed. Starts with a magic word so an offline parser can
// resync on a corrupt stream; ends with a CRC-16/CCITT over the preceding
// bytes. Physical values are stored as scaled integers (no float formatting
// in the hot path). tools/parse_bin_log.py reads it.
//
// Portable.
//
#include <cstddef>
#include <cstdint>

namespace core {

constexpr uint16_t kLogMagic   = 0x5AA5;
constexpr uint8_t  kLogVersion = 1;

// stored = physical * scale
constexpr float kLogAccScale = 2048.0f;   // g   -> int16 (+/-16 g)
constexpr float kLogGyrScale = 16.0f;     // dps -> int16 (+/-2000 dps)
constexpr float kLogAngScale = 100.0f;    // deg -> int16 centidegrees

enum LogFlag : uint8_t {
    LOG_ARMED     = 1 << 0,
    LOG_FAILSAFE  = 1 << 1,
    // bits 2..3: requested mode (0 MANUAL, 1 ASSIST, 2 AUTO)
    LOG_REQMODE_SHIFT = 2,
};

#pragma pack(push, 1)
struct LogFrame {
    uint16_t magic;          // kLogMagic
    uint32_t t_ms;

    int16_t  acc[3];         // g   * kLogAccScale
    int16_t  gyr[3];         // dps * kLogGyrScale
    int16_t  att[3];         // roll/pitch/yaw deg * kLogAngScale

    float    press_pa;
    int16_t  temp_dC;        // deci-Celsius
    int32_t  alt_mm;

    int32_t  lat_1e7;
    int32_t  lon_1e7;
    int32_t  gps_alt_mm;
    uint16_t gps_speed_cms;
    uint8_t  gps_sats;
    uint8_t  gps_fix;

    uint16_t rc_us[8];
    uint16_t out_us[8];

    uint8_t  mode;           // active mode id
    uint8_t  flags;          // LogFlag bits
    uint16_t crc;            // CRC-16/CCITT over bytes [0, offsetof(crc))
};
#pragma pack(pop)

uint16_t log_crc16(const void* data, size_t len);      // CCITT, init 0xFFFF

// Fill magic + crc. Call after populating all other fields.
void log_frame_finalize(LogFrame& f);
bool log_frame_valid(const LogFrame& f);

// The file header written once at the top of each FLTxxxxx.BIN.
#pragma pack(push, 1)
struct LogFileHeader {
    char     tag[4];         // "STFC"
    uint8_t  version;        // kLogVersion
    uint8_t  frame_size;     // sizeof(LogFrame)
    uint16_t magic;          // kLogMagic
    float    acc_scale;
    float    gyr_scale;
    float    ang_scale;
};
#pragma pack(pop)

void log_file_header_init(LogFileHeader& h);

} // namespace core
