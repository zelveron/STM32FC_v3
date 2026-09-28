#pragma once
//
// crsf.hpp -- CRSF (Crossfire / ExpressLRS) receiver link parser.
//
// UART4/J2 @ 420000 8N1, not inverted. Frame: [0xC8][len][type][payload][crc8],
// crc8 = CRC-8/DVB-S2 (poly 0xD5) over type+payload. Decodes RC_CHANNELS_PACKED
// (0x16) and LINK_STATISTICS (0x14); notes other valid frame types.
//
// Portable: hal only. poll() is non-blocking (drains the UART, feeds a byte
// state machine, resyncs on a bad length or CRC).
//
// Telemetry is scheduled by core/telemetry_schedule.hpp; DMA RX is not used.
//
#include <cstddef>
#include <cstdint>

namespace crsf {

struct Channels {
    uint16_t us[16];   // microseconds, ~988..2012, 1500 = centre
};

struct LinkStats {
    int8_t  up_rssi_dbm;   // uplink RSSI, negative dBm (antenna 1)
    int8_t  up_rssi2_dbm;  // antenna 2
    uint8_t up_lq;         // uplink link quality, 0..100 %
    int8_t  up_snr;        // uplink SNR, dB
    uint8_t active_ant;    // 0 / 1
    uint8_t rf_mode;       // ELRS packet-rate index
    uint8_t up_tx_power;   // TX power enum index
    int8_t  dn_rssi_dbm;   // downlink RSSI
    uint8_t dn_lq;         // downlink link quality
    int8_t  dn_snr;        // downlink SNR
};

enum Event : uint8_t {
    EV_NONE  = 0,
    EV_RC    = 1 << 0,   // fresh channels()
    EV_LINK  = 1 << 1,   // fresh link()
    EV_OTHER = 1 << 2,   // some other CRC-valid frame (see last_frame_type())
};

void    begin(uint32_t baud);   // configure Uart::crsf
uint8_t poll();                 // returns OR of Event; call every loop pass

const Channels&  channels();
const LinkStats& link();
uint8_t          last_frame_type();

// --- health ---
bool     receiving();       // a valid full RC-channel frame within the last 200 ms
uint32_t frames_ok();
uint32_t crc_errors();
uint32_t resyncs();
uint32_t bytes_rx();

// --- diagnostic: copy of the most recent raw RX bytes (up to 32) ---
size_t   raw_sample(uint8_t* out, size_t max);

// --- telemetry TX (FC -> handset, over the same UART the RX reads) ----------
//
// Each builder assembles a CRSF frame [0xC8][len][type][payload][crc8] with a
// big-endian payload (CRSF convention) and hands it to hal::uart_write. The
// frame is written whole or not at all (checked against the TX ring space), so
// a full ring drops the frame instead of corrupting the stream. Non-blocking.
// Returns true if the frame was queued.
//
// Call these from a low-rate task (~10 Hz); the ELRS link paces the actual
// over-air telemetry, the FC just keeps fresh frames available.

struct GpsTelem {
    int32_t lat_1e7      = 0;
    int32_t lon_1e7      = 0;
    float   ground_mps   = 0.0f;
    float   heading_deg  = 0.0f;   // course over ground; 0 if unknown
    float   altitude_m   = 0.0f;   // MSL (EdgeTX labels it "GAlt")
    uint8_t sats         = 0;
};

bool send_gps        (const GpsTelem& g);              // 0x02
bool send_vario      (float climb_mps);                // 0x07  (+ = up)
bool send_battery    (float volts, float amps,
                      uint32_t mah_used, uint8_t pct); // 0x08
bool send_attitude   (float pitch_rad, float roll_rad,
                      float yaw_rad);                  // 0x1E
bool send_flight_mode(const char* mode);               // 0x21  (ASCII, <=15 ch)

uint32_t telem_frames_tx();   // count of frames queued, for the debug line

} // namespace crsf
