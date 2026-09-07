#pragma once
//
// crsf.hpp -- CRSF (Crossfire / ExpressLRS) receiver link parser.
//
// USART3 @ 420000 8N1, not inverted. Frame: [0xC8][len][type][payload][crc8],
// crc8 = CRC-8/DVB-S2 (poly 0xD5) over type+payload. Decodes RC_CHANNELS_PACKED
// (0x16) and LINK_STATISTICS (0x14); notes other valid frame types.
//
// Portable: hal only. poll() is non-blocking (drains the UART, feeds a byte
// state machine, resyncs on a bad length or CRC).
//
// TODO: telemetry TX (battery / GPS / attitude uplink); DMA circular RX.
//
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
bool     receiving();       // a valid frame within the last 500 ms
uint32_t frames_ok();
uint32_t crc_errors();
uint32_t resyncs();
uint32_t bytes_rx();

} // namespace crsf
