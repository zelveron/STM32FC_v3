#include "crsf.hpp"
#include "../hal/hal.hpp"

#include <cstring>

namespace crsf {
namespace {

constexpr hal::Uart kPort       = hal::Uart::crsf;
constexpr uint8_t   kAddrFC     = 0xC8;   // frames from the RX are addressed to the FC
constexpr uint8_t   kTypeRC     = 0x16;   // RC_CHANNELS_PACKED
constexpr uint8_t   kTypeLink   = 0x14;   // LINK_STATISTICS
constexpr uint8_t   kMaxFrame   = 64;

// --- parser state ---
// s_len == 0 : hunting for the sync byte
// s_len == 1 : have sync, next byte is the length field
// s_len >= 2 : collecting the body until s_len == s_need
uint8_t  s_buf[kMaxFrame];
uint8_t  s_len  = 0;
uint8_t  s_need = 0;      // total frame bytes once known

Channels  s_ch{};
LinkStats s_link{};
uint8_t   s_last_type = 0;

uint32_t s_frames_ok = 0;
uint32_t s_crc_err   = 0;
uint32_t s_resync    = 0;
uint32_t s_bytes     = 0;
uint32_t s_last_ok_ms = 0;

uint8_t crc8_dvbs2(const uint8_t* p, uint8_t n)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

void unpack_rc(const uint8_t* p /* 22 bytes */)
{
    uint32_t bits = 0;
    int      nbits = 0;
    int      ch = 0;
    for (int i = 0; i < 22 && ch < 16; i++) {
        bits |= (uint32_t)p[i] << nbits;
        nbits += 8;
        while (nbits >= 11 && ch < 16) {
            const int raw = (int)(bits & 0x7FF);
            bits >>= 11;
            nbits -= 11;
            int us = ((raw - 992) * 5) / 8 + 1500;   // 172->988, 992->1500, 1811->2012
            if (us < 800)  us = 800;
            if (us > 2200) us = 2200;
            s_ch.us[ch++] = (uint16_t)us;
        }
    }
}

void parse_link(const uint8_t* p /* 10 bytes */)
{
    s_link.up_rssi_dbm  = -(int8_t)p[0];
    s_link.up_rssi2_dbm = -(int8_t)p[1];
    s_link.up_lq        = p[2];
    s_link.up_snr       = (int8_t)p[3];
    s_link.active_ant   = p[4];
    s_link.rf_mode      = p[5];
    s_link.up_tx_power  = p[6];
    s_link.dn_rssi_dbm  = -(int8_t)p[7];
    s_link.dn_lq        = p[8];
    s_link.dn_snr       = (int8_t)p[9];
}

// s_buf holds a full frame: [addr][len][type][payload...][crc]
uint8_t dispatch()
{
    const uint8_t len     = s_buf[1];
    const uint8_t type    = s_buf[2];
    const uint8_t* pl     = &s_buf[3];
    const uint8_t pl_len  = (uint8_t)(len - 2);          // type + payload + crc = len; payload = len-2
    const uint8_t crc_rx  = s_buf[2 + (len - 1)];        // last byte of the frame
    const uint8_t crc_calc = crc8_dvbs2(&s_buf[2], (uint8_t)(len - 1));  // over type + payload

    if (crc_rx != crc_calc) { s_crc_err++; return EV_NONE; }

    s_frames_ok++;
    s_last_ok_ms = hal::millis();
    s_last_type  = type;

    if (type == kTypeRC && pl_len >= 22) { unpack_rc(pl);  return EV_RC; }
    if (type == kTypeLink && pl_len >= 10) { parse_link(pl); return EV_LINK; }
    return EV_OTHER;
}

} // namespace

void begin(uint32_t baud)
{
    hal::uart_config(kPort, baud);
    s_len = 0;
    s_need = 0;
}

uint8_t poll()
{
    uint8_t ev = EV_NONE;

    uint8_t in[96];
    size_t  n;
    while ((n = hal::uart_read(kPort, in, sizeof(in))) > 0) {
        s_bytes += n;
        for (size_t i = 0; i < n; i++) {
            const uint8_t c = in[i];

            if (s_len == 0) {                         // hunting for sync
                if (c == kAddrFC) { s_buf[0] = c; s_len = 1; }
                continue;
            }
            if (s_len == 1) {                         // length field
                if (c < 2 || c > (kMaxFrame - 2)) {   // invalid -> resync
                    s_resync++;
                    if (c == kAddrFC) { s_buf[0] = c; s_len = 1; }
                    else              { s_len = 0; }
                    continue;
                }
                s_buf[1] = c;
                s_need = (uint8_t)(c + 2);            // addr + len byte + c more
                s_len  = 2;
                continue;
            }
            s_buf[s_len++] = c;                       // body
            if (s_len >= s_need) {
                ev |= dispatch();
                s_len  = 0;
                s_need = 0;
            }
        }
    }
    return ev;
}

const Channels&  channels()        { return s_ch; }
const LinkStats& link()            { return s_link; }
uint8_t          last_frame_type() { return s_last_type; }

bool     receiving()   { return s_frames_ok && (hal::millis() - s_last_ok_ms) < 500; }
uint32_t frames_ok()   { return s_frames_ok; }
uint32_t crc_errors()  { return s_crc_err; }
uint32_t resyncs()     { return s_resync; }
uint32_t bytes_rx()    { return s_bytes; }

} // namespace crsf
