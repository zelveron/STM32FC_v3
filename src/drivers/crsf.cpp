#include "crsf.hpp"
#include "../hal/hal.hpp"

#include <cmath>
#include <cstring>

namespace crsf {
namespace {

constexpr hal::Uart kPort       = hal::Uart::crsf;
constexpr uint8_t   kAddrFC     = 0xC8;   // sync byte: RX->FC frames, and FC->handset telemetry
constexpr uint8_t   kTypeRC     = 0x16;   // RC_CHANNELS_PACKED
constexpr uint8_t   kTypeLink   = 0x14;   // LINK_STATISTICS
constexpr uint8_t   kMaxFrame   = 64;

// telemetry frame types (FC -> handset)
constexpr uint8_t   kTypeGps    = 0x02;
constexpr uint8_t   kTypeVario  = 0x07;
constexpr uint8_t   kTypeBatt   = 0x08;
constexpr uint8_t   kTypeAtt    = 0x1E;
constexpr uint8_t   kTypeFlight = 0x21;

uint32_t s_tx_frames = 0;

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
uint32_t s_last_rc_ms = 0;
uint32_t s_last_byte_ms = 0;
bool s_have_rc = false;

// diagnostic ring of the most recent raw bytes
uint8_t  s_raw[32];
uint8_t  s_raw_head = 0;   // next write index
uint8_t  s_raw_fill = 0;

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

// --- telemetry frame assembly (big-endian payloads) ---
void be16(uint8_t* d, uint16_t v) { d[0] = (uint8_t)(v >> 8); d[1] = (uint8_t)v; }
void be32(uint8_t* d, uint32_t v)
{
    d[0] = (uint8_t)(v >> 24); d[1] = (uint8_t)(v >> 16);
    d[2] = (uint8_t)(v >> 8);  d[3] = (uint8_t)v;
}
int16_t clamp_i16(long v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

// Frame = [addr][len][type][payload...][crc]; len counts type + payload + crc.
// Written whole or not at all (drop, don't half-send and desync the RX).
bool emit(uint8_t type, const uint8_t* payload, uint8_t pl_len)
{
    const uint8_t total = (uint8_t)(pl_len + 4);       // addr + len + type + payload + crc
    if (hal::uart_write_space(kPort) < total) return false;

    uint8_t f[kMaxFrame];
    f[0] = kAddrFC;
    f[1] = (uint8_t)(pl_len + 2);
    f[2] = type;
    for (uint8_t i = 0; i < pl_len; i++) f[3 + i] = payload[i];
    f[3 + pl_len] = crc8_dvbs2(&f[2], (uint8_t)(pl_len + 1));   // over type + payload

    if (hal::uart_write(kPort, f, total) != total) return false;
    s_tx_frames++;
    return true;
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
    s_last_type  = type;

    if (type == kTypeRC && pl_len >= 22) {
        unpack_rc(pl);
        s_last_rc_ms = hal::millis();
        s_have_rc = true;
        return EV_RC;
    }
    if (type == kTypeLink && pl_len >= 10) { parse_link(pl); return EV_LINK; }
    return EV_OTHER;
}

} // namespace

void begin(uint32_t baud)
{
    hal::uart_config(kPort, baud);
    s_len = 0;
    s_need = 0;
    s_have_rc = false;
    s_last_rc_ms = s_last_byte_ms = 0;
    s_ch = Channels{};
    for (auto& us : s_ch.us) us = 1500;
    s_ch.us[2] = s_ch.us[4] = 1000;
    s_link = LinkStats{};
    s_frames_ok = s_crc_err = s_resync = s_bytes = s_tx_frames = 0;
    s_raw_head = s_raw_fill = s_last_type = 0;
}

uint8_t poll()
{
    uint8_t ev = EV_NONE;
    if (s_len && (uint32_t)(hal::millis() - s_last_byte_ms) > 20) {
        s_len = s_need = 0;
        ++s_resync;
    }

    uint8_t in[96];
    size_t  n;
    // Bound scheduler occupancy even if bytes arrive continuously.
    for (unsigned batch = 0; batch < 4 &&
         (n = hal::uart_read(kPort, in, sizeof(in))) > 0; ++batch) {
        s_last_byte_ms = hal::millis();
        s_bytes += n;
        for (size_t i = 0; i < n; i++) {
            const uint8_t c = in[i];

            s_raw[s_raw_head] = c;
            s_raw_head = (uint8_t)((s_raw_head + 1) % sizeof(s_raw));
            if (s_raw_fill < sizeof(s_raw)) s_raw_fill++;

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

bool     receiving()   { return s_have_rc && (uint32_t)(hal::millis() - s_last_rc_ms) < 200; }
uint32_t frames_ok()   { return s_frames_ok; }
uint32_t crc_errors()  { return s_crc_err; }
uint32_t resyncs()     { return s_resync; }
uint32_t bytes_rx()    { return s_bytes; }

// --- telemetry TX --------------------------------------------------------------

bool send_attitude(float pitch_rad, float roll_rad, float yaw_rad)
{
    if (!std::isfinite(pitch_rad) || !std::isfinite(roll_rad) || !std::isfinite(yaw_rad)) return false;
    uint8_t p[6];
    be16(&p[0], (uint16_t)clamp_i16(std::lround(pitch_rad * 10000.0f)));
    be16(&p[2], (uint16_t)clamp_i16(std::lround(roll_rad  * 10000.0f)));
    be16(&p[4], (uint16_t)clamp_i16(std::lround(yaw_rad   * 10000.0f)));
    return emit(kTypeAtt, p, sizeof(p));
}

bool send_vario(float climb_mps)
{
    if (!std::isfinite(climb_mps)) return false;
    uint8_t p[2];
    be16(&p[0], (uint16_t)clamp_i16(std::lround(climb_mps * 100.0f)));   // cm/s
    return emit(kTypeVario, p, sizeof(p));
}

bool send_gps(const GpsTelem& g)
{
    uint8_t p[15];
    be32(&p[0], (uint32_t)g.lat_1e7);
    be32(&p[4], (uint32_t)g.lon_1e7);
    long spd = std::lround(g.ground_mps * 36.0f);                 // km/h * 10
    long hdg = std::lround(g.heading_deg * 100.0f);               // deg * 100
    long alt = std::lround(g.altitude_m) + 1000;                  // m + 1000 offset
    if (spd < 0) spd = 0;   if (spd > 65535) spd = 65535;
    hdg %= 36000; if (hdg < 0) hdg += 36000;
    if (alt < 0) alt = 0;   if (alt > 65535) alt = 65535;
    be16(&p[8],  (uint16_t)spd);
    be16(&p[10], (uint16_t)hdg);
    be16(&p[12], (uint16_t)alt);
    p[14] = g.sats;
    return emit(kTypeGps, p, sizeof(p));
}

bool send_battery(float volts, float amps, uint32_t mah_used, uint8_t pct)
{
    uint8_t p[8];
    long dv = std::lround(volts * 10.0f);   if (dv < 0) dv = 0; if (dv > 65535) dv = 65535;
    long da = std::lround(amps  * 10.0f);   if (da < 0) da = 0; if (da > 65535) da = 65535;
    be16(&p[0], (uint16_t)dv);
    be16(&p[2], (uint16_t)da);
    p[4] = (uint8_t)(mah_used >> 16);
    p[5] = (uint8_t)(mah_used >> 8);
    p[6] = (uint8_t)(mah_used);
    p[7] = pct;
    return emit(kTypeBatt, p, sizeof(p));
}

bool send_flight_mode(const char* mode)
{
    uint8_t p[16];
    uint8_t n = 0;
    while (mode[n] && n < sizeof(p) - 1) { p[n] = (uint8_t)mode[n]; n++; }
    p[n++] = 0;                          // NUL terminator is part of the payload
    return emit(kTypeFlight, p, n);
}

uint32_t telem_frames_tx() { return s_tx_frames; }

size_t raw_sample(uint8_t* out, size_t max)
{
    const size_t n = (s_raw_fill < max) ? s_raw_fill : max;
    // oldest-first: start (s_raw_head - s_raw_fill) mod size
    uint8_t idx = (uint8_t)((s_raw_head + sizeof(s_raw) - s_raw_fill) % sizeof(s_raw));
    for (size_t i = 0; i < n; i++) {
        out[i] = s_raw[idx];
        idx = (uint8_t)((idx + 1) % sizeof(s_raw));
    }
    return n;
}

} // namespace crsf
