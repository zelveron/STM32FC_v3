#include "ublox.hpp"
#include "../hal/hal.hpp"

#include <cstdlib>
#include <cstring>

namespace ublox {
namespace {

constexpr hal::Uart kPort = hal::Uart::gps;

const uint32_t kBauds[] = {
    9600, 38400, 115200, 4800, 57600, 19200, 230400, 460800, 921600
};
constexpr uint8_t kBaudCount = sizeof(kBauds) / sizeof(kBauds[0]);

uint8_t  s_baud_idx      = 0;
uint32_t s_last_switch_ms = 0;
uint32_t s_last_ubx_ms   = 0;
uint32_t s_rx_bytes      = 0;
bool     s_locked        = false;
bool     s_nmea_valid    = false;

uint8_t  s_boot[160];
uint16_t s_boot_len = 0;

int   s_sats      = 0;
int   s_fix       = 0;
char  s_time[12]  = "";
float s_speed_kmh = 0.0f;
float s_lat = 0.0f, s_lon = 0.0f, s_alt = 0.0f;

char    s_line[128];
uint8_t s_line_len = 0;
char    s_last_nmea[96] = "";

// --- UBX (u-blox binary) helpers -------------------------------------------
void ubx_send(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len)
{
    uint8_t f[8 + 32];
    if (len > 32) return;

    uint8_t a = 0, b = 0;
    size_t  k = 0;
    f[k++] = 0xB5;
    f[k++] = 0x62;
    f[k++] = cls;                a += cls;                       b += a;
    f[k++] = id;                 a += id;                        b += a;
    f[k++] = (uint8_t)(len & 0xFF);        a += (uint8_t)(len & 0xFF);        b += a;
    f[k++] = (uint8_t)((len >> 8) & 0xFF); a += (uint8_t)((len >> 8) & 0xFF); b += a;
    for (uint16_t i = 0; i < len; i++) { f[k++] = payload[i]; a += payload[i]; b += a; }
    f[k++] = a;
    f[k++] = b;

    hal::uart_write(kPort, f, k);
}

// UBX-MON-VER poll: any u-blox module answers regardless of output config.
void ubx_poll_monver() { ubx_send(0x0A, 0x04, nullptr, 0); }

// UBX-CFG-PRT: UART1 -> NMEA + UBX out at 9600 8N1.
void ubx_enable_nmea()
{
    static const uint8_t prt[20] = {
        0x01, 0x00, 0x00, 0x00,       // portID = UART1, reserved, txReady
        0xD0, 0x08, 0x00, 0x00,       // mode: 8N1
        0x80, 0x25, 0x00, 0x00,       // baudRate: 9600
        0x03, 0x00,                   // inProtoMask: UBX | NMEA
        0x03, 0x00,                   // outProtoMask: UBX | NMEA
        0x00, 0x00, 0x00, 0x00        // flags, reserved
    };
    ubx_send(0x06, 0x00, prt, sizeof(prt));
}

// --- NMEA parsing (moved verbatim from the streamer) ----------------------
int split(char* s, char* f[], int max)
{
    int n = 0;
    f[n++] = s;
    for (char* p = s; *p && n < max; p++) {
        if (*p == ',') { *p = '\0'; f[n++] = p + 1; }
    }
    return n;
}

bool valid_nmea(const char* line)
{
    int len = (int)std::strlen(line);
    if (line[0] != '$' || len < 8 || line[len - 3] != '*') return false;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)std::strtol(line + len - 2, nullptr, 16);
    return cs == cs_hex;
}

void store_time(const char* nmea_time)
{
    if (nmea_time[0] == '\0' || std::strlen(nmea_time) < 6) return;
    s_time[0] = nmea_time[0];
    s_time[1] = nmea_time[1];
    s_time[2] = ':';
    s_time[3] = nmea_time[2];
    s_time[4] = nmea_time[3];
    s_time[5] = ':';
    s_time[6] = nmea_time[4];
    s_time[7] = nmea_time[5];
    s_time[8] = '\0';
}

// $GxRMC: UTC time + speed over ground (knots -> km/h).
void process_rmc(char* line)
{
    int len = (int)std::strlen(line);
    if (line[0] != '$' || line[3] != 'R' || line[4] != 'M' || line[5] != 'C') return;
    if (len < 10 || line[len - 3] != '*') return;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)std::strtol(line + len - 2, nullptr, 16);
    if (cs != cs_hex) return;

    line[len - 3] = '\0';
    char* f[13];
    int n = split(line, f, 13);
    if (n < 8) return;

    store_time(f[1]);
    if (f[7][0] != '\0') s_speed_kmh = (float)std::atof(f[7]) * 1.852f;   // knots -> km/h
}

// $GxGGA: position / sats / fix / time. Returns the events raised.
uint8_t process_gga(char* line)
{
    int len = (int)std::strlen(line);
    if (line[0] != '$' || line[3] != 'G' || line[4] != 'G' || line[5] != 'A') return EV_NONE;
    if (len < 10 || line[len - 3] != '*') return EV_NONE;
    uint8_t cs = 0;
    for (int i = 1; i < len - 3; i++) cs ^= (uint8_t)line[i];
    uint8_t cs_hex = (uint8_t)std::strtol(line + len - 2, nullptr, 16);
    if (cs != cs_hex) return EV_NONE;

    line[len - 3] = '\0';
    char* f[15];
    int n = split(line, f, 15);
    if (n < 11) return EV_NONE;   // need fields through altitude unit

    s_sats = std::atoi(f[7]);
    s_fix  = std::atoi(f[6]);
    store_time(f[1]);

    uint8_t ev = EV_GGA;
    if (f[2][0] == '\0' || f[4][0] == '\0') return ev;

    float lat_raw = (float)std::atof(f[2]);
    float lat = (int)(lat_raw / 100.0f) + (lat_raw - (int)(lat_raw / 100.0f) * 100.0f) / 60.0f;
    if (f[3][0] == 'S') lat = -lat;

    float lon_raw = (float)std::atof(f[4]);
    float lon = (int)(lon_raw / 100.0f) + (lon_raw - (int)(lon_raw / 100.0f) * 100.0f) / 60.0f;
    if (f[5][0] == 'W') lon = -lon;

    s_lat    = lat;
    s_lon    = lon;
    s_alt    = (float)std::atof(f[9]);
    s_locked = true;
    return ev | EV_FIX;
}

void capture_byte(uint8_t c)
{
    s_rx_bytes++;
    if (s_boot_len < sizeof(s_boot)) s_boot[s_boot_len++] = c;
}

} // namespace

void begin(uint32_t baud)
{
    hal::uart_config(kPort, baud);
}

void drain_rx()
{
    uint8_t buf[64];
    size_t  n;
    while ((n = hal::uart_read(kPort, buf, sizeof(buf))) > 0)
        for (size_t i = 0; i < n; i++) capture_byte(buf[i]);
}

uint8_t poll()
{
    uint8_t ev = EV_NONE;

    uint8_t buf[128];
    size_t  n;
    while ((n = hal::uart_read(kPort, buf, sizeof(buf))) > 0) {
        for (size_t i = 0; i < n; i++) {
            char c = (char)buf[i];
            capture_byte((uint8_t)c);

            if (c == '\n') {
                s_line[s_line_len] = '\0';
                if (valid_nmea(s_line)) {
                    s_nmea_valid = true;
                    std::strncpy(s_last_nmea, s_line, sizeof(s_last_nmea) - 1);
                    s_last_nmea[sizeof(s_last_nmea) - 1] = '\0';
                }
                ev |= process_gga(s_line);
                process_rmc(s_line);
                s_line_len = 0;
            } else if (c != '\r' && s_line_len < (sizeof(s_line) - 1)) {
                s_line[s_line_len++] = c;
            }
        }
    }

    // NMEA baud auto-detect: cycle until a checksum-valid sentence appears.
    if (!s_nmea_valid && (hal::millis() - s_last_switch_ms) >= 2000) {
        s_last_switch_ms = hal::millis();
        s_baud_idx = (uint8_t)((s_baud_idx + 1) % kBaudCount);
        begin(kBauds[s_baud_idx]);
    }

    // Active probe: poll version + (re)enable NMEA every ~2 s.
    if ((hal::millis() - s_last_ubx_ms) >= 2000) {
        s_last_ubx_ms = hal::millis();
        ubx_poll_monver();
        ubx_enable_nmea();
    }

    return ev;
}

int         sats()       { return s_sats; }
int         fix()        { return s_fix; }
const char* time_str()   { return s_time; }
float       speed_kmh()  { return s_speed_kmh; }
float       lat_deg()    { return s_lat; }
float       lon_deg()    { return s_lon; }
float       alt_m()      { return s_alt; }

uint32_t    rx_bytes()     { return s_rx_bytes; }
uint32_t    current_baud() { return kBauds[s_baud_idx]; }
bool        locked()       { return s_locked; }
bool        nmea_valid()   { return s_nmea_valid; }
const char* last_nmea()    { return s_last_nmea; }

uint16_t boot_capture(const uint8_t*& data)
{
    data = s_boot;
    return s_boot_len;
}

} // namespace ublox
