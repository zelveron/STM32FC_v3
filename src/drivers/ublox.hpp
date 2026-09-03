#pragma once
//
// ublox.hpp -- u-blox NEO-7M GNSS over USART1 (crossed wiring: MCU TX = PA9,
// MCU RX = PA10). NMEA in, with active UBX probing + NMEA-baud auto-detect.
//
// Bench-era behaviour, moved verbatim: parses $GxGGA (position / sats / fix /
// time) and $GxRMC (time / speed), cycles 9 candidate bauds until a
// checksum-valid sentence appears, and pokes UBX-MON-VER + UBX-CFG-PRT every
// ~2 s. The Phase 1 driver replaces this with a UBX-NAV-PVT binary parser.
//
// Depends on hal:: and libc only. No Arduino / STM32.
//
#include <cstdint>

namespace ublox {

enum Event : uint8_t {
    EV_NONE = 0,
    EV_GGA  = 1 << 0,   // a checksum-valid GGA was parsed (sats/fix/time updated)
    EV_FIX  = 1 << 1,   // that GGA carried a usable lat/lon (position updated)
};

void begin(uint32_t baud);   // (re)start the UART at this baud

// Accumulate RX bytes into the rx_bytes count and the boot-capture buffer
// without parsing. Used during the bounded boot wait.
void drain_rx();

// Full service: assemble lines, parse, run auto-baud + UBX poll timers.
// Returns the OR of Events seen this call. Call every loop iteration.
uint8_t poll();

// --- parsed state ---
int         sats();
int         fix();
const char* time_str();     // "HH:MM:SS", "" until known
float       speed_kmh();
float       lat_deg();
float       lon_deg();
float       alt_m();

// --- bench / debug ---
uint32_t    rx_bytes();
uint32_t    current_baud();
bool        locked();       // sticky: true after the first position fix
bool        nmea_valid();   // true after the first checksum-valid sentence
const char* last_nmea();
uint16_t    boot_capture(const uint8_t*& data);   // returns length, sets data

} // namespace ublox
