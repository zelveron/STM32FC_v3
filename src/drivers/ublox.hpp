#pragma once
#include <cstdint>
// SAM-M10Q factory NMEA profile. Passive baud detection, bounded parsing,
// checksums and independent position/speed freshness. No receiver reconfiguration.
namespace ublox {
enum Event : uint8_t { EV_NONE=0, EV_GGA=1, EV_FIX=2 };
void begin(uint32_t baud=38400);
void drain_rx();
uint8_t poll();
int sats();
int fix();
const char* time_str();
float speed_kmh();
float course_deg();
bool speed_valid();
double lat_deg();
double lon_deg();
float alt_m();
uint32_t rx_bytes();
uint32_t fix_sequence(); // increments on a newly timed valid GGA fix
uint32_t current_baud();
bool locked();
bool nmea_valid();
bool receiving(); // Recent UART bytes, including malformed/non-NMEA traffic.
int satellites_used(); // Fresh GGA count even without a fix; -1 if unknown.
int satellites_visible(); // Fresh GSV: -1 unknown, 0 none reported, 1 in view.
uint32_t valid_messages();
const char* last_nmea();
uint16_t boot_capture(const uint8_t*& data);
}
