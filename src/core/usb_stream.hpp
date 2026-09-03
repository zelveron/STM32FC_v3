#pragma once
//
// usb_stream.hpp -- non-blocking USB CDC line logger.  BENCH SCAFFOLDING.
//
// *** This module deliberately sits OUTSIDE the hal boundary. ***  It uses the
// Arduino `SerialUSB` / `Print` classes directly, so it does NOT compile for
// [env:native] and is excluded there. That is acceptable because the entire
// USB telemetry path is temporary: there is no ST-Link on this project, so USB
// CDC is the only bring-up debug channel, and it is removed once the
// restructure is validated (the flying aircraft has no USB connection; flight
// recording is SD-only). See CLAUDE.md "Telemetry / logging strategy".
//
// The stock USBSerial::write() busy-waits up to USB_CDC_TRANSMIT_TIMEOUT (3 ms)
// per call on a connected-but-stalled host and can emit half lines. log()
// returns a Print that buffers one line and pushes it only if it fits the CDC
// TX queue whole; otherwise the line is dropped and counted. Never blocks,
// never emits a partial line.
//
#include <Arduino.h>
#include <cstdint>

namespace usb_stream {

void      begin();          // SerialUSB.begin()
bool      host_ready();     // true once the USB host has opened the port
Print&    log();            // the drop-on-full line logger
uint32_t  drops();          // cumulative whole lines dropped (host too slow)

} // namespace usb_stream
