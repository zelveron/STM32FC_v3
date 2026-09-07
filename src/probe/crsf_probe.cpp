//
// crsf_probe.cpp -- read the ER8 (ExpressLRS) CRSF link and stream it to USB.
//
// NOT flight firmware. Built by [env:crsf_probe]. USART3: PB11 <- RX TX,
// PB10 -> RX RX, 420000 8N1 not inverted. Verifies live sticks + link stats
// before the real drivers/crsf (DMA circular RX + telemetry TX) is wired in.
//
#include <Arduino.h>

#include "hal/hal.hpp"
#include "drivers/crsf.hpp"
#include "core/usb_stream.hpp"

namespace { Print* L = nullptr; }

void setup()
{
    hal::init();
    usb_stream::begin();
    for (int i = 0; i < 400 && !usb_stream::host_ready(); i++) delay(10);
    delay(200);
    L = &usb_stream::log();

    L->println(F("=== CRSF PROBE -- USART3 PB11(rx)/PB10(tx) @ 420000 8N1 ==="));
    L->println(F("power the ER8 + TX16S, bind if needed; RC = live sticks, LINK = radio link"));
    crsf::begin(420000);
}

void loop()
{
    const uint8_t ev = crsf::poll();

    static uint32_t last_rc = 0;
    if ((ev & crsf::EV_RC) && (millis() - last_rc) >= 50) {        // 20 Hz
        last_rc = millis();
        const crsf::Channels& ch = crsf::channels();
        L->print(F("RC"));
        for (int i = 0; i < 16; i++) { L->print(','); L->print(ch.us[i]); }
        L->println();
    }

    static uint32_t last_link = 0;
    if ((ev & crsf::EV_LINK) && (millis() - last_link) >= 200) {
        last_link = millis();
        const crsf::LinkStats& lk = crsf::link();
        L->print(F("LINK,up_rssi_dbm=")); L->print(lk.up_rssi_dbm);
        L->print(F(",up_lq="));           L->print(lk.up_lq);
        L->print(F(",up_snr="));          L->print(lk.up_snr);
        L->print(F(",rf_mode="));         L->print(lk.rf_mode);
        L->print(F(",tx_pwr_idx="));      L->print(lk.up_tx_power);
        L->print(F(",dn_rssi_dbm="));     L->print(lk.dn_rssi_dbm);
        L->print(F(",dn_lq="));           L->print(lk.dn_lq);
        L->println();
    }

    static uint32_t last_other = 0;
    if ((ev & crsf::EV_OTHER) && (millis() - last_other) >= 500) {
        last_other = millis();
        L->print(F("FRAME,type=0x")); L->println(crsf::last_frame_type(), HEX);
    }

    static uint32_t last_stat = 0;
    if ((millis() - last_stat) >= 1000) {
        last_stat = millis();
        L->print(F("CRSF_STAT,receiving=")); L->print(crsf::receiving() ? 1 : 0);
        L->print(F(",frames_ok="));          L->print(crsf::frames_ok());
        L->print(F(",crc_err="));            L->print(crsf::crc_errors());
        L->print(F(",resync="));             L->print(crsf::resyncs());
        L->print(F(",bytes_rx="));           L->print(crsf::bytes_rx());
        L->println();

        uint8_t raw[32];
        const size_t rn = crsf::raw_sample(raw, sizeof(raw));
        L->print(F("RAW,"));
        for (size_t i = 0; i < rn; i++) {
            if (raw[i] < 0x10) L->print('0');
            L->print(raw[i], HEX);
            L->print(' ');
        }
        L->println();
    }
}
