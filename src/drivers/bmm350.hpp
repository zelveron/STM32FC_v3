#pragma once
#include <cstdint>
namespace mag350 {
struct Sample { float x_ut,y_ut,z_ut,temp_c; };
// Stage: 0 ready/not started, 1 bus, 2 Bosch init, 3 interrupt config,
// 4 interrupt enable, 5 ODR, 6 axes, 7 normal mode, 8 status, 9 data, 10 invalid sample.
struct Diagnostics {
    bool initialized=false;
    uint8_t stage=0,chip_id=0,last_error_register=0,status=0;
    int8_t result=0;
    int16_t id14=-1,id15=-1; // -1 = register transfer failed; otherwise CHIP_ID byte
    uint32_t bus_errors=0,samples=0;
};
bool begin();
bool poll(Sample& out);
bool healthy();
const Diagnostics& diagnostics();
}
