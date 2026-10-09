#pragma once
#include <cstdint>
namespace mag350 {
constexpr unsigned otp_pass_count=3,otp_word_count=32,trim_count=19;
struct Sample { float x_ut,y_ut,z_ut,temp_c; };
// Stage: 0 ready/not started, 1 bus, 2 Bosch init, 3 interrupt config,
// 4 interrupt enable, 5 ODR, 6 axes, 7 normal mode, 8 status, 9 data,
// 10 nonfinite sample, 11 out of range, 12 invalid OTP acquisition,
// 13 self-test failed, 14 configuration readback mismatch.
struct Diagnostics {
    bool initialized=false;
    uint8_t stage=0,chip_id=0,last_error_register=0,status=0;
    int8_t result=0;
    int16_t id14=-1,id15=-1; // -1 = register transfer failed; otherwise CHIP_ID byte
    uint32_t bus_errors=0,samples=0;
    int8_t last_bus_status=0;
    uint32_t reads=0,invalid_samples=0,recoveries=0,consecutive_errors=0;
    uint8_t otp_error=0;
    uint8_t otp_passes=0;
    uint32_t otp_mismatch=0;
    uint16_t otp[otp_pass_count][otp_word_count]{};
    // t_off,x_off,y_off,z_off,t_sens,x_sens,y_sens,z_sens,
    // x_tco,y_tco,z_tco,x_tcs,y_tcs,z_tcs,t0,cxy,cyx,czx,czy.
    float trim[trim_count]{};
    bool self_test_ok=false;
    float self_test_x=0,self_test_y=0;
    uint8_t error_reg=0,pmu=0,aggr=0,axes=0,self_test_reg=0;
    int32_t raw[4]{}; // same burst used by Bosch compensation, signed 24-bit
    Sample last_sample{}; // diagnostics even when rejected; not flight data
};
bool begin();
bool poll(Sample& out);
bool healthy();
bool communicating();
const Diagnostics& diagnostics();
}
