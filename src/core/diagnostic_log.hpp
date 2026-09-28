#pragma once
#include "log_frame.hpp"
#include <cmath>
namespace core {
#pragma pack(push,1)
struct ImuLogFrame {
    uint16_t magic=0xA16D;
    uint32_t host_us=0,sequence=0,sensor_time=0;
    int16_t raw[6]{},filtered[6]{}; // gyro dps*16, accel g*2048
    uint8_t sensor=0,flags=0; // bit0 calibrated; raw is sensor-filtered body axes, before stored calibration
    uint16_t crc=0;
};
struct ControlLogFrame {
    uint16_t magic=0xC17D;
    uint32_t host_us=0;
    int16_t demand[2]{},measured[3]{},surface[3]{}; // rates*16; outputs*10000
    uint16_t throttle=0; // *10000
    uint8_t mode=0,flags=0; // armed, failsafe, I enabled
    uint16_t crc=0;
};
#pragma pack(pop)
inline int16_t log_i16(float value,float scale) {
    if(!std::isfinite(value)) return 0;
    const float v=value*scale;
    return int16_t(v<-32768?-32768:v>32767?32767:v);
}
template<class T> void finish_diagnostic(T& f) { f.crc=log_crc16(&f,sizeof(f)-2); }
}
