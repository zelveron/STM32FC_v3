#pragma once
#include "../hal/hal.hpp"
#include <bmi270.h>
namespace imu270 {
struct Sample { float ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps; };
enum class Result { ok, no_data, fault };
class Device {
public:
    bool begin(hal::SpiBus bus,hal::PinId cs);
    // Startup only: discard samples accumulated during other devices' init.
    bool discard_pending();
    uint32_t sensor_time() const { return _sensor_time; }
    // Every FIFO sample is returned, oldest first, at 400 Hz. Backlogs over
    // 20 ms are faults, never silently decimated or treated as fresh data.
    Result read(Sample (&samples)[8],uint8_t& count);
private:
    static int8_t read_reg(uint8_t,uint8_t*,uint32_t,void*);
    static int8_t write_reg(uint8_t,const uint8_t*,uint32_t,void*);
    static void delay(uint32_t,void*);
    bmi2_dev _dev{};
    hal::SpiBus _bus=hal::SpiBus::imu;
    hal::PinId _cs=0;
    uint32_t _sensor_time=0;
    bool _ready=false, _have_time=false;
};
}
