#pragma once
#include "../hal/hal.hpp"
#include <bmi323.h>

// BMI323 assembly variant. Fixed paired FIFO: accel XYZ then gyro XYZ,
// six little-endian words per frame; FIFO fill level is in WORDS, not bytes.
namespace imu323 {
struct Sample { float ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps; };
enum class Result { ok, no_data, fault };
class Device {
public:
    bool begin(hal::SpiBus bus,hal::PinId cs);
    bool discard_pending();
    Result read(Sample (&samples)[8],uint8_t& count);
    uint32_t sensor_time() const { return _sensor_time; }
    int error() const { return _error; }
    uint32_t health_registers() const { return _health_registers; }
private:
    static int8_t read_reg(uint8_t,uint8_t*,uint32_t,void*);
    static int8_t write_reg(uint8_t,const uint8_t*,uint32_t,void*);
    static void delay(uint32_t,void*);
    bool fail(int code) { _error=code; _ready=false; return false; }
    bmi3_dev _dev{};
    hal::SpiBus _bus=hal::SpiBus::imu;
    hal::PinId _cs=0;
    uint32_t _sensor_time=0;
    uint32_t _health_registers=0;
    int _error=0;
    bool _ready=false, _have_time=false;
    bool _spi_select_pending=false;
};
}
