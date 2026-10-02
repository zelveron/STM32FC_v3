#include "bmi323_fifo.hpp"
#include <cstring>
#include <cstdlib>

namespace imu323 {
namespace {
uint16_t word(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1])<<8); }
}
int8_t Device::read_reg(uint8_t reg,uint8_t* data,uint32_t len,void* context) {
    auto& d=*static_cast<Device*>(context);
    uint8_t tx[258]{},rx[258]{};
    if(len+1>sizeof(tx)) return BMI3_E_COM_FAIL;
    tx[0]=reg;
    if(hal::spi_xfer(d._bus,d._cs,tx,rx,len+1)!=hal::Status::ok) return BMI3_E_COM_FAIL;
    if(d._spi_select_pending) {
        // Bosch soft_reset() performs a dummy read but its generic 2 us idle
        // is shorter than the BMI323's 200 us protocol-switch requirement.
        hal::delay_us(250); d._spi_select_pending=false;
    }
    std::memcpy(data,rx+1,len); // Bosch consumes the one SPI dummy byte.
    return BMI3_INTF_RET_SUCCESS;
}
int8_t Device::write_reg(uint8_t reg,const uint8_t* data,uint32_t len,void* context) {
    auto& d=*static_cast<Device*>(context); uint8_t tx[258]{};
    if(len+1>sizeof(tx)) return BMI3_E_COM_FAIL;
    tx[0]=reg; std::memcpy(tx+1,data,len);
    if(hal::spi_xfer(d._bus,d._cs,tx,nullptr,len+1)!=hal::Status::ok) return BMI3_E_COM_FAIL;
    if((reg&0x7f)==BMI3_REG_CMD && len==2 && data[0]==0xaf && data[1]==0xde)
        d._spi_select_pending=true;
    return BMI3_INTF_RET_SUCCESS;
}
void Device::delay(uint32_t us,void*) { hal::delay_us(us); }
bool Device::begin(hal::SpiBus bus,hal::PinId cs) {
    _ready=false; _have_time=false; _sensor_time=0; _error=0; _health_registers=0; _spi_select_pending=false;
    _bus=bus; _cs=cs; _dev={};
    hal::gpio_write(cs,true); hal::gpio_config(cs,hal::PinMode::output);
    if(hal::spi_config(bus,1000000,0)!=hal::Status::ok) return fail(1);
    // Select SPI before the API's first soft-reset write, including cold boot.
    hal::gpio_write(cs,false); hal::delay_us(2);
    hal::gpio_write(cs,true); hal::delay_us(250);
    _dev.intf=BMI3_SPI_INTF; _dev.read=read_reg; _dev.write=write_reg;
    _dev.delay_us=delay; _dev.intf_ptr=this; _dev.read_write_len=32;
    const int8_t init=bmi323_init(&_dev);
    if(init!=BMI3_OK) return fail(100-int(init));

    // Enable FIFO BEFORE enabling measurement (datasheet section 5.7.2).
    if(bmi323_set_fifo_config(BMI3_FIFO_ALL_EN|BMI3_FIFO_STOP_ON_FULL,BMI3_DISABLE,&_dev)!=BMI3_OK ||
       bmi323_set_fifo_config(BMI3_FIFO_ACC_EN|BMI3_FIFO_GYR_EN|BMI3_FIFO_STOP_ON_FULL,BMI3_ENABLE,&_dev)!=BMI3_OK)
        return fail(2);
    bmi3_sens_config cfg[2]{}; cfg[0].type=BMI323_ACCEL; cfg[1].type=BMI323_GYRO;
    if(bmi323_get_sensor_config(cfg,2,&_dev)!=BMI3_OK) return fail(3);
    cfg[0].cfg.acc.odr=BMI3_ACC_ODR_400HZ; cfg[0].cfg.acc.range=BMI3_ACC_RANGE_8G;
    cfg[0].cfg.acc.bwp=BMI3_ACC_BW_ODR_QUARTER; cfg[0].cfg.acc.avg_num=BMI3_ACC_AVG1;
    cfg[0].cfg.acc.acc_mode=BMI3_ACC_MODE_HIGH_PERF;
    cfg[1].cfg.gyr.odr=BMI3_GYR_ODR_400HZ; cfg[1].cfg.gyr.range=BMI3_GYR_RANGE_2000DPS;
    cfg[1].cfg.gyr.bwp=BMI3_GYR_BW_ODR_HALF; cfg[1].cfg.gyr.avg_num=BMI3_GYR_AVG1;
    cfg[1].cfg.gyr.gyr_mode=BMI3_GYR_MODE_HIGH_PERF;
    if(bmi323_set_sensor_config(cfg,2,&_dev)!=BMI3_OK) return fail(4);
    // Configuration settling is startup-only. Discard dummy/settling frames.
    hal::delay_ms(80);
    bmi3_sens_config actual[2]{}; actual[0].type=BMI323_ACCEL; actual[1].type=BMI323_GYRO;
    uint16_t fifo_cfg=0;
    if(bmi323_get_sensor_config(actual,2,&_dev)!=BMI3_OK ||
       actual[0].cfg.acc.odr!=cfg[0].cfg.acc.odr || actual[0].cfg.acc.range!=cfg[0].cfg.acc.range ||
       actual[0].cfg.acc.acc_mode!=cfg[0].cfg.acc.acc_mode ||
       actual[1].cfg.gyr.odr!=cfg[1].cfg.gyr.odr || actual[1].cfg.gyr.range!=cfg[1].cfg.gyr.range ||
       actual[1].cfg.gyr.gyr_mode!=cfg[1].cfg.gyr.gyr_mode ||
       bmi323_get_fifo_config(&fifo_cfg,&_dev)!=BMI3_OK || fifo_cfg!=0x0601) return fail(5);
    _ready=true; return discard_pending();
}
bool Device::discard_pending() {
    _have_time=false;
    const uint8_t flush[2]={1,0};
    if(!_ready || bmi323_set_regs(BMI3_REG_FIFO_CTRL,flush,2,&_dev)!=BMI3_OK) return fail(6);
    // Startup only: the SPI-select transition can leave clear-on-read I3C
    // S0/S1/parity flags from the previous interface state (observed 0x0800).
    // Clear those before accepting any samples. Fatal/configuration/feature
    // errors still reject startup, and ALL subsequent errors latch in read().
    uint8_t error[2]{};
    if(bmi323_get_regs(BMI3_REG_ERR_REG,error,2,&_dev)!=BMI3_OK ||
       (word(error)&~(BMI3_I3C_ERROR0_MASK|BMI3_I3C_ERROR3_MASK))) return fail(7);
    return true;
}
Result Device::read(Sample (&samples)[8],uint8_t& count) {
    count=0; if(!_ready) return Result::fault;
    // CHIP_ID and ERR_REG are adjacent 16-bit registers.
    uint8_t health[4]{}; uint16_t words=0; uint32_t stamp=0;
    const int8_t health_result=bmi323_get_regs(BMI3_REG_CHIP_ID,health,4,&_dev);
    _health_registers=(uint32_t(word(health))<<16)|word(health+2);
    if(health_result!=BMI3_OK || health[0]!=BMI323_CHIP_ID || word(health+2)!=0) {
        fail(7); return Result::fault;
    }
    const int8_t len_result=bmi323_get_fifo_length(&words,&_dev);
    if(len_result<0 || words>48) { fail(8); return Result::fault; }
    if(words<6) return Result::no_data;
    if(bmi323_get_sensor_time(&stamp,&_dev)!=BMI3_OK || (_have_time && stamp==_sensor_time)) {
        fail(9); return Result::fault;
    }
    const uint8_t frames=uint8_t(words/6);
    uint8_t raw[96]{};
    if(bmi323_get_regs(BMI3_REG_FIFO_DATA,raw,uint16_t(frames)*12,&_dev)!=BMI3_OK) {
        fail(10); return Result::fault;
    }
    for(unsigned n=0;n<frames;++n) {
        int16_t v[6];
        for(unsigned a=0;a<6;++a) {
            v[a]=static_cast<int16_t>(word(raw+n*12+a*2));
            // Reject clipping, FIFO overread (0x8000) and settling dummy frames.
            if(std::abs(int(v[a]))>32700) { fail(11); return Result::fault; }
        }
        samples[n]={v[0]/4096.0f,v[1]/4096.0f,v[2]/4096.0f,
                    v[3]*(2000.0f/32768),v[4]*(2000.0f/32768),v[5]*(2000.0f/32768)};
    }
    _sensor_time=stamp; _have_time=true; count=frames;
    return Result::ok;
}
}
