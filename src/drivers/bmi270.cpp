#include "bmi270.hpp"
#include <cstring>
#include <cstdlib>
namespace imu270 {
int8_t Device::read_reg(uint8_t reg,uint8_t* data,uint32_t len,void* context) {
    auto& d=*static_cast<Device*>(context);
    uint8_t tx[258]{},rx[258]{};
    if(len+1>sizeof(tx)) return -1;
    tx[0]=reg;
    if(hal::spi_xfer(d._bus,d._cs,tx,rx,len+1)!=hal::Status::ok) return -1;
    std::memcpy(data,rx+1,len); // Bosch API consumes the SPI dummy byte.
    return 0;
}
int8_t Device::write_reg(uint8_t reg,const uint8_t* data,uint32_t len,void* context) {
    auto& d=*static_cast<Device*>(context); uint8_t tx[258]{};
    if(len+1>sizeof(tx)) return -1;
    tx[0]=reg; std::memcpy(tx+1,data,len);
    return hal::spi_xfer(d._bus,d._cs,tx,nullptr,len+1)==hal::Status::ok?0:-1;
}
void Device::delay(uint32_t us,void*) { hal::delay_us(us); }
bool Device::capture_health() {
    uint8_t id=0,err=0,status=0;
    const auto ir=bmi2_get_regs(BMI2_CHIP_ID_ADDR,&id,1,&_dev);
    const auto er=bmi2_get_regs(0x02 /* ERR_REG */,&err,1,&_dev);
    const auto sr=bmi2_get_regs(BMI2_INTERNAL_STATUS_ADDR,&status,1,&_dev);
    _health_registers=(uint32_t(id)<<16)|(uint32_t(status)<<8)|err;
    return ir==BMI2_OK && er==BMI2_OK && sr==BMI2_OK && id==BMI270_CHIP_ID &&
        !err && (status&BMI2_CONFIG_LOAD_STATUS_MASK)==BMI2_CONFIG_LOAD_SUCCESS;
}
bool Device::begin(hal::SpiBus bus,hal::PinId cs) {
    _ready=false; _have_time=false; _sensor_time=0; _error=0; _health_registers=0;
    _bus=bus; _cs=cs; _dev={};
    hal::gpio_write(cs,true); hal::gpio_config(cs,hal::PinMode::output);
    // Keep reset/config upload at 1 MHz; use 5 MHz for FIFO service only after
    // the device has accepted its configuration and completed startup.
    if(hal::spi_config(bus,1000000,0)!=hal::Status::ok) return fail(1);
    _dev.intf=BMI2_SPI_INTF; _dev.read=read_reg; _dev.write=write_reg;
    _dev.delay_us=delay; _dev.intf_ptr=this; _dev.read_write_len=32;
    // Bosch performs SPI selection, reset, the complete 8192-byte config
    // upload and INTERNAL_STATUS verification separately for each device.
    const int8_t init=bmi270_init(&_dev);
    const bool init_health=capture_health();
    if(init!=BMI2_OK) return fail(100-int(init));
    if(!init_health) return fail(7);
    if(bmi2_set_adv_power_save(BMI2_DISABLE,&_dev)!=BMI2_OK) return fail(2);
    bmi2_sens_config cfg[2]{}; cfg[0].type=BMI2_ACCEL; cfg[1].type=BMI2_GYRO;
    if(bmi270_get_sensor_config(cfg,2,&_dev)!=BMI2_OK) return fail(3);
    cfg[0].cfg.acc.odr=BMI2_ACC_ODR_400HZ; cfg[0].cfg.acc.range=BMI2_ACC_RANGE_8G;
    cfg[0].cfg.acc.bwp=BMI2_ACC_OSR4_AVG1; cfg[0].cfg.acc.filter_perf=BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.odr=BMI2_GYR_ODR_400HZ; cfg[1].cfg.gyr.range=BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp=BMI2_GYR_OSR4_MODE;
    cfg[1].cfg.gyr.filter_perf=BMI2_PERF_OPT_MODE; cfg[1].cfg.gyr.noise_perf=BMI2_PERF_OPT_MODE;
    uint8_t sensors[2]={BMI2_ACCEL,BMI2_GYRO};
    if(bmi270_set_sensor_config(cfg,2,&_dev)!=BMI2_OK) return fail(4);
    if(bmi2_set_fifo_config(BMI2_FIFO_ALL_EN,BMI2_DISABLE,&_dev)!=BMI2_OK ||
       bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN|BMI2_FIFO_TIME_EN,BMI2_DISABLE,&_dev)!=BMI2_OK ||
       bmi2_set_fifo_config(BMI2_FIFO_ACC_EN|BMI2_FIFO_GYR_EN|BMI2_FIFO_STOP_ON_FULL,BMI2_ENABLE,&_dev)!=BMI2_OK ||
       bmi2_set_fifo_filter_data(BMI2_ACCEL,BMI2_FIFO_FILTERED_DATA,&_dev)!=BMI2_OK ||
       bmi2_set_fifo_filter_data(BMI2_GYRO,BMI2_FIFO_FILTERED_DATA,&_dev)!=BMI2_OK) return fail(5);
    if(bmi270_sensor_enable(sensors,2,&_dev)!=BMI2_OK) return fail(4);
    // Suspend-to-normal gyro startup is 45 ms (Bosch datasheet, table 3).
    // Headerless FIFO may contain dummy accel/gyro frames while settling.
    // Startup only: let both channels settle, then discard those frames before
    // strict paired-frame validation starts. Runtime faults remain latched.
    hal::delay_ms(80);
    if(hal::spi_config(bus,5000000,0)!=hal::Status::ok) return fail(1);
    _ready=true; return discard_pending();
}
bool Device::discard_pending() {
    _have_time=false;
    if(!_ready || bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD,&_dev)!=BMI2_OK) {
        return fail(6);
    }
    return true;
}
Result Device::read(Sample (&samples)[8],uint8_t& count) {
    count=0; if(!_ready) return Result::fault;
    uint8_t time[3]{}; uint16_t len=0;
    if(!capture_health()) { fail(7); return Result::fault; }
    if(bmi2_get_fifo_length(&len,&_dev)!=BMI2_OK || len>96) { fail(8); return Result::fault; }
    if(len<12) return Result::no_data;
    if(bmi2_get_regs(BMI2_SENSORTIME_ADDR,time,3,&_dev)!=BMI2_OK) { fail(9); return Result::fault; }
    const uint32_t stamp=uint32_t(time[0])|(uint32_t(time[1])<<8)|(uint32_t(time[2])<<16);
    if(_have_time && stamp==_sensor_time) { fail(9); return Result::fault; }
    _sensor_time=stamp; _have_time=true;
    // Read whole paired frames only. An in-progress FIFO frame stays queued.
    len=uint16_t((len/12)*12);
    uint8_t raw[97]{}; bmi2_fifo_frame fifo{}; fifo.data=raw; fifo.length=len+_dev.dummy_byte;
    if(bmi2_read_fifo_data(&fifo,&_dev)!=BMI2_OK) { fail(10); return Result::fault; }
    bmi2_sens_axes_data acc[8]{},gyr[8]{}; uint16_t na=8,ng=8;
    const int8_t ar=bmi2_extract_accel(acc,&na,&fifo,&_dev), gr=bmi2_extract_gyro(gyr,&ng,&fifo,&_dev);
    if(ar<0 || gr<0 || na!=ng || na!=len/12) { fail(11); return Result::fault; }
    for(unsigned i=0;i<na;++i) {
        if(std::abs(int(acc[i].x))>32700 || std::abs(int(acc[i].y))>32700 ||
           std::abs(int(acc[i].z))>32700 || std::abs(int(gyr[i].x))>32700 ||
           std::abs(int(gyr[i].y))>32700 || std::abs(int(gyr[i].z))>32700) { fail(12); return Result::fault; }
        samples[i]={acc[i].x/4096.0f,acc[i].y/4096.0f,acc[i].z/4096.0f,
                    gyr[i].x*(2000.0f/32768),gyr[i].y*(2000.0f/32768),gyr[i].z*(2000.0f/32768)};
    }
    count=uint8_t(na); return Result::ok;
}
}
