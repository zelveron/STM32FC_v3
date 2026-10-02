#include "bmp581.hpp"
#include "../hal/hal.hpp"
#include "../config/airframe.hpp"
#include <bmp5.h>
#include <cmath>
#include <cstring>
namespace bmp581 {
namespace {
bmp5_dev dev{};
bmp5_osr_odr_press_config cfg{};
bool ready=false;
uint32_t last_ms=0;
bool have_data=false;
int fault=0;
bool fail(int code) { fault=code; ready=false; return false; }
int8_t read_reg(uint8_t reg,uint8_t* data,uint32_t n,void*) {
    return hal::i2c_write_read(hal::I2cBus::baro,config::bmp581_address,&reg,1,data,n)==hal::Status::ok?0:-1;
}
int8_t write_reg(uint8_t reg,const uint8_t* data,uint32_t n,void*) {
    uint8_t buf[32]; if(n+1>sizeof(buf)) return -1;
    buf[0]=reg; std::memcpy(buf+1,data,n);
    return hal::i2c_write_read(hal::I2cBus::baro,config::bmp581_address,buf,n+1,nullptr,0)==hal::Status::ok?0:-1;
}
void delay(uint32_t us,void*) { hal::delay_us(us); }
}
bool begin() {
    ready=have_data=false; dev={}; fault=0;
    if(hal::i2c_config(hal::I2cBus::baro,100000)!=hal::Status::ok) return fail(1);
    dev.intf=BMP5_I2C_INTF; dev.read=read_reg; dev.write=write_reg; dev.delay_us=delay;
    // MCU/DFU resets do not reset a still-powered BMP581. Return it to its
    // reset state before Bosch's NVM-ready check; no sensor power cycling.
    const uint8_t reset=BMP5_SOFT_RESET_CMD;
    if(bmp5_set_regs(BMP5_REG_CMD,&reset,1,&dev)!=BMP5_OK) return fail(2);
    hal::delay_us(20000);
    const int8_t init=bmp5_init(&dev);
    if(init!=BMP5_OK) return fail(100-int(init));
    cfg.osr_t=BMP5_OVERSAMPLING_1X; cfg.osr_p=BMP5_OVERSAMPLING_4X;
    cfg.press_en=BMP5_ENABLE; cfg.odr=BMP5_ODR_50_HZ;
    struct bmp5_int_source_select source{}; source.drdy_en=BMP5_ENABLE;
    if(bmp5_set_osr_odr_press_config(&cfg,&dev)!=BMP5_OK ||
       bmp5_int_source_select(&source,&dev)!=BMP5_OK ||
       bmp5_set_power_mode(BMP5_POWERMODE_NORMAL,&dev)!=BMP5_OK) return fail(3);
    bmp5_osr_odr_eff effective{};
    if(bmp5_get_osr_odr_eff(&effective,&dev)!=BMP5_OK || !effective.odr_is_valid) return fail(4);
    ready=true; return true;
}
bool poll(Sample& out) {
    if(!ready) return false;
    uint8_t status=0;
    if(bmp5_get_interrupt_status(&status,&dev)!=BMP5_OK) return fail(5);
    if(!(status&BMP5_INT_ASSERTED_DRDY)) return false;
    bmp5_sensor_data data{};
    if(bmp5_get_sensor_data(&data,&cfg,&dev)!=BMP5_OK) return fail(6);
    const float p=float(data.pressure),t=float(data.temperature);
    if(!std::isfinite(p)||!std::isfinite(t)||p<30000||p>125000||t<-40||t>85) return false;
    out={p,t}; last_ms=hal::millis(); have_data=true; return true;
}
bool healthy() { return ready && have_data && uint32_t(hal::millis()-last_ms)<200; }
int error() { return fault; }
}
