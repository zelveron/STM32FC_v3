#include "bmm350.hpp"
#include "../hal/hal.hpp"
#include "../config/airframe.hpp"
#include <bmm350.h>
#include <cmath>
#include <cstring>
namespace mag350 {
namespace {
bmm350_dev dev{};
bool ready=false,have_data=false;
uint32_t last_ms=0;
int8_t read_reg(uint8_t reg,uint8_t* data,uint32_t n,void*) {
    // The Bosch API asks for and removes the two I2C dummy bytes.
    return hal::i2c_write_read(hal::I2cBus::mag,config::bmm350_address,&reg,1,data,n)==hal::Status::ok?0:-1;
}
int8_t write_reg(uint8_t reg,const uint8_t* data,uint32_t n,void*) {
    uint8_t buf[32]; if(n+1>sizeof(buf)) return -1;
    buf[0]=reg; std::memcpy(buf+1,data,n);
    return hal::i2c_write_read(hal::I2cBus::mag,config::bmm350_address,buf,n+1,nullptr,0)==hal::Status::ok?0:-1;
}
void delay(uint32_t us,void*) { hal::delay_us(us); }
}
bool begin() {
    ready=have_data=false; dev={};
    if(hal::i2c_config(hal::I2cBus::mag,100000)!=hal::Status::ok) return false;
    dev.read=read_reg; dev.write=write_reg; dev.delay_us=delay;
    if(bmm350_init(&dev)!=BMM350_OK ||
       bmm350_configure_interrupt(BMM350_PULSED,BMM350_ACTIVE_HIGH,BMM350_INTR_PUSH_PULL,BMM350_UNMAP_FROM_PIN,&dev)!=BMM350_OK ||
       bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT,&dev)!=BMM350_OK ||
       bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ,BMM350_AVERAGING_8,&dev)!=BMM350_OK ||
       bmm350_enable_axes(BMM350_X_EN,BMM350_Y_EN,BMM350_Z_EN,&dev)!=BMM350_OK ||
       bmm350_set_powermode(BMM350_NORMAL_MODE,&dev)!=BMM350_OK) return false;
    ready=true; return true;
}
bool poll(Sample& out) {
    if(!ready) return false;
    uint8_t status=0;
    if(bmm350_get_regs(BMM350_REG_INT_STATUS,&status,1,&dev)!=BMM350_OK) { ready=false; return false; }
    if(!(status&BMM350_DRDY_DATA_REG_MSK)) return false;
    bmm350_mag_temp_data m{};
    if(bmm350_get_compensated_mag_xyz_temp_data(&m,&dev)!=BMM350_OK) { ready=false; return false; }
    if(!std::isfinite(m.x)||!std::isfinite(m.y)||!std::isfinite(m.z)||!std::isfinite(m.temperature)) return false;
    out={float(m.x),float(m.y),float(m.z),float(m.temperature)};
    last_ms=hal::millis(); have_data=true; return true;
}
bool healthy() { return ready && have_data && uint32_t(hal::millis()-last_ms)<200; }
}
