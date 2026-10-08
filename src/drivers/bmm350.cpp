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
Diagnostics diag{};
int8_t transfer_result(hal::Status status,uint8_t reg) {
    if(status==hal::Status::ok) return 0;
    ++diag.bus_errors; diag.last_error_register=reg;
    return -1;
}
int8_t read_reg(uint8_t reg,uint8_t* data,uint32_t n,void*) {
    // The Bosch API asks for and removes the two I2C dummy bytes.
    return transfer_result(hal::i2c_write_read(hal::I2cBus::mag,config::bmm350_address,&reg,1,data,n),reg);
}
int8_t write_reg(uint8_t reg,const uint8_t* data,uint32_t n,void*) {
    uint8_t buf[32]; if(n+1>sizeof(buf)) return -1;
    buf[0]=reg; std::memcpy(buf+1,data,n);
    return transfer_result(hal::i2c_write_read(hal::I2cBus::mag,config::bmm350_address,buf,n+1,nullptr,0),reg);
}
void delay(uint32_t us,void*) { hal::delay_us(us); }
int16_t probe_id(uint8_t address) {
    const uint8_t reg=BMM350_REG_CHIP_ID;
    uint8_t data[1+BMM350_DUMMY_BYTES]{};
    // Boot-only, read-only identity probes at the two documented addresses.
    // These do not change the configured address or retry a failed device.
    if(hal::i2c_write_read(hal::I2cBus::mag,address,&reg,1,data,sizeof(data))!=hal::Status::ok) return -1;
    return data[BMM350_DUMMY_BYTES];
}
bool accept(int8_t result,uint8_t stage) {
    diag.result=result;
    if(result==BMM350_OK) return true;
    diag.stage=stage; return false;
}
}
bool begin() {
    ready=have_data=false; dev={}; diag={};
    if(hal::i2c_config(hal::I2cBus::mag,100000)!=hal::Status::ok) { diag.stage=1; return false; }
    dev.read=read_reg; dev.write=write_reg; dev.delay_us=delay;
    const int8_t init_result=bmm350_init(&dev);
    diag.chip_id=dev.chip_id;
    diag.id14=probe_id(0x14); diag.id15=probe_id(0x15);
    if(!accept(init_result,2) ||
       !accept(bmm350_configure_interrupt(BMM350_PULSED,BMM350_ACTIVE_HIGH,BMM350_INTR_PUSH_PULL,BMM350_UNMAP_FROM_PIN,&dev),3) ||
       !accept(bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT,&dev),4) ||
       !accept(bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ,BMM350_AVERAGING_8,&dev),5) ||
       !accept(bmm350_enable_axes(BMM350_X_EN,BMM350_Y_EN,BMM350_Z_EN,&dev),6) ||
       !accept(bmm350_set_powermode(BMM350_NORMAL_MODE,&dev),7)) return false;
    ready=true; diag.initialized=true; return true;
}
bool poll(Sample& out) {
    if(!ready) return false;
    uint8_t status=0;
    if(!accept(bmm350_get_regs(BMM350_REG_INT_STATUS,&status,1,&dev),8)) { ready=false; return false; }
    diag.status=status;
    if(!(status&BMM350_DRDY_DATA_REG_MSK)) return false;
    bmm350_mag_temp_data m{};
    if(!accept(bmm350_get_compensated_mag_xyz_temp_data(&m,&dev),9)) { ready=false; return false; }
    if(!std::isfinite(m.x)||!std::isfinite(m.y)||!std::isfinite(m.z)||!std::isfinite(m.temperature)) { diag.stage=10; return false; }
    out={float(m.x),float(m.y),float(m.z),float(m.temperature)};
    last_ms=hal::millis(); have_data=true; diag.stage=0; ++diag.samples; return true;
}
bool healthy() { return ready && have_data && uint32_t(hal::millis()-last_ms)<200; }
const Diagnostics& diagnostics() { return diag; }
}
