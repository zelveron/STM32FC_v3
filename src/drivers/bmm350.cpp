#include "bmm350.hpp"
#include "../hal/hal.hpp"
#include "../config/airframe.hpp"
#include <bmm350.h>
#include <cmath>
#include <cstring>
namespace mag350 {
namespace {
bmm350_dev dev{};
bool ready=false,have_data=false,have_rx=false;
uint32_t last_ms=0,last_rx_ms=0,retry_start=0,retry_delay=0;
Diagnostics diag{};
int8_t transfer_result(hal::Status status,uint8_t reg) {
    if(status==hal::Status::ok) return 0;
    ++diag.bus_errors; diag.last_error_register=reg; diag.last_bus_status=int8_t(status);
    return -1;
}
int8_t read_reg(uint8_t reg,uint8_t* data,uint32_t n,void*) {
    // The Bosch API asks for and removes the two I2C dummy bytes.
    const int8_t result=transfer_result(hal::i2c_write_read(hal::I2cBus::mag,config::bmm350_address,&reg,1,data,n),reg);
    if(result==0 && n>=3 && reg==BMM350_REG_OTP_STATUS_REG)
        diag.otp_error|=BMM350_OTP_STATUS_ERROR(data[BMM350_DUMMY_BYTES]);
    if(result==0 && reg==BMM350_REG_MAG_X_XLSB && n==BMM350_MAG_TEMP_DATA_LEN+BMM350_DUMMY_BYTES) {
        for(unsigned i=0;i<4;++i) {
            const uint8_t* p=data+BMM350_DUMMY_BYTES+3*i;
            const uint32_t value=uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16);
            diag.raw[i]=int32_t(value)-((value&0x800000)?0x1000000:0);
        }
    }
    return result;
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
void read_failed() {
    have_data=have_rx=false;
    if(diag.consecutive_errors<1000) ++diag.consecutive_errors;
    const uint32_t shift=diag.consecutive_errors>4?4:diag.consecutive_errors-1;
    retry_delay=100u<<shift; if(retry_delay>1000) retry_delay=1000;
    retry_start=hal::millis();
}
}
bool begin() {
    ready=have_data=have_rx=false; dev={}; diag={}; retry_delay=0;
    // Fast-mode keeps the complete 14-byte read well inside the bounded
    // transport budget, without extending the flight-loop blocking time.
    if(hal::i2c_config(hal::I2cBus::mag,400000)!=hal::Status::ok) { diag.stage=1; return false; }
    dev.read=read_reg; dev.write=write_reg; dev.delay_us=delay;
    const int8_t init_result=bmm350_init(&dev);
    diag.chip_id=dev.chip_id;
    diag.id14=probe_id(0x14); diag.id15=probe_id(0x15);
    // The vendor OTP loop returns its final word's result. Do not let a
    // successful last read conceal any earlier transport/OTP-status failure.
    if(init_result==BMM350_OK && (diag.bus_errors||diag.otp_error)) {
        diag.stage=12; diag.result=diag.bus_errors?BMM350_E_COM_FAIL:BMM350_E_OTP_UNDEFINED;
        return false;
    }
    if(!accept(init_result,2)) return false;
    // Boot only: Bosch's positive/negative X/Y test includes a full magnetic
    // reset. Never run its blocking delays after the scheduler starts.
    bmm350_self_test self_test{};
    const int8_t self_result=bmm350_perform_self_test(&self_test,&dev);
    // Also explicitly clean up after an interrupted vendor self-test.
    const uint8_t st_off=0;
    const int8_t cleanup=bmm350_set_regs(BMM350_REG_TMR_SELFTEST_USER,&st_off,1,&dev);
    if(!accept(self_result,13)||!accept(cleanup,13)) return false;
    diag.self_test_x=self_test.out_ust_x; diag.self_test_y=self_test.out_ust_y;
    // Full positive-minus-negative threshold from Bosch bmm350_oor.h v1.10.0.
    diag.self_test_ok=std::isfinite(diag.self_test_x)&&std::isfinite(diag.self_test_y)&&
                      diag.self_test_x>=300&&diag.self_test_y>=300;
    if(!accept(bmm350_set_powermode(BMM350_SUSPEND_MODE,&dev),7)||
       !accept(bmm350_configure_interrupt(BMM350_PULSED,BMM350_ACTIVE_HIGH,BMM350_INTR_PUSH_PULL,BMM350_UNMAP_FROM_PIN,&dev),3) ||
       !accept(bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT,&dev),4) ||
       !accept(bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ,BMM350_AVERAGING_8,&dev),5) ||
       !accept(bmm350_enable_axes(BMM350_X_EN,BMM350_Y_EN,BMM350_Z_EN,&dev),6) ||
       !accept(bmm350_set_powermode(BMM350_NORMAL_MODE,&dev),7)) return false;
    // Verify that successful bus writes actually installed the intended mode.
    if(!accept(bmm350_get_regs(BMM350_REG_ERR_REG,&diag.error_reg,1,&dev),14)||
       !accept(bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET,&diag.aggr,1,&dev),14)||
       !accept(bmm350_get_regs(BMM350_REG_PMU_CMD_AXIS_EN,&diag.axes,1,&dev),14)||
       !accept(bmm350_get_regs(BMM350_REG_PMU_CMD_STATUS_0,&diag.pmu,1,&dev),14)||
       !accept(bmm350_get_regs(BMM350_REG_TMR_SELFTEST_USER,&diag.self_test_reg,1,&dev),14)) return false;
    if(diag.error_reg||diag.aggr!=0x36||diag.axes!=7||diag.pmu!=0x28||diag.self_test_reg) {
        diag.stage=14; return false;
    }
    ready=true; diag.initialized=true; return true;
}
bool poll(Sample& out) {
    if(!ready) return false;
    if(retry_delay && uint32_t(hal::millis()-retry_start)<retry_delay) return false;
    uint8_t status=0;
    if(!accept(bmm350_get_regs(BMM350_REG_INT_STATUS,&status,1,&dev),8)) { read_failed(); return false; }
    diag.status=status;
    if(!(status&BMM350_DRDY_DATA_REG_MSK)) {
        // A responsive but not-ready sensor is polled at 10 Hz after a fault.
        if(retry_delay) { retry_start=hal::millis(); retry_delay=100; }
        return false;
    }
    bmm350_mag_temp_data m{};
    if(!accept(bmm350_get_compensated_mag_xyz_temp_data(&m,&dev),9)) { read_failed(); return false; }
    if(diag.consecutive_errors) ++diag.recoveries;
    diag.consecutive_errors=retry_delay=0;
    have_rx=true; last_rx_ms=hal::millis(); ++diag.reads;
    diag.last_sample={float(m.x),float(m.y),float(m.z),float(m.temperature)};
    if(!std::isfinite(m.x)||!std::isfinite(m.y)||!std::isfinite(m.z)||!std::isfinite(m.temperature)) {
        have_data=false; ++diag.invalid_samples; diag.stage=10; return false;
    }
    if(m.x*m.x+m.y*m.y+m.z*m.z>2000.0f*2000.0f||m.temperature< -40||m.temperature>85) {
        have_data=false; ++diag.invalid_samples; diag.stage=11; return false;
    }
    if(!diag.self_test_ok) { have_data=false; ++diag.invalid_samples; diag.stage=13; return false; }
    out=diag.last_sample;
    last_ms=hal::millis(); have_data=true; diag.stage=0; ++diag.samples; return true;
}
bool healthy() { return ready && have_data && uint32_t(hal::millis()-last_ms)<200; }
bool communicating() { return ready && have_rx && uint32_t(hal::millis()-last_rx_ms)<200; }
const Diagnostics& diagnostics() { return diag; }
}
