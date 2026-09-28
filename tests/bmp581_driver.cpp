// Real Bosch API plus application driver with a register-level I2C model.
#include "../src/drivers/bmp581.hpp"
#include "../src/hal/hal.hpp"
#include <bmp5.h>
#include <cstdio>
#include <cstring>
#include <cmath>
uint8_t regs[128]{};
uint32_t now=0;bool fault=false;
namespace hal {
uint32_t millis(){return now;}void delay_us(uint32_t us){now+=us/1000;}
Status i2c_config(I2cBus b,uint32_t hz){return b==I2cBus::baro&&hz==100000?Status::ok:Status::error;}
Status i2c_write_read(I2cBus b,uint8_t addr,const uint8_t* wr,size_t wn,uint8_t* rd,size_t rn) {
    if(fault||b!=I2cBus::baro||addr!=0x47||!wn||wr[0]+rn>128)return Status::error;
    if(rn) {
        std::memcpy(rd,regs+wr[0],rn);
        if(wr[0]==BMP5_REG_INT_STATUS)regs[wr[0]]=0; // read clears flags
    } else for(size_t i=1;i<wn;++i)regs[wr[0]+i-1]=wr[i];
    return Status::ok;
}
}
void reset(){std::memset(regs,0,sizeof(regs));regs[1]=0x50;regs[0x28]=2;regs[0x38]=0x80;regs[0x37]=0x80;fault=false;}
void sample(float pressure=101325,float temperature=20) {
    const uint32_t p=uint32_t(pressure*64),t=uint32_t(int32_t(temperature*65536));
    for(unsigned i=0;i<3;++i){regs[0x1d+i]=uint8_t(t>>(8*i));regs[0x20+i]=uint8_t(p>>(8*i));}
    if(regs[0x15]&1)regs[0x27]|=1;
}
int checks=0,fails=0;
void check(const char* name,bool ok){++checks;fails+=!ok;std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);}
int main(){
    reset();check("BMP581 initializes on correct I2C address",bmp581::begin());
    check("normal 50 Hz configuration and DRDY source enabled",(regs[0x37]&3)==BMP5_POWERMODE_NORMAL&&((regs[0x37]>>2)&31)==BMP5_ODR_50_HZ&&(regs[0x15]&1));
    bmp581::Sample out{};
    check("no fabricated sample before data ready",!bmp581::poll(out)&&!bmp581::healthy());
    sample();check("Bosch conversion returns valid pressure and temperature",bmp581::poll(out)&&std::fabs(out.pressure_pa-101325)<.1&&std::fabs(out.temp_c-20)<.01&&bmp581::healthy());
    check("same DRDY event is not reused",!bmp581::poll(out));
    now+=200;check("pressure health expires after 200 ms",!bmp581::healthy());
    sample(20000);check("out-of-range pressure rejected",!bmp581::poll(out)&&!bmp581::healthy());
    fault=true;check("I2C fault stops normal polling",!bmp581::poll(out)&&!bmp581::healthy());
    fault=false;sample();check("bus recovery does not silently restart failed device",!bmp581::poll(out));
    reset();regs[0x38]=0;check("invalid oversampling/rate combination rejected",!bmp581::begin());
    std::printf("%d checks, %d failures\n",checks,fails);return fails?1:0;
}
