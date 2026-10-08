// Real Bosch API: distinguish missing I2C transfers, wrong ID and other address.
#include "../src/drivers/bmm350.hpp"
#include "../src/hal/hal.hpp"
#include <bmm350.h>
#include <cstdio>
#include <cstring>
bool config_ok=true;
int identity14=-1,identity15=-1,transfers=0,checks=0,failures=0;
namespace hal {
uint32_t millis(){return 0;}
void delay_us(uint32_t){}
Status i2c_config(I2cBus bus,uint32_t hz) {
    return config_ok && bus==I2cBus::mag && hz==100000 ? Status::ok:Status::error;
}
Status i2c_write_read(I2cBus bus,uint8_t addr,const uint8_t* wr,size_t wn,uint8_t* rd,size_t rn) {
    ++transfers;
    if(bus!=I2cBus::mag || !wn) return Status::error;
    const int id=addr==0x14?identity14:addr==0x15?identity15:-1;
    if(id<0) return Status::error;
    if(rn) {
        if(wr[0]!=BMM350_REG_CHIP_ID || rn!=3) return Status::error;
        rd[0]=0xde;rd[1]=0xad;rd[2]=uint8_t(id);
    }
    return Status::ok;
}
}
void check(const char* name,bool ok){++checks;failures+=!ok;std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);}
int main() {
    check("missing magnetometer fails initialization",!mag350::begin());
    auto d=mag350::diagnostics();
    check("Bosch device-not-found preserves transport failure evidence",d.stage==2&&d.result==BMM350_E_DEV_NOT_FOUND&&d.bus_errors>0&&d.last_error_register==BMM350_REG_CMD);
    check("both failed identity transfers are distinct from chip ID zero",d.id14==-1&&d.id15==-1&&!d.initialized&&!mag350::healthy());
    identity15=BMM350_CHIP_ID;
    check("other-address device is reported without silently changing configuration",!mag350::begin()&&mag350::diagnostics().id15==BMM350_CHIP_ID&&mag350::diagnostics().id14==-1);
    const int before=transfers;mag350::Sample sample{};
    check("failed device is not probed repeatedly by runtime polling",!mag350::poll(sample)&&transfers==before);
    identity14=0x42;identity15=-1;
    check("wrong identity is preserved after removing two dummy bytes",!mag350::begin()&&mag350::diagnostics().chip_id==0x42&&mag350::diagnostics().id14==0x42);
    check("wrong identity with working transport has no stale bus error",mag350::diagnostics().bus_errors==0);
    config_ok=false;
    check("bus setup failure resets previous diagnostics",!mag350::begin()&&mag350::diagnostics().stage==1&&mag350::diagnostics().id14==-1&&mag350::diagnostics().samples==0);
    std::printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
