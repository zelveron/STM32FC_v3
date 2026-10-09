// Real Bosch API: distinguish missing I2C transfers, wrong ID and other address.
#include "../src/drivers/bmm350.hpp"
#include "../src/hal/hal.hpp"
#include <bmm350.h>
#include <cstdio>
#include <cstring>
#include <cmath>
bool config_ok=true;
int identity14=-1,identity15=-1,transfers=0,checks=0,failures=0;
bool model=false,data_ready=true;
bool self_test_response=true,wrong_config=false;
int init_pass=-1;
bool otp_changes=false;
uint32_t now=0; uint8_t regs[256]{},otp_word=0;
int fault_reg=-1,fault_word=-1,otp_bad_word=-1;
hal::Status injected=hal::Status::timeout;
int32_t raw[4]={4200,-2800,5600,52000};
namespace hal {
uint32_t millis(){return now;}
void delay_us(uint32_t){}
Status i2c_config(I2cBus bus,uint32_t hz) {
    init_pass=-1;
    return config_ok && bus==I2cBus::mag && hz==400000 ? Status::ok:Status::error;
}
Status i2c_write_read(I2cBus bus,uint8_t addr,const uint8_t* wr,size_t wn,uint8_t* rd,size_t rn) {
    ++transfers;
    if(bus!=I2cBus::mag || !wn) return Status::error;
    const int id=addr==0x14?identity14:addr==0x15?identity15:-1;
    if(id<0) return Status::error;
    if(model && addr==0x14) {
        const uint8_t reg=wr[0];
        if(reg==fault_reg&&(fault_word<0||otp_word==fault_word)) return injected;
        if(rn) {
            if(rn<3) return Status::error;
            rd[0]=0xde;rd[1]=0xad;
            for(size_t i=2;i<rn;++i)rd[i]=regs[uint8_t(reg+i-2)];
            if(reg==BMM350_REG_CHIP_ID)rd[2]=uint8_t(id);
            if(reg==BMM350_REG_OTP_STATUS_REG)rd[2]=1|(otp_word==otp_bad_word?0x20:0);
            // Distinct MSB/LSB bytes and a signed compensation coefficient.
            const uint16_t otp_value=otp_word==0?0x1234:otp_word==13?0xff01:0;
            if(reg==BMM350_REG_OTP_DATA_MSB_REG)rd[2]=otp_value>>8;
            if(reg==BMM350_REG_OTP_DATA_LSB_REG)rd[2]=uint8_t(otp_value)+(otp_changes&&init_pass==1&&otp_word==14?1:0);
            if(reg==BMM350_REG_INT_STATUS)rd[2]=data_ready?BMM350_DRDY_DATA_REG_MSK:0;
            if(reg==BMM350_REG_PMU_CMD_AGGR_SET&&wrong_config)rd[2]=0;
            if(reg==BMM350_REG_MAG_X_XLSB) {
                if(rn!=14)return Status::error;
                int32_t measured[4];std::memcpy(measured,raw,sizeof(raw));
                if(self_test_response)switch(regs[BMM350_REG_TMR_SELFTEST_USER]) {
                    case BMM350_SELF_TEST_POS_X:measured[0]+=30000;break;
                    case BMM350_SELF_TEST_NEG_X:measured[0]-=30000;break;
                    case BMM350_SELF_TEST_POS_Y:measured[1]+=30000;break;
                    case BMM350_SELF_TEST_NEG_Y:measured[1]-=30000;break;
                }
                for(unsigned i=0;i<4;++i)for(unsigned b=0;b<3;++b)rd[2+i*3+b]=uint32_t(measured[i])>>(8*b);
            }
        } else {
            if(reg==BMM350_REG_CMD&&wr[1]==BMM350_CMD_SOFTRESET)++init_pass;
            for(size_t i=1;i<wn;++i)regs[uint8_t(reg+i-1)]=wr[i];
            if(reg==BMM350_REG_PMU_CMD)regs[BMM350_REG_PMU_CMD_STATUS_0]=((wr[1]==8?7:wr[1])<<5)|(wr[1]==1?8:0);
            if(reg==BMM350_REG_OTP_CMD_REG)otp_word=wr[1]&31;
        }
        return Status::ok;
    }
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
    config_ok=model=true;identity14=BMM350_CHIP_ID;
    check("complete Bosch initialization succeeds with register/OTP model",mag350::begin()&&mag350::diagnostics().initialized);
    check("three identical complete OTP snapshots preserve word byte order",mag350::diagnostics().otp_passes==3&&mag350::diagnostics().otp_mismatch==0&&mag350::diagnostics().otp[0][0]==0x1234&&mag350::diagnostics().otp[2][13]==0xff01);
    check("factory trim snapshot preserves signed coefficient decoding",mag350::diagnostics().trim[0]==0.2f&&mag350::diagnostics().trim[4]==-1.0f/512);
    check("Bosch self-test passes and normal 25 Hz configuration is restored",mag350::diagnostics().self_test_ok&&mag350::diagnostics().self_test_x>400&&regs[BMM350_REG_TMR_SELFTEST_USER]==0&&mag350::diagnostics().aggr==0x36&&mag350::diagnostics().pmu==0x28);
    check("one coherent burst removes dummy bytes and yields compensated data",mag350::poll(sample)&&mag350::healthy()&&mag350::communicating()&&sample.x_ut>20&&sample.x_ut<40&&sample.y_ut<0&&mag350::diagnostics().raw[1]==-2800);
    now+=200;check("live sample validity and communication both expire",!mag350::healthy()&&!mag350::communicating());
    fault_reg=BMM350_REG_MAG_X_XLSB;
    check("data timeout immediately vetoes old data and records exact transport status",!mag350::poll(sample)&&!mag350::healthy()&&mag350::diagnostics().stage==9&&mag350::diagnostics().last_bus_status==int8_t(hal::Status::timeout));
    int calls=transfers;now+=99;mag350::poll(sample);
    check("no tight-loop retries during first 100 ms backoff",transfers==calls);
    fault_reg=-1;now+=1;
    check("transient read failure recovers without reset or reboot",mag350::poll(sample)&&mag350::healthy()&&mag350::diagnostics().recoveries==1&&mag350::diagnostics().consecutive_errors==0);
    fault_reg=BMM350_REG_INT_STATUS;injected=hal::Status::nack;
    mag350::poll(sample);now+=100;mag350::poll(sample);calls=transfers;now+=199;mag350::poll(sample);
    check("repeated faults back off exponentially and preserve NACK reason",transfers==calls&&mag350::diagnostics().consecutive_errors==2&&mag350::diagnostics().last_bus_status==int8_t(hal::Status::nack));
    fault_reg=-1;now+=1;data_ready=false;
    check("not-ready response does not revive old magnetic data",!mag350::poll(sample)&&!mag350::healthy());
    now+=100;data_ready=true;check("new ready sample recovers after not-ready interval",mag350::poll(sample)&&mag350::healthy());
    raw[0]=-550000;
    check("out-of-range field is live communication but never valid heading input",!mag350::poll(sample)&&mag350::communicating()&&!mag350::healthy()&&mag350::diagnostics().stage==11&&mag350::diagnostics().invalid_samples==1);
    raw[0]=4200;check("plausible samples recover from range rejection",mag350::poll(sample)&&mag350::healthy());
    fault_reg=BMM350_REG_INT_STATUS;injected=hal::Status::timeout;now=0xfffffff0u;mag350::poll(sample);now+=100;fault_reg=-1;
    check("retry deadlines survive uint32 millisecond wrap",mag350::poll(sample)&&mag350::healthy());
    fault_reg=BMM350_REG_OTP_DATA_MSB_REG;fault_word=4;
    check("early OTP transfer failure cannot be hidden by final successful OTP word",!mag350::begin()&&mag350::diagnostics().stage==12&&mag350::diagnostics().bus_errors>0);
    fault_reg=fault_word=-1;otp_bad_word=4;
    check("early OTP status error cannot silently corrupt compensation",!mag350::begin()&&mag350::diagnostics().stage==12&&mag350::diagnostics().otp_error!=0);
    otp_bad_word=-1;self_test_response=false;
    check("failed self-test preserves live diagnostics but forbids valid measurements",mag350::begin()&&!mag350::poll(sample)&&mag350::communicating()&&!mag350::healthy()&&mag350::diagnostics().stage==13);
    self_test_response=true;wrong_config=true;
    check("configuration readback mismatch prevents acquisition",!mag350::begin()&&mag350::diagnostics().stage==14);
    wrong_config=false;fault_reg=BMM350_REG_TMR_SELFTEST_USER;
    check("self-test transport failure cannot enable acquisition",!mag350::begin()&&mag350::diagnostics().stage==13&&!mag350::healthy());
    fault_reg=-1;otp_changes=true;
    check("one changed OTP word across boot reads prevents using unstable compensation",!mag350::begin()&&mag350::diagnostics().stage==12&&mag350::diagnostics().otp_passes==3&&mag350::diagnostics().otp_mismatch==(1u<<14));
    std::printf("%d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
