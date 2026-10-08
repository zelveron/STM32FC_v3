// Real Bosch API and application driver, backed by an SPI register/FIFO model.
// Models transport/configuration and sequencing, not MEMS physics.
#include "../src/drivers/bmi270.hpp"
#include "../src/drivers/imu_v2.hpp"
#include "../src/estimation/ahrs.hpp"
#include <array>
#include <deque>
#include <cstdio>
#include <cstring>
#include <cmath>
extern const uint8_t bmi270_config_file[];
struct Chip {
    uint8_t regs[128]{};
    std::deque<uint8_t> fifo;
    uint32_t time=0, accessible_us=0, config_ready_us=0;
    uint32_t gyro_enabled_us=0,last_flush_us=0;
    size_t config_bytes=0;
    unsigned resets=0, selections=0;
    bool missing=false,frozen=false,spi=false,corrupt_config=false,config_ok=true;
};
Chip chip[2];
uint32_t now_us=0;
const hal::PinId hal::pins::imu_cs=0;
const hal::PinId hal::pins::imu2_cs=1;
namespace hal {
uint32_t micros(){return now_us;}uint32_t millis(){return now_us/1000;}
void delay_us(uint32_t us){now_us+=us;}void delay_ms(uint32_t ms){now_us+=ms*1000;}
void gpio_write(PinId,bool){}void gpio_config(PinId,PinMode){}
Status spi_config(SpiBus,uint32_t,uint8_t){return Status::ok;}
Status spi_xfer(SpiBus bus,PinId,const uint8_t* tx,uint8_t* rx,size_t len) {
    auto& c=chip[unsigned(bus)];
    if(c.missing){if(rx)std::memset(rx,0xff,len);return Status::ok;}
    if(now_us<c.accessible_us) return Status::error;
    if(!c.spi) {
        c.spi=true; ++c.selections; c.accessible_us=now_us+200;
        if(rx)std::memset(rx,0,len);
        return Status::ok; // First read only selects SPI; response is invalid.
    }
    const uint8_t addr=tx[0]&0x7f;
    if(tx[0]&0x80) {
        if(rx)std::memset(rx,0,len);
        if(!c.frozen)c.time+=64;
        c.regs[0x18]=c.time;c.regs[0x19]=c.time>>8;c.regs[0x1a]=c.time>>16;
        c.regs[0x24]=uint8_t(c.fifo.size());c.regs[0x25]=uint8_t(c.fifo.size()>>8);
        // rx[0] is address-clock response, rx[1] is required dummy byte.
        for(size_t i=2;i<len;++i) {
            uint8_t v=0;
            if(addr==0x26){if(!c.fifo.empty()){v=c.fifo.front();c.fifo.pop_front();}}
            else {
                const auto reg=(addr+i-2)&127;
                v=reg==0x21 && now_us<c.config_ready_us?0:c.regs[reg];
            }
            if(rx)rx[i]=v;
        }
    } else {
        if(addr==0x7e && len>1 && tx[1]==0xb6) {
            const uint8_t id=c.regs[0];
            std::memset(c.regs,0,sizeof(c.regs));
            c.regs[0]=id;c.regs[0x49]=0x10;c.regs[0x7c]=3;
            c.fifo.clear();c.config_bytes=0;c.config_ok=true;c.spi=false;
            c.accessible_us=now_us+2000;++c.resets;
            return Status::ok;
        }
        if(addr==0x5e) {
            const size_t offset=2*(c.regs[0x5b]+16*c.regs[0x5c]);
            c.config_ok &= offset==c.config_bytes && offset+len-1<=8192 &&
                !(c.regs[0x7c]&1) && c.regs[0x59]==0;
            for(size_t i=1;i<len && offset+i-1<8192;++i)
                c.config_ok &= tx[i]==bmi270_config_file[offset+i-1];
            c.config_bytes+=len-1;
        }
        if(addr==0x59 && len>1 && tx[1]==1) {
            c.regs[0x21]=c.config_ok && c.config_bytes==8192 && !c.corrupt_config?1:2;
            c.config_ready_us=now_us+20000;
        }
        if(addr==0x7c && len>1 && (c.regs[0x7c]&1)) c.accessible_us=now_us+450;
        if(addr==0x7d && len>1 && (tx[1]&2) && !(c.regs[0x7d]&2)) {
            c.gyro_enabled_us=now_us;
            // Headerless gyro startup dummy followed by one accel sample.
            for(uint8_t v:{0x02,0x7f,0x00,0x80,0x00,0x80,0,0,0,0,0,0x10}) c.fifo.push_back(v);
        }
        if(addr==0x7e && len>1 && tx[1]==0xb0) { c.fifo.clear(); c.last_flush_us=now_us; }
        if(addr!=0x5e)for(size_t i=1;i<len;++i)c.regs[(addr+i-1)&127]=tx[i];
    }
    return Status::ok;
}
}
void reset_chips() {
    for(auto& c:chip) {c={};c.regs[0]=0x24;c.regs[0x49]=0x10;c.regs[0x7c]=3;}
}
void sample(unsigned i,int16_t gx=0,int16_t gy=0,int16_t gz=0,int16_t ax=0,int16_t ay=0,int16_t az=4096) {
    for(int16_t v:{gx,gy,gz,ax,ay,az}) {chip[i].fifo.push_back(uint8_t(v));chip[i].fifo.push_back(uint8_t(uint16_t(v)>>8));}
}
int checks=0,fails=0;
void check(const char* name,bool ok){++checks;fails+=!ok;std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);}
int main() {
    reset_chips();imu270::Device d;
    check("Bosch config upload / dual-sensor setup succeeds",d.begin(hal::SpiBus::imu,0));
    check("cold boot and post-reset reads select SPI with required delay",chip[0].selections==2&&chip[0].resets==1);
    check("all 8192 Bosch config bytes upload at correct word addresses",chip[0].config_ok&&chip[0].config_bytes==8192);
    check("startup waits for gyro settling before flushing dummy frames",chip[0].last_flush_us-chip[0].gyro_enabled_us>=45000&&chip[0].fifo.empty());
    check("successful initialization reports actual identity/load/error registers",d.error()==0&&d.health_registers()==0x240100);
    check("FIFO explicitly uses paired headerless filtered samples",(chip[0].regs[0x49]&0xd0)==0xc0 && (chip[0].regs[0x45]&0x88)==0x88);
    imu270::Sample out[8];uint8_t count;
    check("empty FIFO produces no new data",d.read(out,count)==imu270::Result::no_data&&count==0);
    sample(0,16384,0,0,0,0,4096);sample(0,0,16384,0,0,4096,0);
    const auto result=d.read(out,count);
    check("FIFO delivers all paired samples in order",result==imu270::Result::ok&&count==2&&std::fabs(out[0].gx_dps-1000)<.01&&out[0].az_g==1&&out[1].ay_g==1);
    chip[0].frozen=true;sample(0);
    d.read(out,count); // This emulator's earlier FIFO read advanced its clock.
    sample(0);
    check("frozen sensor clock is rejected",d.read(out,count)==imu270::Result::fault);
    reset_chips();check("device can initialize before flight",d.begin(hal::SpiBus::imu,0));
    for(int i=0;i<9;++i)sample(0);
    check("excess FIFO backlog rejected instead of silently decimated",d.read(out,count)==imu270::Result::fault);
    reset_chips();d.begin(hal::SpiBus::imu,0);chip[0].missing=true;
    check("floating MISO cannot appear healthy",d.read(out,count)==imu270::Result::fault);
    reset_chips();d.begin(hal::SpiBus::imu,0);sample(0,32767);
    check("saturated gyro invalidates sample",d.read(out,count)==imu270::Result::fault);
    reset_chips();chip[0].missing=true;
    check("missing device reports Bosch identity failure and floating registers",!d.begin(hal::SpiBus::imu,0)&&d.error()==103&&d.health_registers()==0xffffff);
    reset_chips();chip[0].regs[0]=0x43;
    check("Unexpected chip identity cannot be initialized as BMI270",!d.begin(hal::SpiBus::imu,0)&&d.error()==103&&chip[0].config_bytes==0);
    reset_chips();chip[0].corrupt_config=true;
    check("config-load error is preserved instead of generic unavailable",!d.begin(hal::SpiBus::imu,0)&&d.error()==109&&d.health_registers()==0x240200);

    reset_chips();check("two real driver instances initialize",imu_v2::begin());
    check("both buses receive separate complete configuration uploads",chip[0].config_bytes==8192&&chip[1].config_bytes==8192&&chip[0].config_ok&&chip[1].config_ok);
    for(int n=0;n<100;++n){sample(0);sample(1);}
    imu_v2::poll(true);
    check("startup discards FIFO accumulated during other device init",chip[0].fifo.empty()&&chip[1].fifo.empty());
    for(int n=0;n<1800;++n){sample(0);sample(1);now_us+=2500;imu_v2::poll(true);}
    check("dual stationary calibration yields level attitude",imu_v2::healthy()&&imu_v2::bias_ready()&&std::fabs(ahrs::roll_rad())<.001&&std::fabs(ahrs::pitch_rad())<.001);
    chip[0].missing=true;sample(1);now_us+=2500;imu_v2::poll(false);
    check("real primary disconnect selects calibrated second IMU",imu_v2::active()==1&&imu_v2::healthy()&&imu_v2::bias_ready());
    chip[1].missing=true;now_us+=2500;imu_v2::poll(false);
    check("loss of both drivers invalidates attitude eligibility",!imu_v2::healthy());

    reset_chips();imu_v2::begin();
    for(int n=0;n<1800;++n){sample(0);sample(1);now_us+=2500;imu_v2::poll(true);}
    for(int n=0;n<100;++n){sample(0);sample(1,2000);now_us+=2500;imu_v2::poll(false);}
    check("real dual-IMU disagreement is latched after filtering",imu_v2::ambiguous()&&!imu_v2::healthy());
    reset_chips();imu_v2::begin();imu_v2::poll(true);
    for(int n=0;n<100;++n){sample(0);sample(1);now_us+=2500;imu_v2::poll(true);}
    chip[0].missing=true;sample(1);now_us+=2500;imu_v2::poll(false);
    check("uncalibrated backup cannot silently take control",!imu_v2::healthy()&&imu_v2::active()==0);
    reset_chips();imu_v2::begin();imu_v2::poll(true);
    for(int n=0;n<1800;++n){sample(0);sample(1);now_us+=2500;imu_v2::poll(true);}
    now_us+=20001;imu_v2::poll(false);
    check("responsive chips without new FIFO data expire",!imu_v2::healthy());
    std::printf("%d checks, %d failures\n",checks,fails);return fails?1:0;
}
