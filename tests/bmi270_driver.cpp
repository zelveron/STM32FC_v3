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
struct Chip { uint8_t regs[128]{};std::deque<uint8_t> fifo;uint32_t time=0;bool missing=false,frozen=false; };
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
            else v=c.regs[(addr+i-2)&127];
            if(rx)rx[i]=v;
        }
    } else {
        if(addr==0x7e && len>1 && tx[1]==0xb0)c.fifo.clear();
        if(addr!=0x5e)for(size_t i=1;i<len;++i)c.regs[(addr+i-1)&127]=tx[i];
    }
    return Status::ok;
}
}
void reset_chips() {
    for(auto& c:chip) {c={};c.regs[0]=0x24;c.regs[0x21]=1;c.regs[0x49]=0x10;c.regs[0x7c]=1;}
}
void sample(unsigned i,int16_t gx=0,int16_t gy=0,int16_t gz=0,int16_t ax=0,int16_t ay=0,int16_t az=4096) {
    for(int16_t v:{gx,gy,gz,ax,ay,az}) {chip[i].fifo.push_back(uint8_t(v));chip[i].fifo.push_back(uint8_t(uint16_t(v)>>8));}
}
int checks=0,fails=0;
void check(const char* name,bool ok){++checks;fails+=!ok;std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);}
int main() {
    reset_chips();imu270::Device d;
    check("Bosch config upload / dual-sensor setup succeeds",d.begin(hal::SpiBus::imu,0));
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

    reset_chips();check("two real driver instances initialize",imu_v2::begin());
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
