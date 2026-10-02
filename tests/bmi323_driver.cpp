// Real Bosch API and application driver, backed by an SPI register/FIFO model.
// Models transport/configuration and sequencing, not MEMS physics.
#include "../src/drivers/bmi323_fifo.hpp"
#include "../src/drivers/imu_v2.hpp"
#include "../src/estimation/ahrs.hpp"
#include <array>
#include <deque>
#include <cstdio>
#include <cstring>
#include <cmath>
struct Chip {
    uint16_t regs[128]{};std::deque<uint8_t> fifo;uint32_t time=0;
    uint32_t spi_ready_at=0;bool select_pending=false,switched=false;
    bool missing=false,frozen=false;
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
    if(now_us<c.spi_ready_at)return Status::error;
    if(c.missing){if(rx)std::memset(rx,0xff,len);return Status::ok;}
    const uint8_t addr=tx[0]&0x7f;
    if(tx[0]&0x80) {
        if(c.select_pending) { c.select_pending=false;c.switched=true;c.spi_ready_at=now_us+200; }
        if(rx)std::memset(rx,0,len);
        if(!c.frozen)c.time+=64;
        c.regs[0x0a]=uint16_t(c.time);c.regs[0x0b]=uint16_t(c.time>>16);
        c.regs[0x15]=uint16_t(c.fifo.size()/2);
        // rx[0] is address-clock response, rx[1] is required dummy byte.
        for(size_t i=2;i<len;++i) {
            uint8_t v=0;
            if(addr==0x16){if(!c.fifo.empty()){v=c.fifo.front();c.fifo.pop_front();}}
            else v=uint8_t(c.regs[(addr+(i-2)/2)&127]>>(((i-2)%2)*8));
            if(rx)rx[i]=v;
        }
        if(addr<=1 && addr+(len-2)/2>1)c.regs[1]&=~0x0904; // clear-on-read interface/feature flags
    } else {
        if(addr==0x7e && len==3 && tx[1]==0xaf && tx[2]==0xde)c.select_pending=true;
        if(addr==0x37 && len>2 && tx[1]==1)c.fifo.clear();
        for(size_t i=1;i+1<len;i+=2)c.regs[(addr+(i-1)/2)&127]=uint16_t(tx[i])|(uint16_t(tx[i+1])<<8);
    }
    return Status::ok;
}
}
void reset_chips() {
    for(auto& c:chip) {c={};c.regs[0]=0x43;c.regs[0x11]=1;}
}
void sample(unsigned i,int16_t gx=0,int16_t gy=0,int16_t gz=0,int16_t ax=0,int16_t ay=0,int16_t az=4096) {
    for(int16_t v:{ax,ay,az,gx,gy,gz}) {chip[i].fifo.push_back(uint8_t(v));chip[i].fifo.push_back(uint8_t(uint16_t(v)>>8));}
}
int checks=0,fails=0;
void check(const char* name,bool ok){++checks;fails+=!ok;std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);}
int main() {
    reset_chips();imu323::Device d;
    check("BMI323 Bosch reset / dual-sensor setup succeeds",d.begin(hal::SpiBus::imu,0));
    check("reset dummy read waits 200 us before next SPI access",chip[0].switched&&now_us>=chip[0].spi_ready_at);
    check("FIFO explicitly uses paired accel-first frames with stop-on-full",chip[0].regs[0x36]==0x0601);
    chip[0].regs[1]=0x0800;
    check("startup flush clears historical I3C-interface flag",d.discard_pending()&&chip[0].regs[1]==0);
    imu323::Sample out[8];uint8_t count;
    check("empty FIFO produces no new data",d.read(out,count)==imu323::Result::no_data&&count==0);
    sample(0,16384,0,0,0,0,4096);sample(0,0,16384,0,0,4096,0);
    const auto result=d.read(out,count);
    check("FIFO delivers all paired samples in order",result==imu323::Result::ok&&count==2&&std::fabs(out[0].gx_dps-1000)<.01&&out[0].az_g==1&&out[1].ay_g==1);
    chip[0].frozen=true;sample(0);
    d.read(out,count); // This emulator's earlier FIFO read advanced its clock.
    sample(0);
    check("frozen sensor clock is rejected",d.read(out,count)==imu323::Result::fault);
    reset_chips();check("device can initialize before flight",d.begin(hal::SpiBus::imu,0));
    for(int i=0;i<9;++i)sample(0);
    check("excess FIFO backlog rejected instead of silently decimated",d.read(out,count)==imu323::Result::fault);
    reset_chips();d.begin(hal::SpiBus::imu,0);chip[0].missing=true;
    check("floating MISO cannot appear healthy",d.read(out,count)==imu323::Result::fault);
    reset_chips();d.begin(hal::SpiBus::imu,0);sample(0,32767);
    check("saturated gyro invalidates sample",d.read(out,count)==imu323::Result::fault);

    reset_chips();chip[0].regs[0]=0x24;
    check("BMI270 identity is rejected by BMI323 profile",!d.begin(hal::SpiBus::imu,0));
    reset_chips();d.begin(hal::SpiBus::imu,0);chip[0].regs[1]=0x20;
    check("sensor configuration error latches a fault",d.read(out,count)==imu323::Result::fault);
    reset_chips();d.begin(hal::SpiBus::imu,0);sample(0,0,0,0,0x7f01,-32768,-32768);
    check("BMI323 settling dummy frame cannot become acceleration",d.read(out,count)==imu323::Result::fault&&count==0);
    reset_chips();d.begin(hal::SpiBus::imu,0);for(int n=0;n<8;++n)sample(0);
    check("48 WORD FIFO limit accepts exactly eight paired frames",d.read(out,count)==imu323::Result::ok&&count==8);
    reset_chips();d.begin(hal::SpiBus::imu2,1);sample(1,0,0,0,0,0,-4096);
    check("second bus has independent signed sample conversion",d.read(out,count)==imu323::Result::ok&&out[0].az_g==-1);

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
