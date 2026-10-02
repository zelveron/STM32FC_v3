// Regression tests: real application/control/RC/GPS code; mocked physical devices.
// Hardware, clock, USB and SD are simulated. These tests are NOT hardware qualification.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <algorithm>
#include "../src/main_stm32.cpp"
#include "../src/core/imu_selection.hpp"
#include "../src/estimation/imu_rotation.hpp"

namespace mock {
uint32_t now_us = 0;
uint8_t spi_fill = 0;
bool spi_fail = false;
std::vector<uint8_t> rx[2];
std::vector<uint8_t> tx[2];
uint32_t baud[2]{};
uint16_t pwm[8]{};
bool rebooted = false;
std::string usb_command;
Print sink;
bool imu_ok=true,imu_bias=true; unsigned sd_calls=0;
bool bmp_sample = false;
float bmp_pa = 101325.0f;
int repros = 0;
int observations = 0;
void at_ms(uint32_t t) { now_us = t * 1000; }
void observed(const char* name, bool reproduced) {
    ++observations;
    std::printf("[%s] %s\n", reproduced ? "REPRODUCED" : "NOT REPRODUCED", name);
    if (reproduced) ++repros;
}
uint8_t crc8(const std::vector<uint8_t>& p) {
    uint8_t crc = 0;
    for (uint8_t b : p) {
        crc ^= b;
        for (int j=0;j<8;++j) crc = (crc & 128) ? (uint8_t)((crc<<1)^0xd5) : (uint8_t)(crc<<1);
    }
    return crc;
}
void frame(uint8_t type, std::vector<uint8_t> payload) {
    payload.insert(payload.begin(),type);
    auto &q=rx[(int)hal::Uart::crsf];
    q.push_back(0xc8);q.push_back((uint8_t)(payload.size()+1));
    q.insert(q.end(),payload.begin(),payload.end());q.push_back(crc8(payload));
    crsf::poll();
}
void rc(bool arm, int throttle_raw=192, int mode_raw=192) {
    uint16_t ch[16];for(auto &v:ch)v=992;
    ch[2]=(uint16_t)throttle_raw;ch[4]=arm?1792:192;ch[6]=(uint16_t)mode_raw;
    std::vector<uint8_t> p(22,0);
    for(int i=0;i<16;++i)for(int b=0;b<11;++b)if(ch[i]&(1<<b))p[(11*i+b)/8]|=1<<((11*i+b)%8);
    frame(0x16,p);
}
void nmea(const std::string& body) {
    uint8_t sum=0;for(char c:body)sum^=(uint8_t)c;
    char tail[8];std::snprintf(tail,sizeof(tail),"*%02X\r\n",sum);
    auto line=std::string("$")+body+tail;auto &q=rx[(int)hal::Uart::gps];
    q.insert(q.end(),line.begin(),line.end());ublox::poll();
}
void configure_assist(modes::ModeAssist& m) {
    control::AttitudeCtrlConfig ac;control::RateCtrlConfig rc;
    rc.sample_hz=400;rc.roll.kff=.006f;rc.roll.kp=.010f;rc.roll.ki=.02f;rc.roll.i_max=.4f;
    rc.pitch=rc.roll;rc.pitch.kff=.010f;rc.pitch.kp=.020f;
    rc.yaw=rc.roll;rc.yaw.kff=.004f;rc.yaw.kp=.006f;rc.yaw.ki=0;
    m.configure(ac,rc,.70f,.45f);
}
}

const hal::PinId hal::pins::imu_cs=0;
const hal::PinId hal::pins::imu2_cs=1;
uint32_t micros(){return mock::now_us;}
uint32_t millis(){return mock::now_us/1000;}
namespace hal {
void init(){} bool board_configured(){return true;}
uint32_t micros(){return ::micros();} uint32_t millis(){return ::millis();}
uint32_t cycles(){return mock::now_us;} uint32_t cpu_hz(){return 1000000;}
void delay_ms(uint32_t n){mock::now_us+=n*1000;}void delay_us(uint32_t n){mock::now_us+=n;}
void watchdog_start(uint32_t){}void watchdog_kick(){}
void gpio_config(PinId,PinMode){}void gpio_write(PinId,bool){}bool gpio_read(PinId){return false;}
Status spi_config(SpiBus,uint32_t,uint8_t){return Status::ok;}
Status spi_xfer(SpiBus,PinId,const uint8_t*,uint8_t* rx,size_t n){if(rx)std::memset(rx,mock::spi_fill,n);return mock::spi_fail?Status::error:Status::ok;}
Status spi_xfer_async(SpiBus,PinId,const uint8_t*,uint8_t*,size_t){return Status::unsupported;}
bool spi_busy(SpiBus){return false;}
Status i2c_config(I2cBus,uint32_t){return Status::ok;}
Status i2c_write_read(I2cBus,uint8_t,const uint8_t*,size_t,uint8_t*,size_t){return Status::error;}
Status uart_config(Uart p,uint32_t b){mock::baud[(int)p]=b;return Status::ok;}
size_t uart_rx_available(Uart p){return mock::rx[(int)p].size();}
size_t uart_read(Uart p,uint8_t* b,size_t max){auto&q=mock::rx[(int)p];size_t n=std::min(max,q.size());std::copy_n(q.begin(),n,b);q.erase(q.begin(),q.begin()+n);return n;}
size_t uart_write(Uart p,const uint8_t*b,size_t n){auto&q=mock::tx[(int)p];q.insert(q.end(),b,b+n);return n;}
size_t uart_write_space(Uart){return 4096;}bool uart_tx_idle(Uart){return true;}
Status pwm_config(PwmGroup,uint32_t){return Status::ok;}
Status pwm_write_us(PwmGroup g,uint8_t c,uint16_t u){mock::pwm[(g==PwmGroup::ailerons?0:g==PwmGroup::tail?2:6)+c]=u;return Status::ok;}
Status blk_init(){return Status::unsupported;}uint32_t blk_sector_count(){return 0;}
Status blk_read(uint32_t,uint8_t*,uint32_t){return Status::unsupported;}
Status blk_write(uint32_t,const uint8_t*,uint32_t){return Status::unsupported;}
ResetCause reset_cause(){return ResetCause::power_on;}
void jump_to_bootloader(){mock::rebooted=true;throw 7;}
}
namespace usb_stream {
void begin(){}bool host_ready(){return true;}Print& log(){return mock::sink;}
uint32_t drops(){return 0;}int read(){if(mock::usb_command.empty())return -1;int c=mock::usb_command[0];mock::usb_command.erase(0,1);return c;}
}
namespace sd_bin_log {
Result begin(core::LogRing&){return Result::ok;}bool ok(){return true;}
const char* name(){return "MOCK";}uint32_t bytes_written(){return 0;}
uint32_t flush_step(uint32_t){++mock::sd_calls;return 0;}void sync(){++mock::sd_calls;}
}
namespace bmp581 {
int error(){return 0;}
bool begin(){return true;} bool healthy(){return true;}
bool poll(Sample& s){if(!mock::bmp_sample)return false;mock::bmp_sample=false;s.pressure_pa=mock::bmp_pa;s.temp_c=20;return true;}
}


namespace imu_v2 {
bool begin(){return true;} void poll(bool){} void set_observer(Observer){}
bool healthy(){return mock::imu_ok && ahrs::valid();}
bool bias_ready(){return mock::imu_bias;}
bool sensor_healthy(unsigned){return mock::imu_ok;}
bool ambiguous(){return false;} unsigned active(){return 0;}
int driver_error(unsigned){return 0;}
uint32_t driver_health_registers(unsigned){return 0;}
const estimation::ImuSample& latest(){static estimation::ImuSample s{};return s;}
}
namespace mag350 {
bool begin(){return true;} bool poll(Sample&){return false;} bool healthy(){return false;}
}
int failures=0,checks=0;
void check(const char* name,bool condition) {
    ++checks; if(!condition)++failures;
    std::printf("[%s] %s\n",condition?"PASS":"FAIL",name);
}
void step(bool send_rc,bool arm,int thr=192,int mode=192) {
    mock::now_us+=2500;
    if(send_rc)mock::rc(arm,thr,mode);
    task_control();
}
void run(unsigned ms,bool send_rc,bool arm,int thr=192,int mode=192) {
    for(unsigned n=0;n<ms*1000/2500;++n)step(send_rc,arm,thr,mode);
}
int main() {
    using namespace mock;
    s_log=&sink; s_output_ready=true; crsf::begin(420000);
    configure_assist(s_mode_assist);
    control::PidGains tk;tk.kff=.006f;tk.kp=.01f;tk.ki=.02f;tk.i_max=.4f;
    s_mode_takeoff.configure(tk,400,110,120,.175f);
    ahrs::reset(); ahrs::update(0,std::sin(.3f),std::cos(.3f),0,0,0,.0025f);
    check("RC never received is unhealthy",!crsf::receiving());
    if(config::enable_sd_logging) {
        task_log_flush();
        check("optional SD logger is exercised before arming",sd_calls>0);
        sd_calls=0;
    }
    run(400,true,false); step(true,true); step(true,true,1152);
    if(!config::flight_enabled) {
        check("default build inhibits motor arming",!s_arming.armed());
        check("default build holds both ESCs at idle",pwm[6]==1000&&pwm[7]==1000);
        std::printf("%d checks, %d failures\n",checks,failures);return failures?1:0;
    }
    check("valid deliberate arming applies pilot throttle",s_arming.armed()&&pwm[6]==1600);
    usb_command="\ndfu\n";task_cmd();check("DFU rejected while armed",!rebooted);
    // Continue real link-statistics packets while channels cease.
    for(unsigned n=0;n<79;++n){frame(0x14,std::vector<uint8_t>(10,0));step(false,true);}
    check("last valid RC is usable just before age timeout",s_arming.armed());
    step(false,true);
    check("200 ms RC age cuts both motors",s_failsafe.active()&&pwm[6]==1000&&pwm[7]==1000);
    usb_command="dfu\n";task_cmd();check("DFU rejected during airborne RC failsafe",!rebooted);
    run(100,false,true);
    check("healthy failsafe commands roll toward level",s_mode_cur==modes::Id::assist&&pwm[0]<1500);
    check("failsafe keeps flight session latched",s_flight_session);
    run(290,true,true,1152);
    check("recovery does not restore throttle before stable window",!s_arming.armed()&&pwm[6]==1000);
    run(20,true,true,1152);
    check("requested automatic recovery restores current throttle",s_arming.armed()&&pwm[6]==1600);
    run(220,false,true);
    imu_ok=false;step(false,true);
    check("failsafe with failed estimator centers surfaces",pwm[0]==1500&&pwm[2]==1500&&pwm[6]==1000);
    check("IMU fault demotion is latched",s_assist_lockout);
    run(320,true,false,192);
    check("CH5 low on recovery cancels prior arming",!s_arming.armed());
    imu_ok=true;step(true,false,192,992);
    check("recovered IMU does not silently reenter assist",s_mode_cur==modes::Id::manual);
    bmp_sample=true;bmp_pa=101325;s_flight_session=false;task_bmp();
    s_flight_session=true;bmp_sample=true;bmp_pa=90000;task_bmp();
    check("airborne disarm preserves barometric ground reference",g_alt_agl_m>900);
    task_log_flush();check("asynchronous SD service continues after arming",sd_calls>0);
    usb_command="\ndfu\r\n";try{task_cmd();}catch(int){}
    check("DFU accepted after explicit disarm and idle on fresh link",rebooted);
    rebooted=false;
    const bool flying_before=s_flying;
    usb_command="SIM_FLYING 1\n";task_cmd();check("simulation override removed from flight commands",s_flying==flying_before);
    s_flight_session=false;usb_command=std::string(24,'x')+"REBOOT_BL\n";
    task_cmd();check("overflowed USB line cannot execute its suffix",!rebooted);
    usb_command=std::string("dfu\0extra\n",10);task_cmd();
    check("binary USB command cannot execute DFU prefix",!rebooted);
    rc(true);usb_command="dfu\n";task_cmd();
    check("disarmed DFU rejects high CH5",!rebooted);
    rc(false,1152);usb_command="dfu\n";task_cmd();
    check("disarmed DFU rejects raised throttle",!rebooted);
    rc(false);s_flying=true;usb_command="dfu\n";task_cmd();
    check("DFU rejects flight eligibility even if disarmed",!rebooted);s_flying=false;
    now_us+=250000;s_flight_session=true;usb_command="dfu\n";task_cmd();
    check("DFU rejects stale RC after flight session",!rebooted);
    s_flight_session=false;
    usb_command="REBOOT_BL\n";try{task_cmd();}catch(int){}
    check("legacy bootloader alias works before first arm without RC",rebooted);

    // Independent arming policy, including cold boot and recovery provenance.
    core::Arming arm;
    arm.update({true,0,false});check("cold boot with CH5 high cannot arm",!arm.armed());
    arm.update({false,0,false});arm.update({true,.9f,false});
    check("initial arming needs idle throttle",!arm.armed());
    arm.update({false,0,false});arm.update({true,0,false});
    arm.update({true,.7f,true});arm.update({true,.7f,false});
    check("automatic recovery retains only previous authorization",arm.armed());
    arm.update({true,.7f,true});arm.update({false,.7f,false});
    check("disarm cancels automatic recovery",!arm.armed());
    core::Arming never;
    never.update({true,.7f,true});never.update({true,.7f,false});
    check("RC outage cannot arm a previously disarmed aircraft",!never.armed());
    arm.update({false,0,false});arm.update({true,0,false});
    arm.update({true,.7f,true});arm.update({true,.7f,true,false});arm.update({true,.7f,false});
    check("permission fault cancels pending automatic resume",!arm.armed());

    // Parser reset, malformed frames, independent RC freshness.
    crsf::begin(420000);frame(0x14,std::vector<uint8_t>(10,1));
    check("statistics alone never establish RC",!crsf::receiving());
    frame(0x16,std::vector<uint8_t>(21,0));
    check("short RC payload never establishes RC",!crsf::receiving());
    rc(false);check("full channel payload establishes RC",crsf::receiving());
    now_us+=201000;frame(0x99,std::vector<uint8_t>(22,0));
    check("unknown valid traffic does not refresh channels",!crsf::receiving());
    rx[1]={0xc8,62,1};crsf::poll();now_us+=25000;rc(false);
    check("partial frame timeout allows next valid packet",crsf::receiving());
    crsf::begin(420000);check("begin clears old RC authorization",!crsf::receiving());
    rx[1]=std::vector<uint8_t>(10000,0x33);crsf::poll();
    check("CRSF drain has a finite byte budget",rx[1].size()==10000-384);rx[1].clear();

    ublox::begin(38400);
    nmea("GNGGA,120000.00,4101.12345,N,02901.54321,E,1,12,0.8,123.4,M,0,M,,");
    nmea("GNRMC,120000.00,A,4101.12345,N,02901.54321,E,20.0,123.4,280926,,,A");
    check("SAM-M10Q NMEA yields valid fix/speed/course",ublox::locked()&&ublox::speed_valid()&&std::fabs(ublox::course_deg()-123.4f)<.01);
    check("GNSS double precision preserves decimal minutes",std::fabs(ublox::lat_deg()-(41+1.12345/60))<1e-9);
    nmea("GNRMC,120001.00,V,4101.12345,N,02901.54321,E,90.0,123.4,280926,,,N");
    check("RMC void invalidates navigation data",!ublox::locked()&&!ublox::speed_valid());
    nmea("GNGGA,120002.00,4101.12345,N,02901.54321,E,0,12,0.8,123.4,M,0,M,,");
    check("GGA no-fix cannot produce usable coordinates",!ublox::locked());
    nmea("GNGGA,120003.00,4161.0,N,02901.54321,E,1,12,0.8,123.4,M,0,M,,");
    check("invalid latitude minutes rejected",!ublox::locked());
    nmea("GNGGA,120004.00,4101.12345,N,02901.54321,E,1,12,0.8,123.4,M,0,M,,");
    now_us+=2100000;ublox::poll();
    check("stale GPS fix expires",!ublox::locked()&&ublox::fix()==0);
    check("GPS discovery never writes a conflicting baud command",tx[0].empty());
    now_us+=3100000;ublox::poll();
    check("silent receiver restarts passive baud detection",ublox::current_baud()!=38400);
    ublox::begin(115200);check("selected host baud tracked accurately",ublox::current_baud()==115200);

    estimation::ImuPrep prep;prep.configure(400,30,15,4);
    for(int n=0;n<1800;++n)prep.process(0,0,30,0,0,1,.0025f);
    check("constant rotation rejected as gyro calibration",!prep.bias_ready());
    for(int n=0;n<1800;++n)prep.process(0,0,0,0,0,0,.0025f);
    check("invalid gravity rejected during calibration",!prep.bias_ready());
    for(int n=0;n<1800;++n)prep.process(1,-.5f,.2f,0,0,1,.0025f,false);
    check("calibration cannot run after flight authorization",!prep.bias_ready());
    for(int n=0;n<1800;++n)prep.process(1,-.5f,.2f,0,0,1,.0025f);
    check("stationary valid data calibrates",prep.bias_ready());
    float bx,by,bz;prep.gyro_bias(bx,by,bz);
    check("independent calibrated gyro bias",std::fabs(bx-1)<.001&&std::fabs(by+.5)<.001);
    const int8_t mapping[3]={1,-2,-3},reflection[3]={1,2,-3};
    check("mounting transform is a rotation, not reflection",estimation::rotation_valid(mapping)&&!estimation::rotation_valid(reflection));
    float body[3];const float sensor[3]={0,0,1};estimation::rotate(mapping,sensor,body);
    ahrs::reset();ahrs::update(-body[0],-body[1],-body[2],0,0,0,.0025f);
    check("PCB orientation produces level attitude",ahrs::valid()&&std::fabs(ahrs::roll_rad())<.001);
    ahrs::update(0,0,1,std::numeric_limits<float>::quiet_NaN(),0,0,.0025f);
    check("NaN invalidates estimator without poisoning quaternion",!ahrs::valid()&&std::isfinite(ahrs::roll_rad()));
    ahrs::update(0,0,1,0,0,0,.0025f);
    ahrs::update(0,0,1,0,0,0,.1f);check("large estimator time gap is rejected",!ahrs::valid());

    core::ImuSelection sel;
    sel.update(true,true,false,0);check("primary selected with two healthy IMUs",sel.healthy()&&sel.active()==0);
    sel.update(false,true,false,10);check("failed primary switches to healthy backup",sel.healthy()&&sel.active()==1);
    sel.update(true,true,false,20);check("primary recovery does not oscillate selection",sel.active()==1);
    sel.update(true,true,true,30);sel.update(true,true,true,129);
    check("brief IMU disagreement is debounced",sel.healthy());
    sel.update(true,true,true,130);check("persistent two-IMU disagreement is ambiguous",!sel.healthy()&&sel.ambiguous());
    sel.update(true,true,false,140);check("ambiguous IMUs require restart/inspection",!sel.healthy());
    core::ImuSelection dead;dead.update(false,false,false,0);check("no healthy IMU inhibits stabilization",!dead.healthy());

    // Production-gain transition sweep at 400 Hz, including I-enable edge.
    bool continuity=true,finite=true,throttle_live=true;
    for(float bank:{-.9f,-.5235988f,0.f,.5235988f,.9f}) {
        modes::ModeAssist mode;configure_assist(mode);
        control::Outputs current;current.ch[0]=current.ch[1]=.6f;
        current.ch[2]=current.ch[3]=-.2f;mode.enter(current);
        modes::ModeInput in;in.dt_s=.0025f;in.roll_rad=bank;in.sticks.throttle=.7f;
        in.allow_integrators=false;control::Outputs out;
        mode.update(in,out);continuity&=std::fabs(out.ch[0]-.6f)<1e-6;
        for(int n=0;n<400;++n) {
            const auto prev=out;in.allow_integrators=n>100;mode.update(in,out);
            for(int i=0;i<6;++i) {continuity&=std::fabs(out.ch[i]-prev.ch[i])<=.03001f;finite&=std::isfinite(out.ch[i]);}
            throttle_live&=std::fabs(out.ch[6]-.7f)<1e-6;
        }
    }
    check("ASSIST entry/I-enable obeys configured 15 us per control tick ceiling",continuity);
    check("surface slew preserves live pilot throttle",throttle_live&&finite);
    modes::ModeTakeoff takeoff;takeoff.configure(tk,400,110,120,.175f);
    control::Outputs prev;prev.ch[0]=.5;takeoff.enter(prev);
    modes::ModeInput in;in.dt_s=.0025f;in.roll_rad=.8f;in.sticks={0,.6f,-.4f,.7f};
    control::Outputs out;takeoff.update(in,out);
    check("TKOFF smooths roll while retaining pilot pitch/yaw/throttle",out.ch[0]==.5f&&out.ch[2]==.6f&&out.ch[4]==-.4f&&out.ch[6]==.7f);
    control::SrvChannel servo;
    check("nonfinite actuator commands produce defined pulses",servo.from_norm(NAN)==1500&&servo.from_unipolar(NAN)==1000);
    core::Failsafe wrap;wrap.update(true,0xffffff00);wrap.update(true,0x30);
    check("failsafe recovery window survives uint32 wrap",!wrap.active());

    // Deliberate missed-control-deadline case after a flight session.
    s_flight_session=true;s_control_fault=false;
    now_us+=50000;task_control();
    check("control deadline fault inhibits motors",s_control_fault&&!s_arming.armed()&&pwm[6]==1000);
    std::printf("%d checks, %d failures\n",checks,failures);return failures?1:0;
}
