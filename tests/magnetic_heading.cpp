#include "../src/estimation/mag_heading.hpp"
#include "../src/estimation/ahrs.hpp"
#include "../src/modes/mode_assist.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>

namespace {
constexpr float rad=.017453292519943295f;
int checks=0,failures=0;
void check(const char* name,bool ok) { ++checks; failures+=!ok; std::printf("[%s] %s\n",ok?"PASS":"FAIL",name); }
float wrap(float v) { return std::atan2(std::sin(v),std::cos(v)); }
estimation::MagHeadingConfig configured() {
    estimation::MagHeadingConfig c; c.calibrated=c.orientation_confirmed=true; return c;
}
// Independent body->NED matrix; inverse observation of field (30,0,40) uT.
void field(float roll,float pitch,float yaw,float (&out)[3]) {
    const float cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch),cy=std::cos(yaw),sy=std::sin(yaw);
    out[0]=30*cy*cp-40*sp;
    out[1]=30*(cy*sp*sr-sy*cr)+40*cp*sr;
    out[2]=30*(cy*sp*cr+sy*sr)+40*cp*cr;
}
float qualify(estimation::MagHeading& m,const float (&b)[3],float r=0,float p=0,float yaw=0,uint32_t start=0,bool can_align=true) {
    for(unsigned i=0;i<20;++i) yaw=wrap(yaw+m.observe(b[0],b[1],b[2],r,p,yaw,true,can_align,start+40*i));
    return yaw;
}
}
int main() {
    using estimation::MagHeading;
    float b[3]={30,0,40}; MagHeading unconfigured;
    check("online sensor cannot bypass missing installation calibration",unconfigured.observe(30,0,40,0,0,0,true,true,1000)==0&&!unconfigured.aiding(1000,true,true)&&unconfigured.state(1000,true,true)==MagHeading::State::setup);
    check("driver failure takes diagnostic priority over missing calibration",unconfigured.state(1000,true,false)==MagHeading::State::driver);
    auto c=configured(); c.enabled=false; MagHeading disabled(c);
    check("compile-time disabled magnetometer remains gyro-only",disabled.observe(30,0,40,0,0,0,true,true,0)==0&&disabled.state(0,true,true)==MagHeading::State::disabled);
    c=configured(); c.rotation[2]=-3; MagHeading reflection(c);
    check("improper mounting rotation is rejected",!reflection.configured());
    c=configured(); c.correction[2][2]=-1; MagHeading bad_cal(c);
    check("reflected calibration is rejected",!bad_cal.configured());
    c=configured(); c.field_ut=NAN; MagHeading nan_config(c);
    check("nonfinite calibration parameters are rejected",!nan_config.configured());
    bool cardinal=true;
    for(float yaw:{0.f,90.f,180.f,-90.f,179.f,-179.f}) for(float r:{-55.f,0.f,60.f}) for(float p:{-35.f,0.f,40.f}) {
        MagHeading m(configured()); field(r*rad,p*rad,yaw*rad,b);
        const float y=qualify(m,b,r*rad,p*rad);
        cardinal &= std::fabs(wrap(y-yaw*rad))<.0001f && m.heading_valid(760,true,true);
    }
    check("tilt-compensated magnetic cardinal headings and wrap work in FRD/NED",cardinal);
    c=configured(); c.rotation[1]=-2; c.rotation[2]=-3;
    c.offset_ut[0]=10;c.offset_ut[1]=-5;c.offset_ut[2]=7;c.correction[0][0]=2;c.correction[2][2]=.5f;
    MagHeading transformed(c); field(.4f,-.2f,1.f,b);
    float distorted[3]={b[0]/2+10,-b[1]-5,-b[2]*2+7};
    check("hard iron, soft iron and sensor mounting are applied before heading",std::fabs(qualify(transformed,distorted,.4f,-.2f)-1)<.0001f);
    ahrs::reset(); ahrs::update(-std::sin(.3f),std::sin(.5f)*std::cos(.3f),std::cos(.5f)*std::cos(.3f),0,0,0,.0025f);
    ahrs::correct_yaw(2.f);
    check("world yaw correction preserves bank and pitch",std::fabs(ahrs::roll_rad()-.5f)<1e-5f&&std::fabs(ahrs::pitch_rad()-.3f)<1e-5f&&std::fabs(ahrs::yaw_rad()-2)<1e-5f);
    ahrs::correct_yaw(NAN);
    check("invalid yaw observation cannot poison attitude",ahrs::valid());
    MagHeading m(configured()); b[0]=30;b[1]=0;b[2]=40;
    for(unsigned i=0;i<1000;++i) m.observe(30,0,40,0,0,0,true,true,100);
    check("duplicate timestamps cannot qualify the compass",!m.aiding(100,true,true));
    qualify(m,b);
    check("fresh accepted BMM350 becomes heading source",m.heading_valid(760,true,true));
    check("200 ms expiry removes heading eligibility",!m.heading_valid(960,true,true)&&m.state(960,true,true)==MagHeading::State::stale);
    check("driver fault or missing attitude vetoes recent magnetic reference",!m.heading_valid(760,true,false)&&!m.heading_valid(760,false,true));
    check("strong magnetic interference is rejected immediately",m.observe(300,0,400,0,0,0,true,false,800)==0&&!m.aiding(800,true,true)&&m.state(800,true,true)==MagHeading::State::field);
    qualify(m,b,0,0,0,1000);
    check("plausible-magnitude but wrong-direction field is rejected",m.observe(0,-30,40,0,0,0,true,false,1800)==0&&!m.heading_valid(1800,true,true)&&m.state(1800,true,true)==MagHeading::State::innovation);
    check("recovery must requalify instead of accepting one good frame",m.observe(30,0,40,0,0,0,true,false,1840)==0&&!m.heading_valid(1840,true,true));
    qualify(m,b,0,0,0,2000);
    check("stable field recovers after qualification",m.heading_valid(2760,true,true));
    m.invalidate_attitude();
    check("AHRS reset invalidates the old magnetic alignment",!m.aiding(2760,true,true));
    MagHeading weak(configured());
    check("near-vertical field cannot define yaw",weak.observe(1,0,50,0,0,0,true,true,0)==0&&weak.state(0,true,true)==MagHeading::State::field);
    check("NaN magnetic data is rejected",weak.observe(NAN,0,40,0,0,0,true,true,40)==0&&!weak.aiding(40,true,true));
    MagHeading late(configured()); field(0,0,120*rad,b);
    const float late_yaw=qualify(late,b,0,0,0,0,false);
    check("first magnetic acquisition airborne is rate-limited, never a yaw jump",late_yaw>0&&late_yaw<2*rad&&!late.heading_valid(760,true,true)&&late.state(760,true,true)==MagHeading::State::aligning);
    MagHeading wrapping(configured()); b[0]=30;b[1]=0;b[2]=40; qualify(wrapping,b,0,0,0,0xffffff00u);
    check("sample freshness survives millisecond counter wrap",wrapping.heading_valid(uint32_t(0xffffff00u+760),true,true));
    MagHeading drift(configured()); ahrs::reset(); ahrs::update(0,0,1,0,0,0,.0025f);
    for(unsigned i=0;i<24000;++i) {
        ahrs::update(0,0,1,0,0,.5f,.0025f);
        if(i%16==0) ahrs::correct_yaw(drift.observe(30,0,40,0,0,ahrs::yaw_rad(),true,true,i*5/2));
    }
    check("magnetic aiding bounds gyro yaw drift over a minute",std::fabs(ahrs::yaw_rad())<1.2f*rad&&drift.heading_valid(60000,true,true));

    modes::ModeAssist assist; control::RateCtrlConfig rate; control::AttitudeCtrlConfig att;
    rate.roll.kp=.01f;rate.pitch.kp=.02f;rate.yaw.kp=.006f;
    assist.configure(att,rate,.7f,.45f); assist.enter({});
    modes::ModeInput in; control::Outputs out; in.heading_valid=true;in.allow_heading_hold=true;in.yaw_rad=179*rad;
    for(int i=0;i<210;++i) assist.update(in,out);
    check("centered sticks capture heading after settling",assist.heading_hold()&&std::fabs(assist.heading_target_rad()-179*rad)<.001f);
    in.yaw_rad=-179*rad;assist.update(in,out);
    check("heading hold steers the shortest way across north/wrap",assist.roll_target_rad()<0&&std::fabs(assist.roll_target_rad()+rad)<.001f);
    in.yaw_rad=90*rad;in.sticks.throttle=.62f;
    for(int i=0;i<300;++i) assist.update(in,out);
    check("heading hold limits bank to 15 degrees",std::fabs(assist.roll_target_rad()-15*rad)<.001f);
    check("heading hold leaves pilot throttle unchanged",std::fabs(out.ch[control::kThrottleOutput]-.62f)<.0001f);
    in.sticks.roll=.2f;assist.update(in,out);
    check("pilot roll immediately overrides heading hold",!assist.heading_hold()&&std::fabs(assist.roll_target_rad()-.14f)<.001f);
    in.sticks.roll=0;in.sticks.yaw=.2f;assist.update(in,out);
    check("pilot rudder immediately overrides heading hold",!assist.heading_hold());
    in.sticks.yaw=0;in.yaw_rad=.4f;in.roll_rad=.4f;
    for(int i=0;i<300;++i) assist.update(in,out);
    check("heading is not captured while rolling out of a turn",!assist.heading_hold());
    in.roll_rad=0;
    for(int i=0;i<210;++i) assist.update(in,out);
    check("after pilot turn, capture new heading rather than returning to old one",assist.heading_hold()&&std::fabs(assist.heading_target_rad()-.4f)<.001f);
    in.heading_valid=false;assist.update(in,out);
    check("magnetic loss releases hold but preserves roll/pitch ASSIST",!assist.heading_hold()&&assist.roll_target_rad()==0);
    in.heading_valid=true;in.allow_heading_hold=false;
    for(int i=0;i<300;++i) assist.update(in,out);
    check("ground/failsafe permission cannot enable heading hold",!assist.heading_hold());
    assist.enter(out);
    check("mode reentry clears the previous heading target",!assist.heading_hold());
    std::printf("%d checks, %d failures\n",checks,failures); return failures?1:0;
}
