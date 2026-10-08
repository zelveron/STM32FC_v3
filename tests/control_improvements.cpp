#include "../src/control/attitude_ctrl.hpp"
#include "../src/control/pid.hpp"
#include "../src/control/command_shape.hpp"
#include "../src/modes/mode_assist.hpp"
#include "../src/core/flight_state.hpp"
#include "../src/core/telemetry_schedule.hpp"
#include "../src/estimation/biquad.hpp"
#include "../src/estimation/ahrs.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
int checks=0,failures=0;
void check(const char* n,bool ok) { ++checks; failures+=!ok; std::printf("[%s] %s\n",ok?"PASS":"FAIL",n); }
int main() {
    using control::bound;
    control::AttitudeController ac; control::AttitudeCtrlConfig cfg; ac.configure(cfg);
    float p,q,r; const float bank=.5235987756f;
    const float heading=ac.coordinated_heading_rate(bank,0,20);
    ac.body_rates(bank,0,bank,0,heading,p,q,r);
    check("30 degree coordinated turn retains required body pitch and yaw rates",std::fabs(p)<.001&&std::fabs(q-8.110f)<.02&&std::fabs(r-14.047f)<.02);
    check("steady bank has zero Euler pitch rate despite nonzero body pitch rate",std::fabs(q*std::cos(bank)-r*std::sin(bank))<.001);
    ac.body_rates(-bank,0,-bank,0,-heading,p,q,r);
    check("left and right turns both require positive body pitch rate",q>8&&r<-14);
    ac.body_rates(-3.13f,0,3.13f,0,0,p,q,r);
    check("roll recovery wraps angle error across pi",p>0&&p<5);
    control::RateDemand demand; demand.reset(0);
    check("stick step respects angular acceleration limit",std::fabs(demand.update(100,600,.0025f)-1.5f)<.001);
    check("invalid time step cannot jump request",demand.update(100,600,.1f)==1.5f);
    control::Pid pid; control::PidGains gains; gains.kp=.1f; gains.ki=1; gains.i_max=.4f;
    pid.configure(gains,400);
    for(int i=0;i<1000;++i) { const float v=pid.update(3,0,.0025f); pid.track_applied(v,0); }
    check("downstream surface limit prevents hidden integral accumulation",std::fabs(pid.i_term())<.0001);
    for(int i=0;i<100;++i) { const float v=pid.update(1,0,.0025f); pid.track_applied(v,v); }
    check("integrator remains effective when output is attainable",pid.i_term()>.2f);
    const float before=pid.i_term(); const float command=pid.update(-1,0,.0025f); pid.track_applied(command,0);
    check("external tracking permits integration away from a limit",pid.i_term()<before);
    control::AssistTuning tuning;
    check("groundspeed/unvalidated airspeed cannot schedule gains",control::airspeed_scale(8,false,tuning)==1);
    check("airspeed scaling is bounded at low and high speed",control::airspeed_scale(8,true,tuning)==2&&control::airspeed_scale(50,true,tuning)==.5f);
    check("invalid airspeed returns nominal gains",control::airspeed_scale(NAN,true,tuning)==1);
    modes::ModeAssist mode; control::RateCtrlConfig rc;
    rc.roll.kp=.01f; rc.pitch.kp=.02f; rc.yaw.kp=.006f;
    mode.configure(cfg,rc,.7f,.45f); mode.enter({});
    modes::ModeInput in; in.roll_rad=bank; in.gyro_q_dps=8.11f; in.gyro_r_dps=14.047f; in.sticks.roll=bank/.7f;
    control::Outputs out;
    for(int i=0;i<1000;++i) mode.update(in,out);
    check("no-pitot steady turn does not command opposing rudder",std::fabs(out.ch[control::kYawOutput])<.002f);
    check("no-pitot geometry retains pitch rate without attitude error",std::fabs(mode.demand_q()-8.11f)<.03f);
    in.sticks.yaw=1;
    for(int i=0;i<1000;++i) mode.update(in,out);
    check("full pilot rudder overrides yaw damping",std::fabs(out.ch[control::kYawOutput]-1)<.001);
    tuning.pitch_trim_rad=.08f; mode.set_tuning(tuning); mode.enter({});
    in={}; in.pitch_rad=.08f;
    for(int i=0;i<300;++i) mode.update(in,out);
    check("pitch trim is an attitude target offset",std::fabs(mode.demand_q())<.001);
    core::FlightState flight;
    check("disarmed aircraft freezes integration",!flight.update(false,false,1,true,20,true,0,0,0));
    flight.update(true,false,.4f,false,0,false,0,0,1);
    check("power evidence is debounced",!flight.update(true,false,.4f,false,0,false,0,0,499));
    check("moderate power enables integration without GNSS",flight.update(true,false,.4f,false,0,false,0,0,501));
    check("glide retains integrators",flight.update(true,false,0,false,0,false,0,0,1000));
    check("RC failsafe retains flight evidence",flight.update(false,true,0,false,0,false,0,0,1500));
    check("explicit disarm clears flight evidence",!flight.update(false,false,0,false,0,false,0,0,2000));
    flight.update(true,false,.4f,true,10,true,0,0,3000); flight.update(true,false,.4f,true,10,true,0,0,3500);
    flight.update(true,false,0,true,0,true,0,0,4000);
    check("landing requires sustained corroborating measurements",flight.update(true,false,0,true,0,true,0,0,6999)&&!flight.update(true,false,0,true,0,true,0,0,7000));
    core::TelemetrySchedule telem; core::TelemConfig tc; tc.bytes_per_second=80; telem.configure(tc);
    auto kind=telem.choose(0,true,true,true,1,"MANUAL",true);
    check("mode change has telemetry priority",kind==core::TelemKind::mode);
    telem.result(kind,false,0,1,"MANUAL");
    check("queue rejection is counted and latest state retried",telem.queue_failures==1&&telem.choose(10,true,true,true,1,"ASSIST",true)==core::TelemKind::mode);
    unsigned bytes=0;
    for(unsigned ms=10;ms<10010;ms+=10) {
        auto k=telem.choose(ms,true,true,true,1,"ASSIST",true);
        if(k!=core::TelemKind::none) { bytes+=k==core::TelemKind::attitude?10:k==core::TelemKind::vario?6:k==core::TelemKind::gps?19:11; telem.result(k,true,ms,1,"ASSIST"); }
    }
    check("telemetry respects byte budget plus bounded initial burst",bytes<=832);
    check("unchanged GNSS fix is transmitted only once",telem.sent[2]==1);
    check("budget pressure is observable",telem.deferred>0);
    check("RC link loss suppresses FC telemetry",telem.choose(20000,true,true,true,2,"!FS-LVL",false)==core::TelemKind::none);
    core::TelemetrySchedule none; tc.enabled[0]=tc.enabled[1]=tc.enabled[2]=tc.enabled[3]=false; none.configure(tc);
    check("each telemetry group can be disabled",none.choose(0,true,true,true,1,"MANUAL",true)==core::TelemKind::none);
    estimation::Notch notch;
    check("unmeasured notch defaults to bypass",notch.apply(.25f)==.25f);
    check("invalid notch settings are rejected",!notch.configure(400,200,3));
    check("measured notch accepts valid frequency/Q",notch.configure(400,80,3));
    float energy=0;
    for(int i=0;i<2000;++i) { float y=notch.apply(std::sin(6.2831853f*80*i/400)); if(i>500)energy+=y*y; }
    check("notch suppresses configured sinusoidal vibration",energy/1500<.001f);
    ahrs::reset(); ahrs::update(0,0,1,0,0,0,.0025f);
    for(int i=0;i<2000;++i) ahrs::update(.6f,0,.8f,0,0,0,.0025f);
    check("large constant-norm acceleration innovation is rejected",std::fabs(ahrs::pitch_rad())<.01&&ahrs::acc_trust()==0);
    float bx,by,bz; ahrs::gyro_bias_dps(bx,by,bz);
    check("rejected acceleration cannot learn false gyro bias",std::fabs(by)<.001);
    std::printf("%d checks, %d failures\n",checks,failures); return failures?1:0;
}
