// Expanded behavioral SITL, not an identified model of the user's airframe.
#include "../src/sitl/aircraft.hpp"
#include "../src/sitl/sensors.hpp"
#include "../src/modes/mode_assist.hpp"
#include "../src/modes/mode_takeoff.hpp"
#include "../src/estimation/ahrs.hpp"
#include "../src/estimation/imu_prep.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
int checks=0,failures=0;
void check(const char* n,bool ok) { ++checks; failures+=!ok; std::printf("[%s] %s\n",ok?"PASS":"FAIL",n); }
int main() {
    for(int scenario=0;scenario<4;++scenario) {
        const bool pitot=scenario==3;
        const double initial=scenario==1?15:scenario==2?24:18;
        sitl::Aircraft aircraft; aircraft.reset(initial,100);
        // Constant wind changes groundspeed, not aerodynamic velocity.
        if(scenario==2) aircraft.set_wind({6,3,0},true);
        double roll,pitch,yaw; aircraft.euler(roll,pitch,yaw);
        control::AssistTuning tuning; tuning.pitch_trim_rad=float(pitch);
        modes::ModeAssist mode; control::AttitudeCtrlConfig ac; control::RateCtrlConfig rc;
        rc.roll.kff=.006f; rc.roll.kp=.010f; rc.roll.ki=.02f; rc.roll.i_max=.4f;
        rc.pitch=rc.roll; rc.pitch.kff=.010f; rc.pitch.kp=.020f;
        rc.yaw.kp=.006f;
        mode.configure(ac,rc,.7f,.45f); mode.set_tuning(tuning); mode.enter({});
        estimation::ImuPrep prep; prep.configure(400,30,15,4);
        for(int i=0;i<1601;++i) prep.process(0,0,0,0,0,1,.0025f);
        ahrs::reset();
        // Correct initial tilt while stationary, then use moving IMU samples.
        ahrs::update(float(-std::sin(pitch)),0,float(std::cos(pitch)),0,0,0,.0025f);
        double held[3]{},delayed[3]{},actual[3]{};
        double max_bank_error=0,max_pitch_error=0,max_estimator_error=0,min_speed=100,max_surface=0;
        double final_bank=0; bool finite=true;
        for(int step=0;step<400*70;++step) {
            const float t=step*.0025f;
            const auto sn=sitl::synth(aircraft.state());
            // Small repeatable gyro noise; both sensor and software filters exist.
            const float vibration=.3f*std::sin(6.2831853f*80*t);
            const auto s=prep.process(sn.gx_dps+vibration+.02f,sn.gy_dps-.03f,sn.gz_dps+.015f,sn.ax_g,sn.ay_g,sn.az_g,.0025f,false);
            ahrs::update(s.ax_g,s.ay_g,s.az_g,s.gx_dps,s.gy_dps,s.gz_dps,.0025f);
            modes::ModeInput in; in.roll_rad=ahrs::roll_rad(); in.pitch_rad=ahrs::pitch_rad();
            in.gyro_p_dps=s.gx_dps; in.gyro_q_dps=s.gy_dps; in.gyro_r_dps=s.gz_dps;
            // Long turns in both directions, followed by wings-level recovery.
            const float target=t<5?0:t<30?.35f:t<55?-.35f:0;
            in.sticks.roll=target/.7f;
            // Simulated pilot adds power to hold cruise speed in long turns.
            // This is explicitly outside ASSIST; the FC has no autothrottle.
            in.sticks.throttle=control::bound(float(aircraft.trim_throttle()+.10*(initial-aircraft.airspeed_mps())),0,1);
            in.airspeed_valid=pitot; in.airspeed_mps=pitot?float(aircraft.airspeed_mps()):0;
            control::Outputs out; mode.update(in,out);
            // 50 Hz zero-order hold + one frame delay + 50 ms actuator lag,
            // capped at six normalized units/s. This is a test assumption.
            if(step%8==0) for(int i=0;i<3;++i) { delayed[i]=held[i]; held[i]=out.ch[control::kAxisOutputs[i]]; }
            for(int i=0;i<3;++i) {
                const double delta=(delayed[i]-actual[i])*.0025/(.05+.0025);
                actual[i]+=std::max(-.015,std::min(.015,delta));
                max_surface=std::max(max_surface,std::fabs(actual[i]));
            }
            aircraft.step({actual[0],actual[1]+aircraft.trim_elevator(),actual[2],in.sticks.throttle},.0025);
            aircraft.euler(roll,pitch,yaw);
            finite=finite&&std::isfinite(roll)&&std::isfinite(pitch);
            min_speed=std::min(min_speed,aircraft.airspeed_mps());
            max_estimator_error=std::max(max_estimator_error,std::fabs(roll-ahrs::roll_rad()));
            if((t>12&&t<29)||(t>38&&t<54)) {
                max_bank_error=std::max(max_bank_error,std::fabs(roll-target));
                max_pitch_error=std::max(max_pitch_error,std::fabs(pitch-tuning.pitch_trim_rad));
            }
            if(t>65) final_bank=std::max(final_bank,std::fabs(roll));
        }
        std::printf("scenario=%d pitot=%d initial=%.1f bank_error=%.3f pitch_error=%.3f estimator_error=%.3f final_bank=%.3f min_speed=%.1f\n",
                    scenario,pitot,initial,max_bank_error,max_pitch_error,max_estimator_error,final_bank,min_speed);
        check("delayed 50 Hz surfaces remain bounded and states finite",finite&&max_surface<=1);
        check("sustained turn tracks bank within 10 degrees",max_bank_error<.175);
        check("sustained turn pitch error remains within 10 degrees",max_pitch_error<.175);
        check("long-turn estimator roll error remains within 10 degrees",max_estimator_error<.175);
        check("wings-level recovery completes after long turns",final_bank<.12);
        check("simulated aircraft retains forward airspeed",min_speed>10);
    }
    // Close the new heading -> bank -> turn loop through the aircraft model.
    // Truth attitude isolates control behavior; magnetic fusion/fault paths
    // are exercised separately in magnetic_heading.cpp and regression.cpp.
    {
        sitl::Aircraft aircraft;aircraft.reset(18,100);
        double roll,pitch,yaw;aircraft.euler(roll,pitch,yaw);
        control::AssistTuning tuning;tuning.pitch_trim_rad=float(pitch);
        modes::ModeAssist mode;control::AttitudeCtrlConfig ac;control::RateCtrlConfig rc;
        rc.roll.kff=.006f;rc.roll.kp=.010f;rc.roll.ki=.02f;rc.roll.i_max=.4f;
        rc.pitch=rc.roll;rc.pitch.kff=.010f;rc.pitch.kp=.020f;rc.yaw.kp=.006f;
        mode.configure(ac,rc,.7f,.45f);mode.set_tuning(tuning);mode.enter({});
        double actual[3]{},held[3]{},delayed[3]{};
        double peak_error=0,final_error=0,max_bank_demand=0;
        bool held_target=true,compass_release=true,pilot_release=true,finite=true;
        for(int step=0;step<400*36;++step) {
            const float t=step*.0025f;
            aircraft.euler(roll,pitch,yaw);
            const auto& state=aircraft.state();
            modes::ModeInput in;in.roll_rad=float(roll);in.pitch_rad=float(pitch);in.yaw_rad=float(yaw);
            in.gyro_p_dps=float(state.omega_body.x*57.2957795);
            in.gyro_q_dps=float(state.omega_body.y*57.2957795);
            in.gyro_r_dps=float(state.omega_body.z*57.2957795);
            in.heading_valid=t<25||t>=29;in.allow_heading_hold=true;
            in.sticks.throttle=control::bound(float(aircraft.trim_throttle()+.10*(18-aircraft.airspeed_mps())),0,1);
            if(t>=32)in.sticks.roll=.4f;
            control::Outputs out;mode.update(in,out);
            if(t>1&&t<25) {
                held_target&=mode.heading_hold()&&std::fabs(mode.heading_target_rad())<.01f;
                max_bank_demand=std::max(max_bank_demand,double(std::fabs(mode.roll_target_rad())));
            }
            if(t>=25&&t<29)compass_release&=!mode.heading_hold();
            if(t>=32)pilot_release&=!mode.heading_hold()&&mode.roll_target_rad()>.27f;
            if(step%8==0)for(int i=0;i<3;++i){delayed[i]=held[i];held[i]=out.ch[control::kAxisOutputs[i]];}
            for(int i=0;i<3;++i)actual[i]+=std::max(-.015,std::min(.015,(delayed[i]-actual[i])*.0025/(.05+.0025)));
            // External rolling disturbance after heading capture.
            const double disturbance=t>=4&&t<6?.25:0;
            aircraft.step({actual[0]+disturbance,actual[1]+aircraft.trim_elevator(),actual[2],in.sticks.throttle},.0025);
            finite&=std::isfinite(yaw)&&std::isfinite(out.ch[0]);
            if(t>=4&&t<25)peak_error=std::max(peak_error,std::fabs(yaw));
            if(t>=22&&t<25)final_error=std::max(final_error,std::fabs(yaw));
        }
        std::printf("heading scenario: peak_error=%.3f final_error=%.3f max_bank_demand=%.3f\n",peak_error,final_error,max_bank_demand);
        check("heading hold retains captured target through rolling disturbance",held_target&&peak_error>.02);
        check("heading bank loop recovers within two degrees",finite&&final_error<.03491);
        check("heading bank demand stays within fifteen degrees",max_bank_demand<=.2618);
        check("compass loss and pilot roll release heading hold in flight",compass_release&&pilot_release);
    }
    std::printf("%d checks, %d failures\n",checks,failures); return failures?1:0;
}
