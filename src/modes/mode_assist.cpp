#include "mode_assist.hpp"

namespace modes {

void ModeAssist::configure(const control::AttitudeCtrlConfig& att,
                           const control::RateCtrlConfig& rate,
                           float max_roll_rad, float max_pitch_rad)
{
    _att.configure(att);
    _rate.configure(rate);
    _max_roll     = max_roll_rad;
    _max_pitch    = max_pitch_rad;
}

void ModeAssist::enter(const control::Outputs& current)
{
    _rate.reset(); _seed=true;
    _holding=false; _capture_s=0; _heading_target=0; _roll_target=0;
    _transition.enter(current);
}

void ModeAssist::update(const ModeInput& in, control::Outputs& out)
{
    using control::bound;
    const float cp=std::cos(in.pitch_rad);
    // Avoid Euler-rate singularities outside the normal fixed-wing envelope.
    const float heading=std::fabs(cp)>.25f ? bound((in.gyro_q_dps*std::sin(in.roll_rad)+
                                  in.gyro_r_dps*std::cos(in.roll_rad))/cp,-90,90) : 0;
    if(_seed) {
        _roll_demand.reset(in.gyro_p_dps); _pitch_demand.reset(in.gyro_q_dps);
        _heading_slow=heading; _seed=false;
    }
    _heading_slow+=bound(in.dt_s/(.5f+in.dt_s),0,1)*(heading-_heading_slow);
    const bool eligible=_tuning.heading_hold_enabled&&in.heading_valid&&in.allow_heading_hold&&
        std::isfinite(in.yaw_rad)&&std::isfinite(in.roll_rad)&&std::isfinite(in.pitch_rad)&&
        std::isfinite(in.dt_s)&&in.dt_s>0&&in.dt_s<=.02f&&std::fabs(cp)>.5f&&
        std::fabs(in.sticks.roll)<=_tuning.heading_stick_deadband&&
        std::fabs(in.sticks.yaw)<=_tuning.heading_stick_deadband;
    if(!eligible) { _holding=false; _capture_s=0; }
    else if(!_holding) {
        // Capture after leveling, not while the pilot's last bank still turns
        // the aircraft. This avoids fighting roll-out or holding an old target.
        if(std::fabs(in.roll_rad)<.174533f&&std::fabs(heading)<8) _capture_s+=in.dt_s;
        else _capture_s=0;
        if(_capture_s>=_tuning.heading_capture_s) { _heading_target=in.yaw_rad; _holding=true; }
    }
    _roll_target=in.sticks.roll*_max_roll;
    if(_holding) {
        const float error=std::atan2(std::sin(_heading_target-in.yaw_rad),std::cos(_heading_target-in.yaw_rad));
        const float limit=std::fmin(_max_roll,_tuning.heading_bank_limit_rad);
        _roll_target=bound(_tuning.heading_bank_per_rad*error,-limit,limit);
    }
    const bool air=in.airspeed_valid&&in.airspeed_mps>=_tuning.min_airspeed_mps&&
                   in.airspeed_mps<=_tuning.max_airspeed_mps&&std::isfinite(in.airspeed_mps);
    const float turn=air?_att.coordinated_heading_rate(in.roll_rad,in.pitch_rad,in.airspeed_mps):heading;
    float p,q,r;
    const float tp=bound(in.sticks.pitch*_max_pitch+_tuning.pitch_trim_rad,-_max_pitch,_max_pitch);
    _att.body_rates(_roll_target,tp,in.roll_rad,in.pitch_rad,turn,p,q,r);
    _dp=_roll_demand.update(p,_tuning.roll_accel_dps2,in.dt_s);
    _dq=_pitch_demand.update(q,_tuning.pitch_accel_dps2,in.dt_s);
    // Without airspeed a washout damps disturbances, then releases a steady
    // natural turn. With airspeed, use the coordinated turn reference.
    if(!air) {
        float unused_p,unused_q;
        _att.body_rates(_roll_target,tp,in.roll_rad,in.pitch_rad,
                        _heading_slow,unused_p,unused_q,r);
    }
    if(!in.allow_integrators) _rate.clear_integrators();
    _rate.set_integrator_enabled(in.allow_integrators);
    float rc,pc,yc;
    _rate.update(_dp,_dq,0,in.gyro_p_dps,in.gyro_q_dps,in.gyro_r_dps-r,in.dt_s,rc,pc,yc);
    const float scale=control::airspeed_scale(in.airspeed_mps,air,_tuning);
    const float yaw=bound(in.sticks.yaw,-1,1);
    // Pilot rudder is direct; yaw damping fades out at full rudder authority.
    const float yc_applied=bound(yc*scale,-_tuning.yaw_damper_limit,_tuning.yaw_damper_limit)*(1-std::fabs(yaw));
    control::mix_manual({rc*scale,pc*scale,yaw+yc_applied,in.sticks.throttle},mix,out);
    _transition.apply(out,in.dt_s);
    const float ar=std::fabs(mix.aileron_gain)>1e-6f?out.ch[control::kRollOutput]/mix.aileron_gain/scale:0;
    const float ap=std::fabs(mix.elevator_gain)>1e-6f?out.ch[control::kPitchOutput]/mix.elevator_gain/scale:0;
    const float ay=std::fabs(mix.rudder_gain)>1e-6f?(out.ch[control::kYawOutput]/mix.rudder_gain-yaw)/scale:0;
    _rate.track_applied(rc,pc,yc,ar,ap,ay);
}
} // namespace modes
