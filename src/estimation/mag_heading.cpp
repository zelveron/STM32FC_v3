#include "mag_heading.hpp"
#include "imu_rotation.hpp"
#include <cmath>

namespace estimation {
namespace {
constexpr float rad=0.017453292519943295f;
float wrap(float v) { return std::atan2(std::sin(v),std::cos(v)); }
float bound(float v,float lo,float hi) { return v<lo?lo:v>hi?hi:v; }
}
void MagHeading::reset() {
    _state=State::waiting; _candidate=_accepted=_aligned=_tracking=false;
    _first_ms=_last_ms=0; _count=0;
    _previous_innovation=_measured=_field=_innovation=0;
}
float MagHeading::reject(State state) {
    _state=state; _candidate=_accepted=_tracking=false; _count=0; return 0;
}
void MagHeading::invalidate_attitude() { reset(); _state=State::no_attitude; }
void MagHeading::driver_failed() { reject(State::driver); }
bool MagHeading::configured() const {
    const auto& c=_config;
    if(!c.orientation_confirmed || !c.calibrated || !rotation_valid(c.rotation)) return false;
    if(!(c.field_ut>=15&&c.field_ut<=100&&c.field_tolerance>.05f&&c.field_tolerance<=.5f&&
         c.min_horizontal_ut>=5&&c.min_horizontal_ut<c.field_ut&&
         c.tau_s>=.5f&&c.tau_s<=30&&c.max_correction_dps>0&&c.max_correction_dps<=10&&
         c.max_innovation_deg>=10&&c.max_innovation_deg<=60)) return false;
    for(unsigned i=0;i<3;++i) {
        if(!std::isfinite(c.offset_ut[i])||std::fabs(c.offset_ut[i])>2000) return false;
        for(unsigned j=0;j<3;++j)
            if(!std::isfinite(c.correction[i][j])||std::fabs(c.correction[i][j])>10||
               std::fabs(c.correction[i][j]-c.correction[j][i])>1e-4f) return false;
    }
    const auto& m=c.correction;
    // Sylvester's criterion: reject reflected/singular calibration matrices.
    const float det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-
                    m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+
                    m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    return m[0][0]>.01f && m[0][0]*m[1][1]-m[0][1]*m[1][0]>.0001f && det>.001f;
}
float MagHeading::observe(float x,float y,float z,float roll,float pitch,float yaw,
                         bool attitude_valid,bool allow_initial_alignment,uint32_t now) {
    if(!_config.enabled) return reject(State::disabled);
    if(!configured()) return reject(State::setup);
    if(!attitude_valid || !std::isfinite(roll)||!std::isfinite(pitch)||!std::isfinite(yaw)||
       std::fabs(std::cos(pitch))<.25f) return reject(State::no_attitude);
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)) return reject(State::field);
    const float raw[3]={x-_config.offset_ut[0],y-_config.offset_ut[1],z-_config.offset_ut[2]};
    float corrected[3]{},body[3]{};
    for(unsigned i=0;i<3;++i) for(unsigned j=0;j<3;++j) corrected[i]+=_config.correction[i][j]*raw[j];
    rotate(_config.rotation,corrected,body);
    _field=std::sqrt(body[0]*body[0]+body[1]*body[1]+body[2]*body[2]);
    // R_y(pitch) R_x(roll) removes tilt in the FRD/NED convention. Heading
    // is atan2(-east, north); magnetic north, not GNSS course or true north.
    const float cr=std::cos(roll),sr=std::sin(roll),cp=std::cos(pitch),sp=std::sin(pitch);
    const float north=cp*body[0]+sp*(sr*body[1]+cr*body[2]);
    const float east=cr*body[1]-sr*body[2];
    const float horizontal=std::sqrt(north*north+east*east);
    if(!std::isfinite(_field)) { _field=0; return reject(State::field); }
    if(std::fabs(_field-_config.field_ut)>_config.field_ut*_config.field_tolerance||
       horizontal<_config.min_horizontal_ut) return reject(State::field);
    _measured=std::atan2(-east,north);
    _innovation=wrap(_measured-yaw);
    if(_aligned && std::fabs(_innovation)>_config.max_innovation_deg*rad) return reject(State::innovation);
    const uint32_t elapsed=now-_last_ms;
    // No duplicate sample may qualify the compass or apply extra correction.
    if(_candidate && elapsed==0) return 0;
    if(!_candidate || elapsed>120 || std::fabs(wrap(_innovation-_previous_innovation))>15*rad) {
        _candidate=true; _accepted=_tracking=false; _first_ms=now; _count=0;
    }
    _previous_innovation=_innovation; _last_ms=now; if(_count<1000) ++_count;
    if(uint32_t(now-_first_ms)<500 || _count<10) { _state=State::qualifying; return 0; }
    _accepted=true;
    if(!_aligned && allow_initial_alignment) {
        _aligned=_tracking=true; _state=State::tracking;
        _previous_innovation=0;
        return _innovation; // Before the first arming only; never jump airborne.
    }
    const float dt=bound(elapsed*.001f,0,.1f);
    const float correction=bound(_innovation*dt/(_config.tau_s+dt),
                                 -_config.max_correction_dps*rad*dt,_config.max_correction_dps*rad*dt);
    _tracking=std::fabs(_innovation)<5*rad;
    if(_tracking) _aligned=true;
    _state=_tracking?State::tracking:State::aligning;
    _previous_innovation=wrap(_innovation-correction);
    return correction;
}
bool MagHeading::aiding(uint32_t now,bool attitude_valid,bool driver_healthy) const {
    return _config.enabled&&configured()&&attitude_valid&&driver_healthy&&_accepted&&uint32_t(now-_last_ms)<200;
}
bool MagHeading::heading_valid(uint32_t now,bool attitude_valid,bool driver_healthy) const {
    return aiding(now,attitude_valid,driver_healthy)&&_tracking;
}
MagHeading::State MagHeading::state(uint32_t now,bool attitude_valid,bool driver_healthy) const {
    if(!_config.enabled) return State::disabled;
    if(_state==State::driver || !driver_healthy) return State::driver;
    if(!configured()) return State::setup;
    if(!attitude_valid) return State::no_attitude;
    if(_state==State::field||_state==State::innovation||_state==State::driver) return _state;
    if(_candidate&&uint32_t(now-_last_ms)>=200) return State::stale;
    return _state;
}
const char* MagHeading::state_name(State s) {
    switch(s) {
    case State::disabled:return "disabled"; case State::setup:return "setup_required";
    case State::waiting:return "waiting"; case State::no_attitude:return "no_attitude";
    case State::field:return "field_rejected"; case State::innovation:return "innovation_rejected";
    case State::qualifying:return "qualifying"; case State::aligning:return "aligning";
    case State::tracking:return "tracking"; case State::stale:return "stale";
    case State::driver:return "driver_unavailable";
    }
    return "waiting";
}
}
