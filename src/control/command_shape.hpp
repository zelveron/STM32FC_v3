#pragma once
#include <cmath>
namespace control {
inline float bound(float x,float lo,float hi) { return x<lo?lo:x>hi?hi:x; }
// Shape requested angular velocity instead of delaying the gyro feedback.
class RateDemand {
public:
    void reset(float v=0) { _v=v; }
    float update(float target,float accel_dps2,float dt) {
        if(!std::isfinite(target)||!std::isfinite(dt)||dt<=0||dt>.02f) return _v;
        _v+=bound(target-_v,-accel_dps2*dt,accel_dps2*dt); return _v;
    }
private: float _v=0;
};
struct AssistTuning {
    float pitch_trim_rad=0;
    float roll_accel_dps2=600, pitch_accel_dps2=400;
    float surface_rate_per_s=12; // normalized travel/s; verify real servo response
    float transition_s=.25f;
    float yaw_damper_limit=.25f;
    bool heading_hold_enabled=true;
    float heading_bank_per_rad=.5f; // heading error -> shallow corrective bank
    float heading_bank_limit_rad=.2617994f; // 15 degrees, also limited by max roll
    float heading_stick_deadband=.05f, heading_capture_s=.5f;
    float reference_airspeed_mps=18;
    float min_airspeed_mps=8, max_airspeed_mps=50;
};
// Only explicitly valid measured airspeed; GPS speed is never substituted.
inline float airspeed_scale(float v,bool valid,const AssistTuning& c) {
    if(!valid||!std::isfinite(v)||v<c.min_airspeed_mps||v>c.max_airspeed_mps) return 1;
    const float ratio=c.reference_airspeed_mps/v;
    return bound(ratio*ratio,.5f,2.f);
}
}
