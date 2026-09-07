#pragma once
//
// surface_test.hpp -- a one-shot sequential control-surface sweep.
//
// aileron, then elevator, then rudder: each centre -> +full -> -full -> centre
// over ~0.6 s. Used as the "gyro-bias calibration complete" signal so a pilot
// at the aircraft knows it is ready without a screen. Never touches throttle.
//
// Portable.
//
namespace control {

class SurfaceTest {
public:
    void start()        { _t_s = 0.0f; }
    void cancel()       { _t_s = -1.0f; }
    bool active() const { return _t_s >= 0.0f; }

    // Advance by dt_s. Writes roll/pitch/yaw deflection in [-1,+1] (only the
    // segment's active axis is non-zero). Returns false once finished.
    bool step(float dt_s, float& roll, float& pitch, float& yaw);

private:
    float _t_s = -1.0f;                       // <0 = inactive
    static constexpr float kSeg   = 0.6f;     // seconds per surface
    static constexpr float kSegs  = 3.0f;
};

inline bool SurfaceTest::step(float dt_s, float& roll, float& pitch, float& yaw)
{
    roll = pitch = yaw = 0.0f;
    if (_t_s < 0.0f) return false;

    _t_s += dt_s;
    if (_t_s >= kSeg * kSegs) { _t_s = -1.0f; return false; }

    const int   seg = static_cast<int>(_t_s / kSeg);        // 0 ail, 1 ele, 2 rud
    const float u   = (_t_s - seg * kSeg) / kSeg;           // 0..1 in the segment

    float d;                                                 // centre -> +1 -> -1 -> centre
    if      (u < 0.25f) d =  u / 0.25f;
    else if (u < 0.75f) d =  1.0f - (u - 0.25f) / 0.25f;
    else                d = -1.0f + (u - 0.75f) / 0.25f;

    if      (seg == 0) roll  = d;
    else if (seg == 1) pitch = d;
    else               yaw   = d;
    return true;
}

} // namespace control
