#pragma once
//
// imu_prep.hpp -- IMU preprocessing before the AHRS.
//
//   1. boot-time stationary gyro-bias calibration (restarts if the board is
//      disturbed during the window);
//   2. first-order low-pass on gyro and accel.
//
// The LPF here is a placeholder anti-alias filter. At the current ~100 Hz
// sample rate a low cutoff is marginal; when the BMI323 moves to FIFO + 1 kHz
// this becomes a proper biquad / notch, tuned from the vibration test.
//
// Portable.
//
#include <cstdint>

namespace estimation {

struct ImuSample {
    float gx_dps, gy_dps, gz_dps;   // bias-corrected, filtered
    float ax_g,   ay_g,   az_g;     // filtered
    bool  bias_ready;               // false until the boot cal completes
};

class ImuPrep {
public:
    // sample_hz: nominal IMU rate. *_lpf_hz: -3 dB cutoffs (<= 0 disables).
    // cal_seconds: length of the stationary gyro-bias window.
    void configure(float sample_hz, float gyro_lpf_hz, float acc_lpf_hz,
                   float cal_seconds = 4.0f);

    ImuSample process(float gx_dps, float gy_dps, float gz_dps,
                      float ax_g, float ay_g, float az_g, float dt_s);

    bool  bias_ready() const { return _bias_ready; }
    void  gyro_bias(float& bx, float& by, float& bz) const { bx=_bx; by=_by; bz=_bz; }
    void  restart_bias_cal();

private:
    // --- gyro bias cal ---
    static constexpr float kMoveSpanDps = 4.0f;  // max-min over the window
    int   _cal_n = 400;                          // window length, set by configure()
    int   _cn = 0;
    float _sx = 0, _sy = 0, _sz = 0;
    float _min[3] = { 1e9f, 1e9f, 1e9f };
    float _max[3] = { -1e9f, -1e9f, -1e9f };
    float _bx = 0, _by = 0, _bz = 0;
    bool  _bias_ready = false;

    // --- LPF state (first order): y += a * (x - y) ---
    float _ga = 1.0f, _aa = 1.0f;   // alpha for gyro / accel
    float _gf[3] = { 0, 0, 0 };
    float _af[3] = { 0, 0, 0 };
    bool  _lpf_primed = false;
};

} // namespace estimation
