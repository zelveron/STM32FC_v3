#include "imu_prep.hpp"
#include <cmath>

namespace estimation {
namespace {
constexpr float kTwoPi = 6.28318530717958647692f;

// first-order LPF coefficient for cutoff fc at timestep dt
float lpf_alpha(float fc, float dt)
{
    if (fc <= 0.0f || dt <= 0.0f) return 1.0f;      // pass-through
    const float rc = 1.0f / (kTwoPi * fc);
    return dt / (rc + dt);
}
} // namespace

void ImuPrep::configure(float sample_hz, float gyro_lpf_hz, float acc_lpf_hz,
                        float cal_seconds)
{
    const float dt = (sample_hz > 0.0f) ? 1.0f / sample_hz : 0.0f;
    _ga = lpf_alpha(gyro_lpf_hz, dt);
    _aa = lpf_alpha(acc_lpf_hz, dt);

    int n = (int)(cal_seconds * sample_hz);
    if (n < 100)  n = 100;
    if (n > 8000) n = 8000;
    _cal_n = n;
}

void ImuPrep::restart_bias_cal()
{
    _cn = 0; _sx = _sy = _sz = 0.0f;
    _min[0] = _min[1] = _min[2] =  1e9f;
    _max[0] = _max[1] = _max[2] = -1e9f;
    _bias_ready = false;
}

ImuSample ImuPrep::process(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt_s)
{
    // --- gyro bias calibration (only while not yet ready) ---
    if (!_bias_ready) {
        const float g[3] = { gx, gy, gz };
        bool moved = false;
        for (int i = 0; i < 3; i++) {
            if (g[i] < _min[i]) _min[i] = g[i];
            if (g[i] > _max[i]) _max[i] = g[i];
            if (_max[i] - _min[i] > kMoveSpanDps) moved = true;
        }
        if (moved) {
            restart_bias_cal();
            // re-seed the window with this sample
            for (int i = 0; i < 3; i++) { _min[i] = _max[i] = g[i]; }
        }
        _sx += gx; _sy += gy; _sz += gz;
        if (++_cn >= _cal_n) {
            _bx = _sx / _cn; _by = _sy / _cn; _bz = _sz / _cn;
            _bias_ready = true;
        }
    }

    // --- apply bias (zero until ready) + LPF ---
    float gc[3] = { gx - _bx, gy - _by, gz - _bz };
    float ac[3] = { ax, ay, az };

    if (!_lpf_primed) {
        for (int i = 0; i < 3; i++) { _gf[i] = gc[i]; _af[i] = ac[i]; }
        _lpf_primed = true;
    } else {
        for (int i = 0; i < 3; i++) {
            _gf[i] += _ga * (gc[i] - _gf[i]);   // alpha fixed at configure()
            _af[i] += _aa * (ac[i] - _af[i]);
        }
    }
    (void)dt_s;   // alpha is precomputed from the nominal rate

    return { _gf[0], _gf[1], _gf[2], _af[0], _af[1], _af[2], _bias_ready };
}

} // namespace estimation
