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
    _gyro_fc = gyro_lpf_hz;
    _acc_fc = acc_lpf_hz;

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
    _bx = _by = _bz = 0;
    _lpf_primed = false;
}

ImuSample ImuPrep::process(float gx, float gy, float gz,
                           float ax, float ay, float az, float dt_s, bool allow_calibration)
{
    const float amag2 = ax*ax + ay*ay + az*az;
    const bool stationary = allow_calibration && std::isfinite(amag2) &&
        amag2 >= 0.81f && amag2 <= 1.21f &&
        std::isfinite(gx) && std::isfinite(gy) && std::isfinite(gz) &&
        std::fabs(gx) < 3 && std::fabs(gy) < 3 && std::fabs(gz) < 3 &&
        std::isfinite(dt_s) && dt_s > 0 && dt_s <= 0.02f;
    if (!_bias_ready && !stationary) restart_bias_cal();
    // --- gyro bias calibration (only while not yet ready) ---
    if (!_bias_ready && stationary) {
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
    for(int i=0;i<3;++i) gc[i]=_notch[i].apply(gc[i]);

    _ga = lpf_alpha(_gyro_fc, dt_s);
    _aa = lpf_alpha(_acc_fc, dt_s);
    if (!_lpf_primed) {
        for (int i = 0; i < 3; i++) { _gf[i] = gc[i]; _af[i] = ac[i]; }
        _lpf_primed = true;
    } else {
        for (int i = 0; i < 3; i++) {
            _gf[i] += _ga * (gc[i] - _gf[i]);   // coefficient follows dt
            _af[i] += _aa * (ac[i] - _af[i]);
        }
    }

    return { _gf[0], _gf[1], _gf[2], _af[0], _af[1], _af[2], _bias_ready };
}

} // namespace estimation
