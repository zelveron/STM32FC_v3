#include "rate_ctrl.hpp"

namespace control {

void RateController::configure(const RateCtrlConfig& c)
{
    _roll.configure(c.roll,   c.sample_hz);
    _pitch.configure(c.pitch, c.sample_hz);
    _yaw.configure(c.yaw,     c.sample_hz);
}

void RateController::reset()
{
    _roll.reset(); _pitch.reset(); _yaw.reset();
}

void RateController::update(float des_p, float des_q, float des_r,
                            float gyro_p, float gyro_q, float gyro_r, float dt_s,
                            float& out_roll, float& out_pitch, float& out_yaw)
{
    out_roll  = _roll.update(des_p, gyro_p, dt_s);
    out_pitch = _pitch.update(des_q, gyro_q, dt_s);
    out_yaw   = _yaw.update(des_r, gyro_r, dt_s);
}

void RateController::preset(float des_p, float des_q, float des_r,
                            float gyro_p, float gyro_q, float gyro_r,
                            float cur_roll, float cur_pitch, float cur_yaw)
{
    _roll.preset_integrator(des_p, gyro_p, cur_roll);
    _pitch.preset_integrator(des_q, gyro_q, cur_pitch);
    _yaw.preset_integrator(des_r, gyro_r, cur_yaw);
}

} // namespace control
