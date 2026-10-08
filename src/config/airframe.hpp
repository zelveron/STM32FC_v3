#pragma once
#include <cstdint>
#include "../control/command_shape.hpp"
#include "../control/mixer.hpp"
#include "../core/telemetry_schedule.hpp"
namespace config {
#ifndef FC_MAG_ENABLED
#define FC_MAG_ENABLED 1
#endif
constexpr const char* imu_model = "BMI270";
constexpr bool enable_magnetometer = FC_MAG_ENABLED != 0;
constexpr control::AssistTuning assist_tuning{};
constexpr core::TelemConfig telemetry{}; // per-type enable/interval and CRSF byte budget
constexpr float gyro_lpf_hz=30, accel_lpf_hz=15;
constexpr float gyro_notch_hz=0, gyro_notch_q=3; // 0 = disabled; measure first
// Six-face calibration in body axes AFTER the specific-force sign conversion.
// corrected = (measurement - offset_g) * scale; identity until measured.
constexpr float accel_offset_g[2][3]={{0,0,0},{0,0,0}};
constexpr float accel_scale[2][3]={{1,1,1},{1,1,1}};
constexpr bool sensor_config_valid() {
    if(!(gyro_lpf_hz>0&&gyro_lpf_hz<180&&accel_lpf_hz>0&&accel_lpf_hz<180)) return false;
    if(!(gyro_notch_hz>=0&&gyro_notch_hz<180&&gyro_notch_q>=.5f&&gyro_notch_q<=20)) return false;
    for(unsigned i=0;i<2;++i) for(unsigned j=0;j<3;++j)
        if(!(accel_scale[i][j]>.8f&&accel_scale[i][j]<1.2f&&
             accel_offset_g[i][j]>-.2f&&accel_offset_g[i][j]<.2f)) return false;
    return true;
}
static_assert(sensor_config_valid(),"Invalid IMU filter/calibration configuration");
// Bench defaults. Validate output direction/endpoints on the actual airframe.
constexpr uint32_t pwm_hz_ailerons = 50;
constexpr uint32_t pwm_hz_tail = 50;
constexpr uint32_t pwm_hz_motors = 50;
// Enable only after electrical rework and the documented propeller-off tests.
#ifndef FC_FLIGHT_ENABLED
#define FC_FLIGHT_ENABLED 0
#endif
constexpr bool flight_enabled = FC_FLIGHT_ENABLED != 0;
constexpr uint16_t servo_min[8] = {1000,1000,1000,1000,1000,1000,1000,1000};
constexpr uint16_t servo_center[8] = {1500,1500,1500,1500,1500,1500,1500,1500};
constexpr uint16_t servo_max[8] = {2000,2000,2000,2000,2000,2000,2000,2000};
constexpr bool servo_reverse[8] = {false,false,false,false,false,true,false,false};
// Each entry is the sensor axis feeding body X,Y,Z (FRD): +/-1=X, +/-2=Y,
// +/-3=Z. Must be a proper rotation, determinant +1. These entries
// match v2.2: components upward, nose toward decreasing PCB X (Y2 side).
// Verify physically before enabling flight; see docs/HARDWARE_V2.md.
constexpr int8_t imu_rotation[2][3] = {{1,-2,-3},{1,-2,-3}};
constexpr bool imu_orientation_confirmed = true;
constexpr uint8_t bmp581_address = 0x47;
constexpr uint8_t bmm350_address = 0x14;
#ifndef FC_SD_LOGGING
#define FC_SD_LOGGING 1
#endif
constexpr bool enable_sd_logging = FC_SD_LOGGING != 0;
constexpr bool outputs_valid() {
    for(unsigned i=0;i<8;++i) if(servo_min[i]<800 || servo_max[i]>2200 ||
        servo_min[i]>=servo_center[i] || servo_center[i]>=servo_max[i]) return false;
    for(unsigned i=0;i<8;++i) if(control::is_motor_output(i) && servo_reverse[i]) return false;
    return true;
}
static_assert(outputs_valid(), "Invalid endpoints or reversed ESC pulse range");
}
