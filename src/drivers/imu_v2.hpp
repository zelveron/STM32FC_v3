#pragma once
#include "../estimation/imu_prep.hpp"
namespace imu_v2 {
using Observer=void(*)(unsigned sensor,uint32_t host_us,uint32_t sequence,uint32_t sensor_clock,
                      const float* raw,const estimation::ImuSample& filtered);
void set_observer(Observer observer);
bool begin();
void poll(bool allow_calibration);
bool healthy();
bool bias_ready();
bool sensor_healthy(unsigned index);
bool ambiguous();
unsigned active();
int driver_error(unsigned index);
uint32_t driver_health_registers(unsigned index);
const estimation::ImuSample& latest();
}
