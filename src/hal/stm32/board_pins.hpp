#pragma once
// Extracted from the supplied v2.2 schematic AND PCB (2026-09-28).
// Pin mapping is confirmed; this does not establish electrical/flight readiness.
namespace board {
constexpr bool confirmed = true;
constexpr uint32_t imu1_sck=PA5, imu1_miso=PA6, imu1_mosi=PA7, imu1_cs=PA4;
constexpr uint32_t imu2_sck=PB13, imu2_miso=PB14, imu2_mosi=PB15, imu2_cs=PB12;
constexpr uint32_t baro_scl=PB6, baro_sda=PB7, mag_scl=PB10, mag_sda=PB11;
constexpr uint32_t gps_rx=PA3, gps_tx=PA2, rc_rx=PA1, rc_tx=PA0;
constexpr uint32_t gps_enable=PE3, gps_reset=PE2, baro_enable=PA15;
constexpr uint32_t pwm_ailerons[2]={PC7,PC6};     // physical SERVO1,2
constexpr uint32_t pwm_tail[4]={PD15,PD14,PD13,PD12}; // physical SERVO3..6
constexpr uint32_t pwm_motors[2]={PA8,PA9};       // physical SERVO8,9
constexpr uint8_t channels_ailerons[2]={2,1};
constexpr uint8_t channels_tail[4]={4,3,2,1};
constexpr uint8_t channels_motors[2]={1,2};
// SERVO7 PD11 has no timer PWM alternate function. SERVO10 PA10 is reserved.
}
