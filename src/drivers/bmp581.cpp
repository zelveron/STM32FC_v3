#include "bmp581.hpp"
#include "../hal/hal.hpp"

#include <cstring>

#include "bmp5.h"        // vendored Bosch API (lib/bmp5)
#include "bmp5_defs.h"

namespace bmp581 {
namespace {

constexpr uint8_t  kAddr       = 0x47;
constexpr uint32_t kI2cHz      = 100000;
// Worst-case 1x temp + 16x pressure oversampling conversion time, with margin.
constexpr uint32_t kConvUs    = 40000;
constexpr uint32_t kTriggerMs = 100;      // ~10 Hz

bmp5_dev                    s_dev;
bmp5_osr_odr_press_config   s_cfg;
bool                        s_ok          = false;
bool                        s_pending     = false;
uint32_t                    s_trig_us     = 0;
uint32_t                    s_last_trig_ms = 0;

int8_t i2c_read(uint8_t reg, uint8_t* data, uint32_t len, void*)
{
    return (hal::i2c_write_read(hal::I2cBus::baro, kAddr, &reg, 1, data, len)
            == hal::Status::ok) ? BMP5_INTF_RET_SUCCESS : BMP5_E_COM_FAIL;
}

int8_t i2c_write(uint8_t reg, const uint8_t* data, uint32_t len, void*)
{
    uint8_t buf[32];
    if (len + 1 > sizeof(buf)) return BMP5_E_COM_FAIL;
    buf[0] = reg;
    std::memcpy(buf + 1, data, len);
    return (hal::i2c_write_read(hal::I2cBus::baro, kAddr, buf, len + 1, nullptr, 0)
            == hal::Status::ok) ? BMP5_INTF_RET_SUCCESS : BMP5_E_COM_FAIL;
}

void i2c_delay_us(uint32_t period, void*) { hal::delay_us(period); }

} // namespace

bool begin()
{
    hal::i2c_config(hal::I2cBus::baro, kI2cHz);

    std::memset(&s_dev, 0, sizeof(s_dev));
    s_dev.intf     = BMP5_I2C_INTF;
    s_dev.read     = i2c_read;
    s_dev.write    = i2c_write;
    s_dev.delay_us = i2c_delay_us;

    if (bmp5_init(&s_dev) != BMP5_OK) return false;

    s_cfg.osr_t    = BMP5_OVERSAMPLING_1X;
    s_cfg.osr_p    = BMP5_OVERSAMPLING_16X;
    s_cfg.press_en = BMP5_ENABLE;
    s_cfg.odr      = BMP5_ODR_100_2_HZ;   // unused in forced mode

    if (bmp5_set_osr_odr_press_config(&s_cfg, &s_dev) != BMP5_OK) return false;

    s_ok      = true;
    s_pending = false;
    return true;
}

bool poll(Sample& out)
{
    if (!s_ok) return false;

    if (!s_pending) {
        if ((hal::millis() - s_last_trig_ms) >= kTriggerMs) {
            s_last_trig_ms = hal::millis();
            if (bmp5_set_power_mode(BMP5_POWERMODE_FORCED, &s_dev) == BMP5_OK) {
                s_trig_us = hal::micros();
                s_pending = true;
            }
        }
        return false;
    }

    if ((hal::micros() - s_trig_us) < kConvUs) return false;
    s_pending = false;

    bmp5_sensor_data data;
    if (bmp5_get_sensor_data(&data, &s_cfg, &s_dev) != BMP5_OK) return false;

    out.pressure_pa = static_cast<float>(data.pressure);
    out.temp_c      = static_cast<float>(data.temperature);
    return true;
}

} // namespace bmp581
