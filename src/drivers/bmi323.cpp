#include "bmi323.hpp"
#include "../hal/hal.hpp"

#include <cstring>

#include "bmi323.h"   // vendored Bosch API (lib/bmi323)

namespace bmi323 {
namespace {

using hal::pins::imu_cs;
using hal::pins::imu_sck;
using hal::pins::imu_miso;
using hal::pins::imu_mosi;

// Configured full-scale ranges. These two constants are the single source of
// truth: the raw->physical scale factors are derived from them so the
// conversion can never fall out of sync with the sensor config (e.g. when the
// accel range moves to +/-8 g per the Phase 0 target).
//   accel FS = 2^(range+1) g            (2G=0 .. 16G=3)
//   gyro  FS = 2000 / 2^(2000dps-range) (125dps=0 .. 2000dps=4)
constexpr uint8_t kAccRange = BMI3_ACC_RANGE_4G;
constexpr uint8_t kGyrRange = BMI3_GYR_RANGE_2000DPS;

constexpr float kAccLsbG =
    static_cast<float>(2u << kAccRange) / 32768.0f;
constexpr float kGyrLsbDps =
    (2000.0f / static_cast<float>(1u << (BMI3_GYR_RANGE_2000DPS - kGyrRange))) / 32768.0f;

bmi3_dev s_dev;

// --- bit-bang SPI mode 0 (sample MISO on rising SCK) --------------------------
uint8_t bb_xfer(uint8_t out)
{
    uint8_t in = 0;
    for (int i = 7; i >= 0; i--) {
        hal::gpio_write(imu_mosi, (out >> i) & 1);
        hal::gpio_write(imu_sck, true);
        hal::delay_us(2);
        if (hal::gpio_read(imu_miso)) in |= (1 << i);
        hal::gpio_write(imu_sck, false);
        hal::delay_us(2);
    }
    return in;
}

int8_t spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length, void*)
{
    if (length == 0) return BMI3_INTF_RET_SUCCESS;
    hal::gpio_write(imu_cs, false);
    hal::delay_us(1);
    bb_xfer(reg_addr);   // driver already set the 0x80 read bit
    for (uint32_t i = 0; i < length; i++) reg_data[i] = bb_xfer(0x00);
    hal::delay_us(1);
    hal::gpio_write(imu_cs, true);
    return BMI3_INTF_RET_SUCCESS;
}

int8_t spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length, void*)
{
    if (length == 0) return BMI3_INTF_RET_SUCCESS;
    hal::gpio_write(imu_cs, false);
    hal::delay_us(1);
    bb_xfer(reg_addr);   // driver already applied the 0x7F write mask
    for (uint32_t i = 0; i < length; i++) bb_xfer(reg_data[i]);
    hal::delay_us(1);
    hal::gpio_write(imu_cs, true);
    return BMI3_INTF_RET_SUCCESS;
}

void spi_delay_us(uint32_t period, void*) { hal::delay_us(period); }

} // namespace

void config_pins()
{
    hal::gpio_config(imu_cs, hal::PinMode::output);   hal::gpio_write(imu_cs, true);
    hal::gpio_config(imu_sck, hal::PinMode::output);  hal::gpio_write(imu_sck, false);
    hal::gpio_config(imu_mosi, hal::PinMode::output); hal::gpio_write(imu_mosi, false);
    hal::gpio_config(imu_miso, hal::PinMode::input_pullup);
}

bool begin()
{
    std::memset(&s_dev, 0, sizeof(s_dev));
    s_dev.intf           = BMI3_SPI_INTF;
    s_dev.read           = spi_read;
    s_dev.write          = spi_write;
    s_dev.delay_us       = spi_delay_us;
    s_dev.read_write_len = 8;

    if (bmi323_init(&s_dev) != BMI3_OK) return false;   // also validates chip id 0x43

    bmi3_sens_config cfg[2];
    cfg[0].type = BMI323_ACCEL;
    cfg[1].type = BMI323_GYRO;
    if (bmi323_get_sensor_config(cfg, 2, &s_dev) != BMI3_OK) return false;

    cfg[0].cfg.acc.odr      = BMI3_ACC_ODR_200HZ;
    cfg[0].cfg.acc.range    = kAccRange;
    cfg[0].cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;
    cfg[0].cfg.acc.avg_num  = BMI3_ACC_AVG4;
    cfg[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    cfg[1].cfg.gyr.odr      = BMI3_GYR_ODR_200HZ;
    cfg[1].cfg.gyr.range    = kGyrRange;
    cfg[1].cfg.gyr.bwp      = BMI3_GYR_BW_ODR_HALF;
    cfg[1].cfg.gyr.avg_num  = BMI3_GYR_AVG1;
    cfg[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    return bmi323_set_sensor_config(cfg, 2, &s_dev) == BMI3_OK;
}

bool read(Sample& out)
{
    bmi3_sensor_data data[2];
    data[0].type = BMI323_ACCEL;
    data[1].type = BMI323_GYRO;
    if (bmi323_get_sensor_data(data, 2, &s_dev) != BMI3_OK) return false;

    out.ax_g   = static_cast<float>(data[0].sens_data.acc.x) * kAccLsbG;
    out.ay_g   = static_cast<float>(data[0].sens_data.acc.y) * kAccLsbG;
    out.az_g   = static_cast<float>(data[0].sens_data.acc.z) * kAccLsbG;
    out.gx_dps = static_cast<float>(data[1].sens_data.gyr.x) * kGyrLsbDps;
    out.gy_dps = static_cast<float>(data[1].sens_data.gyr.y) * kGyrLsbDps;
    out.gz_dps = static_cast<float>(data[1].sens_data.gyr.z) * kGyrLsbDps;
    return true;
}

uint8_t raw_chip_id()
{
    uint8_t b1;
    hal::gpio_write(imu_cs, false);
    hal::delay_us(1);
    bb_xfer(0x80);          // read register 0x00
    (void)bb_xfer(0x00);    // dummy byte
    b1 = bb_xfer(0x00);     // chip id (0x43 for BMI323)
    hal::delay_us(1);
    hal::gpio_write(imu_cs, true);
    return b1;
}

} // namespace bmi323
