#include "bmi323.hpp"
#include "../hal/hal.hpp"

#include <cstring>

#include "bmi323.h"   // vendored Bosch API (lib/bmi323)

namespace bmi323 {
namespace {

constexpr hal::SpiBus kBus   = hal::SpiBus::imu;
// 8 MHz request -> SPI1 APB2/16 -> ~5.25 MHz actual, inside the 10 MHz
// datasheet limit. (10 MHz request -> ~10.5 MHz also verified clean over 1.4M
// reads by the imu_probe, if more speed is ever needed.)
constexpr uint32_t kSpiHz = 8000000;

// Full-scale ranges: single source of truth, scale factors derived from them.
//   accel FS = 2^(range+1) g            (2G=0 .. 16G=3)
//   gyro  FS = 2000 / 2^(2000dps-range) (125dps=0 .. 2000dps=4)
constexpr uint8_t kAccRange = BMI3_ACC_RANGE_8G;
constexpr uint8_t kGyrRange = BMI3_GYR_RANGE_2000DPS;
constexpr float kAccLsbG =
    static_cast<float>(2u << kAccRange) / 32768.0f;
constexpr float kGyrLsbDps =
    (2000.0f / static_cast<float>(1u << (BMI3_GYR_RANGE_2000DPS - kGyrRange))) / 32768.0f;

bmi3_dev s_dev;

// --- Bosch SPI callbacks over hal::spi_xfer -------------------------------
// The Bosch core sends the address itself as reg_addr (read bit / write mask
// already applied) then wants `len` bytes moved. We prepend the address into
// one transfer so the fast SPI.transfer(tx, rx, n) path is used.

int8_t spi_read(uint8_t reg, uint8_t* data, uint32_t len, void*)
{
    uint8_t tx[64] = { reg };
    uint8_t rx[64];
    if (len + 1 > sizeof(tx)) return BMI3_E_COM_FAIL;
    if (hal::spi_xfer(kBus, hal::pins::imu_cs, tx, rx, len + 1) != hal::Status::ok)
        return BMI3_E_COM_FAIL;
    for (uint32_t i = 0; i < len; i++) data[i] = rx[i + 1];
    return BMI3_INTF_RET_SUCCESS;
}

int8_t spi_write(uint8_t reg, const uint8_t* data, uint32_t len, void*)
{
    uint8_t tx[64];
    if (len + 1 > sizeof(tx)) return BMI3_E_COM_FAIL;
    tx[0] = reg;
    std::memcpy(tx + 1, data, len);
    if (hal::spi_xfer(kBus, hal::pins::imu_cs, tx, nullptr, len + 1) != hal::Status::ok)
        return BMI3_E_COM_FAIL;
    return BMI3_INTF_RET_SUCCESS;
}

void spi_delay_us(uint32_t us, void*) { hal::delay_us(us); }

} // namespace

bool begin()
{
    hal::gpio_config(hal::pins::imu_cs, hal::PinMode::output);
    hal::gpio_write(hal::pins::imu_cs, true);
    if (hal::spi_config(kBus, kSpiHz, 0) != hal::Status::ok) return false;   // mode 0

    std::memset(&s_dev, 0, sizeof(s_dev));
    s_dev.intf           = BMI3_SPI_INTF;
    s_dev.read           = spi_read;
    s_dev.write          = spi_write;
    s_dev.delay_us       = spi_delay_us;
    s_dev.read_write_len = 64;

    if (bmi323_init(&s_dev) != BMI3_OK) return false;   // soft reset + chip-id 0x43 check

    bmi3_sens_config c[2];
    c[0].type = BMI323_ACCEL;
    c[1].type = BMI323_GYRO;
    if (bmi323_get_sensor_config(c, 2, &s_dev) != BMI3_OK) return false;

    c[0].cfg.acc.odr      = BMI3_ACC_ODR_1600HZ;
    c[0].cfg.acc.range    = kAccRange;
    c[0].cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;   // internal filter on
    c[0].cfg.acc.avg_num  = BMI3_ACC_AVG1;
    c[0].cfg.acc.acc_mode = BMI3_ACC_MODE_HIGH_PERF;

    c[1].cfg.gyr.odr      = BMI3_GYR_ODR_1600HZ;
    c[1].cfg.gyr.range    = kGyrRange;
    c[1].cfg.gyr.bwp      = BMI3_GYR_BW_ODR_HALF;
    c[1].cfg.gyr.avg_num  = BMI3_GYR_AVG1;
    c[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_HIGH_PERF;

    return bmi323_set_sensor_config(c, 2, &s_dev) == BMI3_OK;
}

Result read(Sample& out)
{
    uint16_t status = 0;
    if (bmi3_get_sensor_status(&status, &s_dev) != BMI3_OK) return Result::comm_error;

    constexpr uint16_t kWant = BMI3_DRDY_ACC_MASK | BMI3_DRDY_GYR_MASK;
    if ((status & kWant) != kWant) return Result::no_data;

    bmi3_sensor_data d[2];
    d[0].type = BMI323_ACCEL;
    d[1].type = BMI323_GYRO;
    if (bmi323_get_sensor_data(d, 2, &s_dev) != BMI3_OK) return Result::comm_error;

    out.ax_g   = static_cast<float>(d[0].sens_data.acc.x) * kAccLsbG;
    out.ay_g   = static_cast<float>(d[0].sens_data.acc.y) * kAccLsbG;
    out.az_g   = static_cast<float>(d[0].sens_data.acc.z) * kAccLsbG;
    out.gx_dps = static_cast<float>(d[1].sens_data.gyr.x) * kGyrLsbDps;
    out.gy_dps = static_cast<float>(d[1].sens_data.gyr.y) * kGyrLsbDps;
    out.gz_dps = static_cast<float>(d[1].sens_data.gyr.z) * kGyrLsbDps;
    return Result::ok;
}

uint8_t raw_chip_id()
{
    uint8_t tx[3] = { 0x80, 0x00, 0x00 };   // reg 0x00 | read bit, dummy, id
    uint8_t rx[3] = { 0, 0, 0 };
    hal::spi_xfer(kBus, hal::pins::imu_cs, tx, rx, 3);
    return rx[2];
}

} // namespace bmi323
