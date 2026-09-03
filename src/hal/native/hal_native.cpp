//
// hal_native.cpp -- desktop backend for hal::.  Used by [env:native] so the
// portable layers (drivers/, estimation/) can be built and unit-tested off
// target. Hardware calls are stubs; time is a real monotonic clock.
//
#include "../hal.hpp"

#include <chrono>
#include <thread>

const hal::PinId hal::pins::imu_cs   = 0;
const hal::PinId hal::pins::imu_sck  = 1;
const hal::PinId hal::pins::imu_miso = 2;
const hal::PinId hal::pins::imu_mosi = 3;

namespace hal {
namespace {

std::chrono::steady_clock::time_point s_t0;

} // namespace

void init() { s_t0 = std::chrono::steady_clock::now(); }

uint32_t micros()
{
    using namespace std::chrono;
    return (uint32_t)duration_cast<microseconds>(steady_clock::now() - s_t0).count();
}
uint32_t millis()
{
    using namespace std::chrono;
    return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - s_t0).count();
}
uint32_t cycles()
{
    using namespace std::chrono;
    return (uint32_t)duration_cast<nanoseconds>(steady_clock::now() - s_t0).count();
}
uint32_t cpu_hz() { return 1000000000u; }   // cycles() is nanoseconds on native

void delay_ms(uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
void delay_us(uint32_t us) { std::this_thread::sleep_for(std::chrono::microseconds(us)); }

void watchdog_start(uint32_t) {}
void watchdog_kick() {}

void gpio_config(PinId, PinMode) {}
void gpio_write (PinId, bool)    {}
bool gpio_read  (PinId)          { return false; }

Status spi_config    (SpiBus, uint32_t, uint8_t)                       { return Status::unsupported; }
Status spi_xfer      (SpiBus, PinId, const uint8_t*, uint8_t*, size_t) { return Status::unsupported; }
Status spi_xfer_async(SpiBus, PinId, const uint8_t*, uint8_t*, size_t) { return Status::unsupported; }
bool   spi_busy      (SpiBus)                                          { return false; }

Status i2c_config    (I2cBus, uint32_t)                                { return Status::unsupported; }
Status i2c_write_read(I2cBus, uint8_t, const uint8_t*, size_t, uint8_t*, size_t) { return Status::unsupported; }

Status uart_config      (Uart, uint32_t)              { return Status::unsupported; }
size_t uart_rx_available(Uart)                        { return 0; }
size_t uart_read        (Uart, uint8_t*, size_t)      { return 0; }
size_t uart_write       (Uart, const uint8_t*, size_t n) { return n; }
bool   uart_tx_idle     (Uart)                        { return true; }

Status pwm_config  (PwmGroup, uint32_t)               { return Status::unsupported; }
Status pwm_write_us(PwmGroup, uint8_t, uint16_t)      { return Status::unsupported; }

Status   blk_init()                                   { return Status::unsupported; }
uint32_t blk_sector_count()                           { return 0; }
Status   blk_read (uint32_t, uint8_t*, uint32_t)      { return Status::unsupported; }
Status   blk_write(uint32_t, const uint8_t*, uint32_t){ return Status::unsupported; }

ResetCause reset_cause() { return ResetCause::unknown; }
void jump_to_bootloader() { while (true) {} }

} // namespace hal
