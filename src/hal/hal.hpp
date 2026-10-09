#pragma once
//
// hal.hpp -- the hardware abstraction interface.
//
// The ONLY layer allowed to touch STM32 / CMSIS / Arduino is src/hal/stm32/.
// Everything in drivers/, estimation/, control/, modes/ includes this header
// and nothing else platform-specific, so it also compiles for [env:native].
//
// Conventions
//   - Time is uint32_t microseconds / milliseconds from micros()/millis().
//     Both wrap; compare by subtraction only:  (now - then) > timeout.
//   - Every call that can fail returns hal::Status. No silent failure.
//   - No dynamic allocation. Buffers are caller-owned.
//   - "blocking" calls are bounded and meant for init() only. Nothing in a
//     scheduler task may call a blocking hal function.
//
#include <cstddef>
#include <cstdint>

namespace hal {

enum class Status : int8_t {
    ok          =  0,
    error       = -1,
    timeout     = -2,
    busy        = -3,
    unsupported = -4,   // capability declared but not implemented on this backend yet
    nack        = -5,   // address/data not acknowledged
};

// Semantic peripheral IDs. The backend maps these to concrete instances / pins;
// nothing above hal/ names a port, an alternate function, or a DMA stream.
enum class SpiBus   : uint8_t { imu, imu2 };
enum class I2cBus   : uint8_t { baro, mag };
enum class Uart     : uint8_t { gps, crsf };        // USART1 PA9/PA10 ; USART2 PA2/PA3
enum class PwmGroup : uint8_t { ailerons, tail, motors }; // TIM3, TIM4, TIM1

enum class PinMode : uint8_t { input, input_pullup, output };

enum class ResetCause : uint8_t {
    power_on, pin, software, iwdg, wwdg, low_power, brownout, unknown
};

// Opaque GPIO handle. On STM32 it is the Arduino pin number; on native it is an
// index into a scratch array. Named pins live in hal::pins (backend-defined).
using PinId = uint32_t;

namespace pins {
// IMU chip-select. SCK/MISO/MOSI are owned by the SPI peripheral (SpiBus::imu)
// and never touched as GPIO.
extern const PinId imu_cs;
extern const PinId imu2_cs;
} // namespace pins

// --------------------------------------------------------------------------
// Lifecycle and time
// --------------------------------------------------------------------------

// Bring up anything the other hal calls assume. Call once, first thing.
void init();
bool board_configured();

uint32_t micros();   // wraps ~1.19 h
uint32_t millis();   // wraps ~49.7 days

// Free-running CPU cycle counter for per-task profiling. Wraps (32-bit: ~25 s
// at 168 MHz). STM32: DWT->CYCCNT. native: nanoseconds, so cpu_hz() == 1e9.
uint32_t cycles();
uint32_t cpu_hz();   // cycles() ticks per second

// Blocking. init() / bring-up only -- never from a scheduler task.
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

// --------------------------------------------------------------------------
// Independent watchdog. Once started it cannot be stopped. Kick it well
// inside the timeout or the MCU resets (see hal::reset_cause()).
// --------------------------------------------------------------------------

void watchdog_start(uint32_t timeout_ms);
void watchdog_kick();

// --------------------------------------------------------------------------
// GPIO -- chip-selects and the odd status line. Not a bit-bang bus.
// --------------------------------------------------------------------------

void gpio_config(PinId pin, PinMode mode);
void gpio_write (PinId pin, bool level);
bool gpio_read  (PinId pin);

// --------------------------------------------------------------------------
// SPI -- full-duplex. tx or rx may be null (that direction is 0x00 / discard).
// CS is asserted for the whole transfer and released on return.
// --------------------------------------------------------------------------

Status spi_config    (SpiBus bus, uint32_t hz, uint8_t mode /*0..3*/);
Status spi_xfer      (SpiBus bus, PinId cs, const uint8_t* tx, uint8_t* rx, size_t n); // blocking
Status spi_xfer_async(SpiBus bus, PinId cs, const uint8_t* tx, uint8_t* rx, size_t n); // DMA; poll spi_busy()
bool   spi_busy      (SpiBus bus);

// --------------------------------------------------------------------------
// I2C -- write wn bytes, repeated-start, read rn bytes. wn or rn may be 0.
// Bounded blocking.
// --------------------------------------------------------------------------

Status i2c_config    (I2cBus bus, uint32_t hz);
Status i2c_write_read(I2cBus bus, uint8_t addr7,
                      const uint8_t* wr, size_t wn,
                      uint8_t* rd, size_t rn);

// --------------------------------------------------------------------------
// UART -- DMA circular RX, non-blocking TX. 8N1.
// --------------------------------------------------------------------------

Status uart_config      (Uart u, uint32_t baud);
size_t uart_rx_available(Uart u);
size_t uart_read        (Uart u, uint8_t* buf, size_t max);   // drains RX ring, never blocks
size_t uart_write       (Uart u, const uint8_t* buf, size_t n); // enqueues; returns accepted count
size_t uart_write_space (Uart u);   // bytes the TX ring can take right now (for atomic-frame writes)
bool   uart_tx_idle     (Uart u);

// --------------------------------------------------------------------------
// PWM servo / ESC output. (No caller yet -- Phase 2.)
// --------------------------------------------------------------------------

Status pwm_config  (PwmGroup g, uint32_t frame_hz);              // 50 Hz by default
Status pwm_write_us(PwmGroup g, uint8_t channel /*0..3*/, uint16_t pulse_us);

// --------------------------------------------------------------------------
// SD block device -- 512-byte sectors. FatFs diskio binds here. (No caller
// yet -- the bench CSV logger still drives the vendored STM32SD stack directly;
// this surface goes live with the binary logger in Phase 0.)
// --------------------------------------------------------------------------

Status   blk_init();
uint32_t blk_sector_count();
Status   blk_read (uint32_t lba, uint8_t* buf, uint32_t count);
Status   blk_write(uint32_t lba, const uint8_t* buf, uint32_t count);

// --------------------------------------------------------------------------
// Misc
// --------------------------------------------------------------------------

ResetCause     reset_cause();
[[noreturn]] void jump_to_bootloader();   // enter the ROM DFU bootloader; no BOOT0 pin

} // namespace hal
