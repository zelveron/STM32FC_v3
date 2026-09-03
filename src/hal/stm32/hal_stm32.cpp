//
// hal_stm32.cpp -- STM32F407 / Arduino-core backend for hal::.
//
// This is the ONLY translation unit permitted to include Arduino / STM32 HAL /
// CMSIS headers. Everything else reaches hardware through hal.hpp.
//
#include "../hal.hpp"

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Named pins (declared extern in hal.hpp)
// ---------------------------------------------------------------------------
const hal::PinId hal::pins::imu_cs   = PA4;
const hal::PinId hal::pins::imu_sck  = PA5;
const hal::PinId hal::pins::imu_miso = PA6;
const hal::PinId hal::pins::imu_mosi = PA7;

namespace hal {

// ---------------------------------------------------------------------------
// Lifecycle / time
// ---------------------------------------------------------------------------
void init()
{
    // The Arduino core has already configured the clock tree, SysTick and the
    // GPIO clocks by the time this runs. Nothing else is needed yet.
}

uint32_t micros() { return ::micros(); }
uint32_t millis() { return ::millis(); }

void delay_ms(uint32_t ms) { ::delay(ms); }
void delay_us(uint32_t us) { ::delayMicroseconds(us); }

// ---------------------------------------------------------------------------
// GPIO (BMI323 bit-bang shim only)
// ---------------------------------------------------------------------------
void gpio_config(PinId pin, PinMode mode)
{
    switch (mode) {
        case PinMode::input:        ::pinMode(pin, INPUT);        break;
        case PinMode::input_pullup: ::pinMode(pin, INPUT_PULLUP); break;
        case PinMode::output:       ::pinMode(pin, OUTPUT);       break;
    }
}
void gpio_write(PinId pin, bool level) { ::digitalWrite(pin, level ? HIGH : LOW); }
bool gpio_read (PinId pin)             { return ::digitalRead(pin) != LOW; }

// ---------------------------------------------------------------------------
// SPI -- not implemented. The IMU is on the bit-bang GPIO path until the
// BMI323 solder joints are reflowed and hardware SPI at 10 MHz is verified
// (README blocker #1). Real SPI1 + DMA lands in that session.
// ---------------------------------------------------------------------------
Status spi_config    (SpiBus, uint32_t, uint8_t)                 { return Status::unsupported; }
Status spi_xfer      (SpiBus, PinId, const uint8_t*, uint8_t*, size_t) { return Status::unsupported; }
Status spi_xfer_async(SpiBus, PinId, const uint8_t*, uint8_t*, size_t) { return Status::unsupported; }
bool   spi_busy      (SpiBus)                                    { return false; }

// ---------------------------------------------------------------------------
// I2C -- BMP581 on I2C1 (PB6/PB7)
// ---------------------------------------------------------------------------
Status i2c_config(I2cBus bus, uint32_t hz)
{
    if (bus != I2cBus::baro) return Status::unsupported;
    Wire.setSCL(PB6);
    Wire.setSDA(PB7);
    Wire.setClock(hz);
    Wire.begin();
    return Status::ok;
}

Status i2c_write_read(I2cBus bus, uint8_t addr7,
                      const uint8_t* wr, size_t wn,
                      uint8_t* rd, size_t rn)
{
    if (bus != I2cBus::baro) return Status::unsupported;

    if (wn > 0) {
        Wire.beginTransmission(addr7);
        Wire.write(wr, wn);
        // repeated start if a read follows, STOP otherwise
        if (Wire.endTransmission(rn == 0) != 0) return Status::error;
    }
    if (rn > 0) {
        if (Wire.requestFrom((uint8_t)addr7, (uint8_t)rn) != rn) return Status::error;
        for (size_t i = 0; i < rn; i++) rd[i] = (uint8_t)Wire.read();
    }
    return Status::ok;
}

// ---------------------------------------------------------------------------
// UART -- GPS on USART1 (crossed wiring: MCU TX = PA9, MCU RX = PA10).
// CRSF (USART2) has no caller yet.
// ---------------------------------------------------------------------------
static HardwareSerial* port_for(Uart u)
{
    return (u == Uart::gps) ? &Serial1 : nullptr;
}

Status uart_config(Uart u, uint32_t baud)
{
    HardwareSerial* p = port_for(u);
    if (!p) return Status::unsupported;
    p->end();
    p->setTx(PA9);
    p->setRx(PA10);
    p->begin(baud);
    return Status::ok;
}

size_t uart_rx_available(Uart u)
{
    HardwareSerial* p = port_for(u);
    if (!p) return 0;
    int a = p->available();
    return a > 0 ? (size_t)a : 0;
}

size_t uart_read(Uart u, uint8_t* buf, size_t max)
{
    HardwareSerial* p = port_for(u);
    if (!p) return 0;
    size_t n = 0;
    while (n < max && p->available() > 0) buf[n++] = (uint8_t)p->read();
    return n;
}

size_t uart_write(Uart u, const uint8_t* buf, size_t n)
{
    HardwareSerial* p = port_for(u);
    if (!p) return 0;
    // Non-blocking: never write more than the TX ring can take right now.
    int space = p->availableForWrite();
    if (space <= 0) return 0;
    size_t k = ((size_t)space < n) ? (size_t)space : n;
    return p->write(buf, k);
}

bool uart_tx_idle(Uart u)
{
    HardwareSerial* p = port_for(u);
    if (!p) return true;
    return p->availableForWrite() >= (int)(SERIAL_TX_BUFFER_SIZE - 1);
}

// ---------------------------------------------------------------------------
// PWM -- not implemented (Phase 2: servo / ESC output).
// ---------------------------------------------------------------------------
Status pwm_config  (PwmGroup, uint32_t)                 { return Status::unsupported; }
Status pwm_write_us(PwmGroup, uint8_t, uint16_t)        { return Status::unsupported; }

// ---------------------------------------------------------------------------
// SD block device -- not implemented. The bench CSV logger still drives the
// vendored STM32SD / FatFs stack directly (README known issue #2: those trees
// are patched and must not be regenerated). This binds when binary logging
// replaces the CSV logger.
// ---------------------------------------------------------------------------
Status   blk_init()                                     { return Status::unsupported; }
uint32_t blk_sector_count()                             { return 0; }
Status   blk_read (uint32_t, uint8_t*, uint32_t)        { return Status::unsupported; }
Status   blk_write(uint32_t, const uint8_t*, uint32_t)  { return Status::unsupported; }

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------
ResetCause reset_cause()
{
    const uint32_t csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;   // clear the flags for next boot

    if (csr & RCC_CSR_LPWRRSTF) return ResetCause::low_power;
    if (csr & RCC_CSR_WWDGRSTF) return ResetCause::wwdg;
    if (csr & RCC_CSR_IWDGRSTF) return ResetCause::iwdg;
    if (csr & RCC_CSR_SFTRSTF)  return ResetCause::software;
    if (csr & RCC_CSR_BORRSTF)  return ResetCause::brownout;
    if (csr & RCC_CSR_PINRSTF)  return ResetCause::pin;
    if (csr & RCC_CSR_PORRSTF)  return ResetCause::power_on;
    return ResetCause::unknown;
}

void jump_to_bootloader()
{
    // STM32F407 system-memory (ROM DFU) bootloader entry.
    const uint32_t kSystemMemoryBase = 0x1FFF0000UL;

    __disable_irq();
    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    const uint32_t sp = *reinterpret_cast<const uint32_t*>(kSystemMemoryBase);
    const uint32_t pc = *reinterpret_cast<const uint32_t*>(kSystemMemoryBase + 4);

    __set_MSP(sp);
    reinterpret_cast<void (*)(void)>(pc)();

    while (true) { }   // not reached
}

} // namespace hal
