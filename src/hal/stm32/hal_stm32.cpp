//
// hal_stm32.cpp -- STM32F407 / Arduino-core backend for hal::.
//
// This is the ONLY translation unit permitted to include Arduino / STM32 HAL /
// CMSIS headers. Everything else reaches hardware through hal.hpp.
//
#include "../hal.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareTimer.h>

// ---------------------------------------------------------------------------
// Named pins (declared extern in hal.hpp)
// ---------------------------------------------------------------------------
const hal::PinId hal::pins::imu_cs = PA4;   // SCK/MISO/MOSI = PA5/PA6/PA7 (SPI1)

namespace hal {

// ---------------------------------------------------------------------------
// Lifecycle / time
// ---------------------------------------------------------------------------
void init()
{
    // The Arduino core has already configured the clock tree, SysTick and the
    // GPIO clocks. Enable the DWT cycle counter for hal::cycles().
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t micros() { return ::micros(); }
uint32_t millis() { return ::millis(); }
uint32_t cycles() { return DWT->CYCCNT; }
uint32_t cpu_hz() { return SystemCoreClock; }   // 168 MHz on this board

void delay_ms(uint32_t ms) { ::delay(ms); }
void delay_us(uint32_t us) { ::delayMicroseconds(us); }

// --- IWDG -----------------------------------------------------------------
// LSI ~32 kHz, prescaler /32 -> ~1 kHz -> 1 reload tick ~= 1 ms.
// Max reload 0xFFF -> ~4.095 s.
void watchdog_start(uint32_t timeout_ms)
{
    if (timeout_ms > 4095) timeout_ms = 4095;
    IWDG->KR  = 0x5555;          // enable register write access
    IWDG->PR  = 3;               // /32
    IWDG->RLR = timeout_ms;      // ~1 ms per tick
    IWDG->KR  = 0xAAAA;          // reload
    IWDG->KR  = 0xCCCC;          // start
}
void watchdog_kick() { IWDG->KR = 0xAAAA; }

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
// SPI -- SPI1 (PA5/PA6/PA7) for the BMI323, blocking. The transfer is a tight
// polled LL loop inside the core (~1 byte / SPI clock), so a 27-byte IMU burst
// at 5.25 MHz is ~45 us. DMA (spi_xfer_async) is a later optimisation.
// ---------------------------------------------------------------------------
static SPISettings s_spi_settings;
static bool        s_spi_ready = false;

Status spi_config(SpiBus bus, uint32_t hz, uint8_t mode)
{
    if (bus != SpiBus::imu) return Status::unsupported;
    const uint8_t m = (mode == 0) ? SPI_MODE0 : (mode == 1) ? SPI_MODE1
                    : (mode == 2) ? SPI_MODE2 : SPI_MODE3;
    s_spi_settings = SPISettings(hz, MSBFIRST, m);
    SPI.setMOSI(PA7);
    SPI.setMISO(PA6);
    SPI.setSCLK(PA5);
    SPI.begin();
    s_spi_ready = true;
    return Status::ok;
}

Status spi_xfer(SpiBus bus, PinId cs, const uint8_t* tx, uint8_t* rx, size_t n)
{
    if (bus != SpiBus::imu || !s_spi_ready || n == 0) return Status::error;

    SPI.beginTransaction(s_spi_settings);
    ::digitalWrite(cs, LOW);
    if (tx && rx)      SPI.transfer(tx, rx, n);
    else if (rx)       { for (size_t i = 0; i < n; i++) rx[i] = SPI.transfer(0x00); }
    else               { for (size_t i = 0; i < n; i++) SPI.transfer(tx ? tx[i] : 0x00); }
    ::digitalWrite(cs, HIGH);
    SPI.endTransaction();
    return Status::ok;
}

Status spi_xfer_async(SpiBus, PinId, const uint8_t*, uint8_t*, size_t) { return Status::unsupported; }
bool   spi_busy      (SpiBus) { return false; }

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
// UART
//   GPS  -- USART1, crossed wiring: MCU TX = PA9, MCU RX = PA10.
//   CRSF -- USART3: MCU RX = PB11 (<- RX TX), MCU TX = PB10 (-> RX RX). Pins
//           are bound by the s_crsf constructor. TODO: DMA circular RX + IDLE.
// ---------------------------------------------------------------------------
static HardwareSerial s_crsf(PB11, PB10);   // (rx, tx) -> selects USART3

static HardwareSerial* port_for(Uart u)
{
    switch (u) {
        case Uart::gps:  return &Serial1;
        case Uart::crsf: return &s_crsf;
    }
    return nullptr;
}

Status uart_config(Uart u, uint32_t baud)
{
    HardwareSerial* p = port_for(u);
    if (!p) return Status::unsupported;
    p->end();
    if (u == Uart::gps) { p->setTx(PA9); p->setRx(PA10); }  // crsf pins fixed by ctor
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
size_t uart_write_space(Uart u)
{
    HardwareSerial* p = port_for(u);
    if (!p) return 0;
    const int space = p->availableForWrite();
    return space > 0 ? (size_t)space : 0;
}

bool uart_tx_idle(Uart u)
{
    HardwareSerial* p = port_for(u);
    if (!p) return true;
    return p->availableForWrite() >= (int)(SERIAL_TX_BUFFER_SIZE - 1);
}

// ---------------------------------------------------------------------------
// PWM servo / ESC output.
//   out_1_4 -> TIM4 CH1..4  = PD12 PD13 PD14 PD15
//   out_5_8 -> TIM1 CH1..4  = PE9  PE11 PE13 PE14
// setPWM() inits each channel (0% duty = no pulse until pwm_write_us). The
// pulse is then set directly in microseconds. Channels are 0-indexed here,
// 1-indexed in HardwareTimer.
// ---------------------------------------------------------------------------
static const uint32_t kPwmPins14[4] = { PD12, PD13, PD14, PD15 };
static const uint32_t kPwmPins58[4] = { PE9,  PE11, PE13, PE14 };

static HardwareTimer* pwm_timer(PwmGroup g)
{
    if (g == PwmGroup::out_1_4) { static HardwareTimer t(TIM4); return &t; }
    else                        { static HardwareTimer t(TIM1); return &t; }
}

Status pwm_config(PwmGroup g, uint32_t frame_hz)
{
    HardwareTimer* t = pwm_timer(g);
    const uint32_t* pins = (g == PwmGroup::out_1_4) ? kPwmPins14 : kPwmPins58;
    for (uint32_t ch = 0; ch < 4; ch++)
        t->setPWM(ch + 1, pins[ch], frame_hz, 0);   // 0% duty -> no pulse yet
    return Status::ok;
}

Status pwm_write_us(PwmGroup g, uint8_t channel, uint16_t pulse_us)
{
    if (channel > 3) return Status::error;
    pwm_timer(g)->setCaptureCompare(channel + 1, pulse_us, MICROSEC_COMPARE_FORMAT);
    return Status::ok;
}

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
