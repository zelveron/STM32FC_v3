//
// hal_stm32.cpp -- STM32F407 / Arduino-core backend for hal::.
//
// Hardware access for sensor/control modules. USB/SD and the Arduino entry
// point also include framework headers at their boundary.
//
#include "../hal.hpp"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareTimer.h>
#include "board_pins.hpp"

// ---------------------------------------------------------------------------
// Named pins (declared extern in hal.hpp)
// ---------------------------------------------------------------------------
const hal::PinId hal::pins::imu_cs = board::imu1_cs;
const hal::PinId hal::pins::imu2_cs = board::imu2_cs;

namespace hal {
bool board_configured() { return board::confirmed; }

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
    ::digitalWrite(board::imu1_cs,HIGH); ::pinMode(board::imu1_cs,OUTPUT);
    ::digitalWrite(board::imu2_cs,HIGH); ::pinMode(board::imu2_cs,OUTPUT);
    ::digitalWrite(PB0,HIGH); ::pinMode(PB0,OUTPUT); // unused W25Q16 deselected
    ::digitalWrite(board::gps_enable,LOW); ::pinMode(board::gps_enable,OUTPUT);
    ::digitalWrite(board::baro_enable,LOW); ::pinMode(board::baro_enable,OUTPUT);
    ::digitalWrite(board::gps_reset,LOW); ::pinMode(board::gps_reset,OUTPUT);
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
    IWDG->KR  = 0xCCCC;          // start LSI/watchdog before updating registers
    IWDG->KR  = 0x5555;          // enable register write access
    IWDG->PR  = 3;               // /32
    IWDG->RLR = timeout_ms;      // ~1 ms per tick
    const uint32_t started=::micros();
    while(IWDG->SR && uint32_t(::micros()-started)<20000) {}
    IWDG->KR  = 0xAAAA;          // reload after register synchronization
}
void watchdog_kick() { IWDG->KR = 0xAAAA; }

// ---------------------------------------------------------------------------
// GPIO
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
// SPI1 and SPI2: independent BMI270 buses. Transfers are synchronous.
// Measure actual timing; a stuck peripheral is contained by the watchdog.
// ---------------------------------------------------------------------------
static SPIClass s_spi1(board::imu1_mosi,board::imu1_miso,board::imu1_sck);
static SPIClass s_spi2(board::imu2_mosi,board::imu2_miso,board::imu2_sck);
static SPISettings s_spi_settings[2];
static bool s_spi_ready[2]={false,false};
static SPIClass& spi_for(SpiBus bus) { return bus==SpiBus::imu ? s_spi1 : s_spi2; }

Status spi_config(SpiBus bus,uint32_t hz,uint8_t mode)
{
    if (!board_configured()) return Status::unsupported;
    const unsigned i=static_cast<unsigned>(bus);
    if(i>1) return Status::unsupported;
    const uint8_t m=mode==0?SPI_MODE0:mode==1?SPI_MODE1:mode==2?SPI_MODE2:SPI_MODE3;
    s_spi_settings[i]=SPISettings(hz,MSBFIRST,m);
    spi_for(bus).begin(); s_spi_ready[i]=true; return Status::ok;
}
Status spi_xfer(SpiBus bus,PinId cs,const uint8_t* tx,uint8_t* rx,size_t n)
{
    const unsigned i=static_cast<unsigned>(bus);
    if(i>1 || !s_spi_ready[i] || n==0 || cs==NC) return Status::error;
    SPIClass& spi=spi_for(bus);
    spi.beginTransaction(s_spi_settings[i]); ::digitalWrite(cs,LOW);
    if(tx && rx) spi.transfer(tx,rx,n);
    else for(size_t k=0;k<n;++k) {
        const uint8_t v=spi.transfer(tx?tx[k]:0);
        if(rx) rx[k]=v;
    }
    ::digitalWrite(cs,HIGH); spi.endTransaction(); return Status::ok;
}

Status spi_xfer_async(SpiBus, PinId, const uint8_t*, uint8_t*, size_t) { return Status::unsupported; }
bool   spi_busy      (SpiBus) { return false; }

// ---------------------------------------------------------------------------
// I2C1 PB6/PB7: BMP581. I2C2 PB10/PB11: BMM350.
// ---------------------------------------------------------------------------
static TwoWire s_mag_wire(board::mag_sda,board::mag_scl);
static TwoWire& wire_for(I2cBus bus) { return bus==I2cBus::baro ? Wire : s_mag_wire; }
// BMM350 reads include two dummy bytes. Use the registered Wire IRQ handle,
// but caller-owned buffers and a whole-transaction microsecond deadline.
// Wire's millisecond timeout can truncate an otherwise normal burst depending
// on SysTick phase. No allocation, retry loop, or sensor reset occurs here.
static Status mag_transfer(uint8_t addr,const uint8_t* wr,size_t wn,uint8_t* rd,size_t rn)
{
    auto* h=s_mag_wire.getHandle();
    const uint32_t start=::micros();
    constexpr uint32_t budget_us=1800;
    auto fail=[&](Status status) {
        // An address NACK is expected for the absent boot-probe address.
        // Let the HAL's STOP finish; resetting a healthy peripheral here can
        // disturb the following back-to-back boot transactions.
        if(status==Status::nack && HAL_I2C_GetState(h)==HAL_I2C_STATE_READY) {
            while((__HAL_I2C_GET_FLAG(h,I2C_FLAG_BUSY)||(h->Instance->CR1&I2C_CR1_STOP))&&
                  uint32_t(::micros()-start)<budget_us) {}
            if(!__HAL_I2C_GET_FLAG(h,I2C_FLAG_BUSY)&&!(h->Instance->CR1&I2C_CR1_STOP)) {
                ::delayMicroseconds(2); // Fast-mode tBUF >= 1.3 us
                return status;
            }
        }
        // Cancel IRQ access to caller buffers before returning. Reinitialize
        // only the MCU peripheral, not the sensor or its calibration.
        const uint32_t mask=__get_PRIMASK(); __disable_irq();
        __HAL_I2C_DISABLE_IT(h,I2C_IT_EVT|I2C_IT_BUF|I2C_IT_ERR);
        HAL_I2C_Init(h); // fixed register setup; no bus waits or allocation
        h->Lock=HAL_UNLOCKED;
        if(!mask) __enable_irq();
        return status;
    };
    auto wait=[&]() {
        while(HAL_I2C_GetState(h)!=HAL_I2C_STATE_READY) {
            if(HAL_I2C_GetError(h)!=HAL_I2C_ERROR_NONE) break;
            if(uint32_t(::micros()-start)>=budget_us) return Status::timeout;
        }
        const uint32_t error=HAL_I2C_GetError(h);
        if(error&HAL_I2C_ERROR_AF) return Status::nack;
        if(error&HAL_I2C_ERROR_TIMEOUT) return Status::timeout;
        return error==HAL_I2C_ERROR_NONE?Status::ok:Status::error;
    };
    // Avoid the HAL's internal millisecond BUSY wait before starting a frame.
    if(HAL_I2C_GetState(h)!=HAL_I2C_STATE_READY) return fail(Status::busy);
    while(__HAL_I2C_GET_FLAG(h,I2C_FLAG_BUSY)||(h->Instance->CR1&I2C_CR1_STOP))
        if(uint32_t(::micros()-start)>=budget_us) return fail(Status::busy);
    if(wn) {
        if(HAL_I2C_Master_Seq_Transmit_IT(h,addr<<1,const_cast<uint8_t*>(wr),wn,
                    rn?I2C_FIRST_FRAME:I2C_FIRST_AND_LAST_FRAME)!=HAL_OK) return fail(Status::busy);
        const Status status=wait(); if(status!=Status::ok) return fail(status);
    }
    if(rn) {
        if(uint32_t(::micros()-start)>=budget_us) return fail(Status::timeout);
        if(HAL_I2C_Master_Seq_Receive_IT(h,addr<<1,rd,rn,
                    wn?I2C_LAST_FRAME:I2C_FIRST_AND_LAST_FRAME)!=HAL_OK) return fail(Status::busy);
        const Status status=wait(); if(status!=Status::ok) return fail(status);
    }
    while(__HAL_I2C_GET_FLAG(h,I2C_FLAG_BUSY)||(h->Instance->CR1&I2C_CR1_STOP))
        if(uint32_t(::micros()-start)>=budget_us) return fail(Status::timeout);
    ::delayMicroseconds(2); // Fast-mode minimum bus-free time after STOP
    return Status::ok;
}
Status i2c_config(I2cBus bus,uint32_t hz)
{
    if(!board_configured()) return Status::unsupported;
    TwoWire& wire=wire_for(bus);
    if(bus==I2cBus::baro) { wire.setSCL(board::baro_scl); wire.setSDA(board::baro_sda); }
    wire.begin(); wire.setClock(hz); return Status::ok;
}
Status i2c_write_read(I2cBus bus,uint8_t addr,const uint8_t* wr,size_t wn,uint8_t* rd,size_t rn)
{
    if(!board_configured() || rn>255 || wn>255 || (wn&&!wr) || (rn&&!rd)) return Status::unsupported;
    if(bus==I2cBus::mag) return mag_transfer(addr,wr,wn,rd,rn);
    TwoWire& wire=wire_for(bus);
    if(wn) {
        wire.beginTransmission(addr);
        if(wire.write(wr,wn)!=wn || wire.endTransmission(rn==0)!=0) return Status::error;
    }
    if(rn) {
        if(wire.requestFrom(addr,static_cast<uint8_t>(rn))!=rn) return Status::error;
        for(size_t i=0;i<rn;++i) rd[i]=uint8_t(wire.read());
    }
    return Status::ok;
}

// ---------------------------------------------------------------------------
// UART
//   GPS: USART2, TX PA2 / RX PA3. CRSF: UART4, TX PA0 / RX PA1.
//   Framework interrupt-driven RX ring; no DMA implementation yet.
// ---------------------------------------------------------------------------
static HardwareSerial s_crsf(board::rc_rx, board::rc_tx);
static HardwareSerial s_gps(board::gps_rx, board::gps_tx);

static HardwareSerial* port_for(Uart u)
{
    switch (u) {
        case Uart::gps:  return &s_gps;
        case Uart::crsf: return &s_crsf;
    }
    return nullptr;
}

Status uart_config(Uart u, uint32_t baud)
{
    if (!board_configured()) return Status::unsupported;
    HardwareSerial* p = port_for(u);
    if (!p) return Status::unsupported;
    p->end();
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
//   Ailerons: TIM3, PC7/PC6. Tail: TIM4, PD15/PD14/PD13/PD12.
//   Motors: TIM1, PA8/PA9. Physical SERVO7 (PD11) is intentionally unused.
// setPWM() inits each channel (0% duty = no pulse until pwm_write_us). The
// pulse is then set directly in microseconds. Channels are 0-indexed here,
// 1-indexed in HardwareTimer.
// ---------------------------------------------------------------------------
static HardwareTimer* pwm_timer(PwmGroup g)
{
    static HardwareTimer ailerons(TIM3),tail(TIM4),motors(TIM1);
    switch(g) {
        case PwmGroup::ailerons: return &ailerons;
        case PwmGroup::tail: return &tail;
        case PwmGroup::motors: return &motors;
    }
    return nullptr;
}
static const uint32_t* pwm_pins(PwmGroup g) {
    return g==PwmGroup::ailerons?board::pwm_ailerons:
           g==PwmGroup::tail?board::pwm_tail:board::pwm_motors;
}
static const uint8_t* pwm_channels(PwmGroup g) {
    return g==PwmGroup::ailerons?board::channels_ailerons:
           g==PwmGroup::tail?board::channels_tail:board::channels_motors;
}
Status pwm_config(PwmGroup g,uint32_t hz) {
    auto* timer=pwm_timer(g);
    if(!board_configured() || !timer || hz<40 || hz>400) return Status::error;
    const unsigned count=g==PwmGroup::tail?4:2;
    for(unsigned i=0;i<count;++i) timer->setPWM(pwm_channels(g)[i],pwm_pins(g)[i],hz,0);
    return Status::ok;
}
Status pwm_write_us(PwmGroup g,uint8_t ch,uint16_t pulse_us) {
    auto* timer=pwm_timer(g);
    if(!board_configured() || !timer || ch>=(g==PwmGroup::tail?4:2) || pulse_us<800 || pulse_us>2200) return Status::error;
    timer->setCaptureCompare(pwm_channels(g)[ch],pulse_us,MICROSEC_COMPARE_FORMAT);
    return Status::ok;
}

// ---------------------------------------------------------------------------
// Generic block HAL not implemented. The optional ground-only binary logger
// uses the patched STM32SD/FatFs stack directly.
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

} // namespace hal
