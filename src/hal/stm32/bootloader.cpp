// STM32F407VG ROM entry. This translation unit must accompany our linker script.
#include "../hal.hpp"
#include <stm32f4xx.h>

namespace {
constexpr uint32_t magic = 0x44465531U; // "DFU1"
// NOLOAD, outside .bss and the stack/heap. See ld/stm32f407vg.ld.
__attribute__((section(".boot_request"), used, aligned(8)))
volatile uint32_t request[2];

void bootloader_check()
{
    const bool pending = request[0] == magic && request[1] == ~magic;
    // Consume even invalid/stale requests. Only an explicit software reset may
    // enter ROM; a power/brownout/watchdog reset always boots the application.
    request[0] = request[1] = 0;
    const uint32_t cause = RCC->CSR;
    if (!pending || !(cause & RCC_CSR_SFTRSTF) ||
        (cause & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF | RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))) return;

    // .preinit_array runs after SystemInit/.data/.bss, before Arduino premain:
    // HSI/reset clocks, no USB, SysTick, DMA, timers or watchdog started yet.
    // Remap as well as VTOR so ROM code sees its vectors at address zero.
    constexpr uint32_t rom = 0x1fff0000U;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->MEMRMP = 1;
    SCB->VTOR = rom;
    const uint32_t sp = *reinterpret_cast<const uint32_t*>(rom);
    const uint32_t pc = *reinterpret_cast<const uint32_t*>(rom + 4);
    // One assembly block: no C++ stack access after replacing MSP. Reset has
    // left PRIMASK/BASEPRI/CONTROL clear, and NVIC interrupts disabled.
    __asm__ volatile("dsb\n isb\n msr msp, %0\n bx %1" :: "r"(sp), "r"(pc) : "memory");
    __builtin_unreachable();
}

__attribute__((section(".preinit_array"), used))
void (*const early_boot_check)() = bootloader_check;
}

[[noreturn]] void hal::jump_to_bootloader()
{
    // A software reset stops the software-started IWDG and resets every
    // peripheral. Deinitializing HAL alone cannot stop an active watchdog.
    __disable_irq();
    request[1] = ~magic;
    request[0] = magic;
    RCC->CSR |= RCC_CSR_RMVF;
    __DSB();
    NVIC_SystemReset();
    while (true) {}
}
