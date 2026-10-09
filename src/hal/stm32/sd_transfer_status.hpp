#pragma once
#include "../../core/sd_async_card.hpp"

namespace storage::f407 {
// RM0090 SDIO_STA and DMA2_HISR stream 6 bit positions. The STM32 adapter
// statically checks these masks against the target's CMSIS definitions.
constexpr uint32_t data_errors=(1u<<1)|(1u<<3)|(1u<<4)|(1u<<9);
constexpr uint32_t dma_errors=(1u<<18)|(1u<<19); // DMEIF6, TEIF6
constexpr uint32_t fifo_warning=1u<<16;         // FEIF6
constexpr uint32_t data_end=1u<<8, dma_complete=1u<<21, stream_enabled=1u;

inline Reply write_status(uint32_t sta,uint32_t hisr,uint32_t cr) {
    // Match ST HAL SD_DMAError: FEIF alone is not a failed SD transfer.
    // It may assert while the DMA FIFO is being refilled. Real SD data
    // errors and DMA bus/direct-mode errors always take precedence.
    if((sta&data_errors)||(hisr&dma_errors)) return Reply::error;
    // A warning is never proof of completion. The caller still requires
    // CMD13 READY_FOR_DATA/TRANSFER, and enforces the whole-write timeout.
    return (sta&data_end)&&(hisr&dma_complete)&&!(cr&stream_enabled)
        ?Reply::ok:Reply::pending;
}
}
