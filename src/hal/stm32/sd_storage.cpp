// F407 SDIO / DMA2 stream 6 channel 4, dedicated to the logger after startup.
// Register transactions follow RM0090 and the ST HAL single-block write order.
// Each poll does bounded work; DMA and card programming run independently.
#include "../../core/sd_storage.hpp"
#include <Arduino.h>
#include <STM32SD.h>
#include <bsp_sd.h>
#include <cstdio>
extern "C" SD_HandleTypeDef* BSP_SD_Handle(void);
namespace storage {
namespace {
class F407Card final:public CardIo {
public:
    void command(uint8_t index,uint32_t arg) override {
        SDIO->ICR=SDIO_ICR_CCRCFAILC|SDIO_ICR_CTIMEOUTC|SDIO_ICR_CMDRENDC|SDIO_ICR_CMDSENTC;
        SDIO->ARG=arg;
        SDIO->CMD=index|SDIO_CMD_WAITRESP_0|SDIO_CMD_CPSMEN;
    }
    Reply reply(uint8_t index,uint32_t& response) override {
        const uint32_t status=SDIO->STA;
        if(status&(SDIO_STA_CCRCFAIL|SDIO_STA_CTIMEOUT)) return Reply::error;
        if(!(status&SDIO_STA_CMDREND)) return Reply::pending;
        if((SDIO->RESPCMD&63)!=index) return Reply::error;
        response=SDIO->RESP1; return Reply::ok;
    }
    bool transmit(const uint8_t* data) override {
        if((DMA2_Stream6->CR&DMA_SxCR_EN)||((uintptr_t)data&3)) return false;
        SDIO->DCTRL=0;
        SDIO->ICR=SDIO_ICR_DCRCFAILC|SDIO_ICR_DTIMEOUTC|SDIO_ICR_TXUNDERRC|
                   SDIO_ICR_DATAENDC|SDIO_ICR_DBCKENDC|SDIO_ICR_STBITERRC;
        DMA2->HIFCR=DMA_HIFCR_CFEIF6|DMA_HIFCR_CDMEIF6|DMA_HIFCR_CTEIF6|DMA_HIFCR_CHTIF6|DMA_HIFCR_CTCIF6;
        DMA2_Stream6->PAR=(uint32_t)&SDIO->FIFO;
        DMA2_Stream6->M0AR=(uint32_t)data;
        DMA2_Stream6->NDTR=128;
        // Word transfers, incrementing SRAM, normal mode, full FIFO, INCR4.
        DMA2_Stream6->CR=(4u<<25)|DMA_SxCR_DIR_0|DMA_SxCR_MINC|DMA_SxCR_PSIZE_1|
                         DMA_SxCR_MSIZE_1|DMA_SxCR_PL_0|DMA_SxCR_MBURST_0|DMA_SxCR_PBURST_0;
        DMA2_Stream6->FCR=DMA_SxFCR_DMDIS|DMA_SxFCR_FTH;
        __DMB(); DMA2_Stream6->CR|=DMA_SxCR_EN;
        SDIO->DTIMER=24000000; SDIO->DLEN=512;
        SDIO->DCTRL=(9u<<4)|SDIO_DCTRL_DMAEN|SDIO_DCTRL_DTEN;
        return true;
    }
    Reply transferred() override {
        if((SDIO->STA&(SDIO_STA_DCRCFAIL|SDIO_STA_DTIMEOUT|SDIO_STA_TXUNDERR|SDIO_STA_STBITERR))||
           (DMA2->HISR&(DMA_HISR_FEIF6|DMA_HISR_DMEIF6|DMA_HISR_TEIF6))) return Reply::error;
        return (SDIO->STA&SDIO_STA_DATAEND)&&(DMA2->HISR&DMA_HISR_TCIF6)&&
               !(DMA2_Stream6->CR&DMA_SxCR_EN)?Reply::ok:Reply::pending;
    }
    void stop() override { SDIO->DCTRL=0; DMA2_Stream6->CR&=~DMA_SxCR_EN; }
} io;
AsyncCard card;
Extent owned;
bool initialized=false;
// Generate a new session nonce so unwritten preallocated sectors cannot be
// mistaken for valid data left over from an earlier file on the same card.
bool nonce(uint64_t& value) {
    __HAL_RCC_RNG_CLK_ENABLE(); RNG->CR=RNG_CR_RNGEN;
    value=0;
    for(unsigned n=0;n<2;++n) {
        const uint32_t start=micros();
        while(!(RNG->SR&RNG_SR_DRDY)) {
            if((RNG->SR&(RNG_SR_CECS|RNG_SR_SECS))||uint32_t(micros()-start)>10000) return false;
        }
        if(RNG->SR&(RNG_SR_CECS|RNG_SR_SECS)) return false;
        value=(value<<32)|RNG->DR;
    }
    RNG->CR=0; return value!=0;
}
}
bool prepare(Extent& e,char (&name)[16]) {
    if(initialized) return false; // no restart/mount while DMA may own a buffer
    initialized=true;
    SD.setDx(PC8,PC9,PC10,PC11); SD.setCK(PC12); SD.setCMD(PD2);
    if(!nonce(e.session)||!SD.begin()) return false;
    FIL file{}; FRESULT result=FR_EXIST;
    for(unsigned i=0;i<10000&&result==FR_EXIST;++i) {
        std::snprintf(name,sizeof(name),"FLT%05u.BIN",i);
        result=f_open(&file,name,FA_WRITE|FA_CREATE_NEW);
    }
    if(result!=FR_OK) return false;
    auto* fs=file.obj.fs;
    // Pin the raw mapping to FAT32/512-byte sectors on the sole SD disk.
    constexpr uint32_t size=128u*1024u*1024u;
    if(fs->fs_type!=FS_FAT32||fs->pdrv!=0||f_expand(&file,size,1)!=FR_OK) { f_close(&file); return false; }
    const uint64_t first=uint64_t(fs->database)+uint64_t(fs->csize)*(file.obj.sclust-2);
    e.first_sector=uint32_t(first); e.sectors=size/512;
    const auto* h=BSP_SD_Handle();
    if(first>UINT32_MAX||first+e.sectors>h->SdCard.LogBlockNbr) { f_close(&file); return false; }
    // close commits allocation and file size. No filesystem functions follow.
    if(f_close(&file)!=FR_OK) return false;
    __HAL_RCC_DMA2_CLK_ENABLE();
    if(DMA2_Stream6->CR&DMA_SxCR_EN) return false;
    SDIO->MASK=0;
    card.configure(io,uint16_t(h->SdCard.RelCardAdd),h->SdCard.CardType==CARD_SDHC_SDXC);
    owned=e; return true;
}
bool start_sector(uint32_t lba,const uint8_t* data) {
    if(lba<owned.first_sector||uint64_t(lba)>=uint64_t(owned.first_sector)+owned.sectors) return false;
    return card.start(lba,data,micros());
}
Progress poll_sector() { return card.poll(micros()); }
uint32_t now_ms() { return millis(); }
}
