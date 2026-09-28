// Compile the vendored disk I/O source unchanged with a minimal fake BSP.
// Stop the test by throwing after 100,000 state polls, not by waiting forever.
#include <cstdint>
#include <cstdio>
#define __SD_DISKIO_H
using BYTE=uint8_t;using WORD=uint16_t;using DWORD=uint32_t;
using LBA_t=uint32_t;using UINT=unsigned;using DSTATUS=uint8_t;
enum DRESULT { RES_OK,RES_ERROR,RES_WRPRT,RES_NOTRDY,RES_PARERR };
constexpr int STA_NOINIT=1,MSD_OK=0,CTRL_SYNC=0,GET_SECTOR_COUNT=1,GET_SECTOR_SIZE=2,GET_BLOCK_SIZE=3;
#define SD_DATATIMEOUT 2000U
struct BSP_SD_CardInfo { uint32_t LogBlockNbr=1000,LogBlockSize=512; };
struct Diskio_drvTypeDef {
    DSTATUS(*initialize)(BYTE);DSTATUS(*status)(BYTE);
    DRESULT(*read)(BYTE,BYTE*,LBA_t,UINT);
    DRESULT(*write)(BYTE,const BYTE*,LBA_t,UINT);
    DRESULT(*ioctl)(BYTE,BYTE,void*);
};
static int polls=0;static uint32_t elapsed_ms=0,passed_timeout=0;
int BSP_SD_Init(){return MSD_OK;}
int BSP_SD_ReadBlocks(uint32_t*,uint32_t,UINT,uint32_t timeout){passed_timeout=timeout;return MSD_OK;}
int BSP_SD_WriteBlocks(uint32_t*,uint32_t,UINT,uint32_t timeout){passed_timeout=timeout;return MSD_OK;}
int BSP_SD_GetCardState(){++elapsed_ms;if(++polls==100000)throw 1;return 1;}
void BSP_SD_GetCardInfo(BSP_SD_CardInfo*){}
uint32_t HAL_GetTick(){return elapsed_ms;}
#include "../lib/FatFs/src/drivers/sd_diskio.c"

int main(){
    uint8_t buf[512]{};int fails=0;
    const auto w=SD_write(0,buf,0,1);
    const bool wok=w==RES_ERROR&&elapsed_ms>=2000&&elapsed_ms<=2001;
    std::printf("[%s] SD write busy wait terminates at %u ms\n",wok?"PASS":"FAIL",elapsed_ms);fails+=!wok;
    polls=0;elapsed_ms=0;
    const auto r=SD_read(0,buf,0,1);
    const bool rok=r==RES_ERROR&&elapsed_ms>=2000&&elapsed_ms<=2001;
    std::printf("[%s] SD read busy wait terminates at %u ms\n",rok?"PASS":"FAIL",elapsed_ms);fails+=!rok;
    return fails?1:0;
}
