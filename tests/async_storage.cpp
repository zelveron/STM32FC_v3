#include "../src/core/sd_bin_log.hpp"
#include "../src/core/sd_storage.hpp"
#include "../src/core/log_sector.hpp"
#include "../src/hal/stm32/sd_transfer_status.hpp"
#include <cstdio>
#include <cstring>
#include <vector>
namespace fake {
uint32_t now=0,lba=0;
bool fail=false;
storage::Progress progress=storage::Progress::idle;
const uint8_t* buffer=nullptr;
std::vector<core::LogSector> written;
}
namespace storage {
bool prepare(Extent& e,char (&name)[16]) { e={1000,100,0xabcdef1234567890ull}; std::strcpy(name,"FLT00000.BIN"); return true; }
bool start_sector(uint32_t lba,const uint8_t* data) {
    fake::lba=lba; fake::buffer=data; fake::progress=Progress::busy; return true;
}
Progress poll_sector() { return fake::fail?Progress::error:fake::progress; }
uint32_t now_ms() { return fake::now; }
}
int checks=0,fails=0;
void check(const char* n,bool ok) { ++checks; fails+=!ok; std::printf("[%s] %s\n",ok?"PASS":"FAIL",n); }
void complete() {
    fake::written.push_back(*reinterpret_cast<const core::LogSector*>(fake::buffer));
    fake::progress=storage::Progress::complete; sd_bin_log::flush_step();
}
struct Card:storage::CardIo {
    uint8_t cmd=0; uint32_t arg=0,response=0; bool stopped=false,tx=false;
    storage::Reply rep=storage::Reply::pending,data=storage::Reply::pending;
    void command(uint8_t c,uint32_t a) override { cmd=c; arg=a; rep=storage::Reply::pending; }
    storage::Reply reply(uint8_t,uint32_t& r) override { r=response; return rep; }
    bool transmit(const uint8_t*) override { tx=true; return true; }
    storage::Reply transferred() override { return data; }
    void stop() override { stopped=true; }
};
int main() {
    core::LogRing ring;
    check("boot preallocation opens asynchronous logger",sd_bin_log::begin(ring)==sd_bin_log::Result::ok);
    check("subsector budget cannot initiate IO",sd_bin_log::flush_step(511)==0&&fake::buffer==nullptr);
    sd_bin_log::flush_step();
    check("header write starts without waiting or reporting durable bytes",fake::lba==1000&&sd_bin_log::bytes_written()==0);
    for(int i=0;i<10000;++i) sd_bin_log::flush_step();
    check("pending card returns promptly without consuming or reusing DMA buffer",sd_bin_log::bytes_written()==0&&fake::lba==1000);
    complete();
    check("committed header has valid CRC session and sequence",core::valid(fake::written[0])&&fake::written[0].sequence==0&&fake::written[0].session==0xabcdef1234567890ull);
    std::vector<uint8_t> payload(700); for(unsigned i=0;i<payload.size();++i) payload[i]=uint8_t(i);
    ring.push(payload.data(),payload.size()); sd_bin_log::flush_step();
    check("ring bytes retained until card programming completes",ring.used()==700&&fake::lba==1001);
    complete();check("acknowledged sector consumes exactly its payload",ring.used()==208);
    sd_bin_log::sync(); sd_bin_log::flush_step(); complete();
    check("asynchronous tail request preserves last partial sector",ring.used()==0&&fake::written[2].used==208);
    std::vector<uint8_t> recovered;
    for(unsigned i=1;i<fake::written.size();++i) {
        auto& s=fake::written[i]; recovered.insert(recovered.end(),s.payload,s.payload+s.used);
    }
    check("sectors reconstruct original stream byte for byte",recovered==payload);
    ring.push(payload.data(),payload.size()); sd_bin_log::flush_step(); fake::fail=true; sd_bin_log::flush_step();
    check("card failure latches logger off and preserves pending data",!sd_bin_log::ok()&&ring.used()==700);
    check("in-flight logger cannot remount/reuse pending DMA storage",sd_bin_log::begin(ring)==sd_bin_log::Result::open_failed);
    uint8_t sector[512]{}; Card io; storage::AsyncCard card; card.configure(io,0x1234,true);
    check("CMD24 uses sector addressing",card.start(987,sector,0)&&io.cmd==24&&io.arg==987);
    check("command response is polled without waiting",card.poll(1)==storage::Progress::busy&&!io.tx);
    io.rep=storage::Reply::ok; card.poll(2); check("DMA begins only after accepted write command",io.tx);
    io.data=storage::Reply::ok; card.poll(3);
    check("DMA completion requests card status, not durability",io.cmd==13&&io.arg==0x12340000&&card.poll(4)==storage::Progress::busy);
    io.response=7<<9; io.rep=storage::Reply::ok; card.poll(5); card.poll(1005);
    check("busy card is rechecked in later calls",io.cmd==13&&card.poll(1006)==storage::Progress::busy);
    io.response=(4<<9)|0x100; io.rep=storage::Reply::ok;
    check("only READY_FOR_DATA plus TRANSFER completes",card.poll(1007)==storage::Progress::complete);
    Card timeout; storage::AsyncCard slow; slow.configure(timeout,1,true); slow.start(1,sector,0xfffff000);
    check("command timeout survives timer wrap and aborts without blocking",slow.poll(0x400)==storage::Progress::error&&timeout.stopped);
    Card removed; storage::AsyncCard bad; bad.configure(removed,1,true); bad.start(1,sector,0); removed.rep=storage::Reply::error;
    check("removed card latches error and rejects restart",bad.poll(1)==storage::Progress::error&&!bad.start(2,sector,2));
    Card denied; storage::AsyncCard ro; ro.configure(denied,1,true); ro.start(1,sector,0); denied.rep=storage::Reply::ok; denied.response=1u<<26;
    check("R1 write-protection error cannot start DMA",ro.poll(1)==storage::Progress::error&&!denied.tx);
    Card forever; storage::AsyncCard stall; stall.configure(forever,1,true); stall.start(1,sector,0); forever.rep=storage::Reply::ok; stall.poll(1);
    check("data timeout terminates stuck transfer",stall.poll(1000000)==storage::Progress::error&&forever.stopped);
    auto corrupt=fake::written[1]; corrupt.payload[2]^=1;
    check("sector CRC detects power-cut/corruption",!core::valid(corrupt));
    using storage::Reply;
    using storage::f407::write_status;
    // Replay the real 2026-10-09 first-sector failure snapshot: SDIO is
    // still transmitting, DMA reports FEIF6 only. It must keep polling.
    check("recorded SD FIFO warning does not abort an active transfer",
          write_status(1052736,65536,1)==Reply::pending);
    check("FIFO warning alone never acknowledges a sector",
          write_status(0,0x10000,0)==Reply::pending);
    check("completed DMA plus SD DATAEND accepts an isolated FIFO warning",
          write_status(0x100,0x210000,0)==Reply::ok);
    check("DATAEND alone cannot complete a transfer",write_status(0x100,0,0)==Reply::pending);
    check("DMA complete alone cannot complete a transfer",write_status(0,0x200000,0)==Reply::pending);
    check("active DMA retains buffer ownership even with completion flags",
          write_status(0x100,0x210000,1)==Reply::pending);
    for(uint32_t flag:{2u,8u,16u,512u})
        check("SD data errors override completion and FIFO warning",
              write_status(0x100|flag,0x210000,0)==Reply::error);
    for(uint32_t flag:{0x40000u,0x80000u})
        check("DMA bus/direct-mode errors override completion and FIFO warning",
              write_status(0x100,0x210000|flag,0)==Reply::error);
    Card warned; storage::AsyncCard warning_card; warning_card.configure(warned,1,true);
    warning_card.start(1,sector,0); warned.rep=Reply::ok; warning_card.poll(1);
    warned.data=write_status(1052736,65536,1);
    check("persistent FIFO warning still hits bounded data timeout",
          warning_card.poll(999999)==storage::Progress::busy&&
          warning_card.poll(1000000)==storage::Progress::error&&warned.stopped);
    std::printf("%d checks, %d failures\n",checks,fails); return fails?1:0;
}
