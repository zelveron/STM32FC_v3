#pragma once
#include <cstdint>
namespace storage {
enum class Progress { idle, busy, complete, error };
enum class Reply { pending, ok, error };
// Small non-blocking card protocol, independent of STM32 register access.
// No HAL command helper is used: those helpers contain polling wait loops.
struct CardIo {
    virtual ~CardIo()=default;
    virtual void command(uint8_t index,uint32_t arg)=0;
    virtual Reply reply(uint8_t index,uint32_t& response)=0;
    virtual bool transmit(const uint8_t* sector)=0;
    virtual Reply transferred()=0;
    virtual void stop()=0;
};
class AsyncCard {
public:
    void configure(CardIo& io,uint16_t rca,bool block_addressed) { _io=&io; _rca=rca; _block=block_addressed; _state=State::idle; }
    bool start(uint32_t lba,const uint8_t* sector,uint32_t now) {
        if(!_io||(_state!=State::idle&&_state!=State::done)||!sector) return false;
        if(!_block&&lba>0x7fffff) return false;
        _data=sector; _since=_cmd_since=now; _state=State::write;
        _io->command(24,_block?lba:lba*512); return true;
    }
    Progress poll(uint32_t now) {
        if(_state==State::idle) return Progress::idle;
        if(_state==State::done) return Progress::complete;
        if(_state==State::failed) return Progress::error;
        if(uint32_t(now-_since)>=1000000) return fail();
        uint32_t response=0;
        if(_state==State::write||_state==State::status) {
            const bool write=_state==State::write;
            const Reply reply=_io->reply(write?24:13,response);
            if(reply==Reply::error || (reply==Reply::pending&&uint32_t(now-_cmd_since)>=5000)) return fail();
            if(reply==Reply::pending) return Progress::busy;
            if(response&0xfdffe008u) return fail(); // SD R1 error flags
            if(write) {
                if(!_io->transmit(_data)) return fail();
                _state=State::data;
            } else if((response&0x100)&&((response>>9)&15)==4) { _state=State::done; return Progress::complete; }
            else { _state=State::delay; _cmd_since=now; }
        } else if(_state==State::data) {
            const Reply r=_io->transferred();
            if(r==Reply::error) return fail();
            if(r==Reply::ok) { _io->stop(); status(now); }
        } else if(_state==State::delay&&uint32_t(now-_cmd_since)>=1000) status(now);
        return Progress::busy;
    }
private:
    enum class State { idle,write,data,status,delay,done,failed };
    void status(uint32_t now) { _cmd_since=now; _state=State::status; _io->command(13,uint32_t(_rca)<<16); }
    Progress fail() { _io->stop(); _state=State::failed; return Progress::error; }
    CardIo* _io=nullptr;
    State _state=State::idle;
    const uint8_t* _data=nullptr;
    uint16_t _rca=0;
    bool _block=true;
    uint32_t _since=0,_cmd_since=0;
};
}
