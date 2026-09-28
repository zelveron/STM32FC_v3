#pragma once
#include <cstdint>
#include <cstring>
namespace core {
enum class TelemKind : uint8_t { attitude, vario, gps, mode, none };
struct TelemConfig {
    bool enabled[4]={true,true,true,true};
    uint32_t interval_ms[4]={100,100,400,400};
    uint16_t bytes_per_second=240; // CRSF bytes, NOT UART baud or RF payload bits
};
class TelemetrySchedule {
public:
    void configure(const TelemConfig& c) { _cfg=c; }
    TelemKind choose(uint32_t now,bool attitude,bool vario,bool gps,uint32_t fix_sequence,
                     const char* mode,bool link) {
        if(!_started) { _last=now; _tokens=32; _started=true; }
        const uint32_t elapsed=now-_last; _last=now;
        _tokens+=float(elapsed)*_cfg.bytes_per_second*.001f;
        if(_tokens>32) _tokens=32; // never accumulate a reconnect burst
        if(!link) return TelemKind::none;
        const bool valid[4]={attitude,vario,gps,true};
        const bool change=std::strncmp(mode,_mode,15)!=0;
        // Mode/fault changes preempt normal periodic samples, at most 10 Hz.
        if(change&&_cfg.enabled[3]&&(!_sent[3]||uint32_t(now-_when[3])>=100)) {
            if(_tokens>=cost(3,mode)) return TelemKind::mode;
            ++deferred; return TelemKind::none;
        }
        for(unsigned n=0;n<4;++n) {
            const unsigned i=(_next+n)%4;
            if(!_cfg.enabled[i]||!valid[i]||(_sent[i]&&uint32_t(now-_when[i])<_cfg.interval_ms[i])) continue;
            if(i==2&&_sent[i]&&fix_sequence==_fix) continue;
            if(_tokens>=cost(i,mode)) return TelemKind(i);
            ++deferred;
            // Reserve the next eligible type's turn until it fits. Otherwise
            // small vario frames can permanently starve a 19-byte GPS frame.
            return TelemKind::none;
        }
        return TelemKind::none;
    }
    void result(TelemKind kind,bool ok,uint32_t now,uint32_t fix_sequence,const char* mode) {
        if(kind==TelemKind::none) return;
        const unsigned i=unsigned(kind);
        if(!ok) { ++queue_failures; _next=(i+1)%4; return; }
        _tokens-=cost(i,mode); _when[i]=now; _sent[i]=true; ++sent[i]; _next=(i+1)%4;
        if(i==2) _fix=fix_sequence;
        if(i==3) { std::strncpy(_mode,mode,15); _mode[15]=0; }
    }
    uint32_t sent[4]{},deferred=0,queue_failures=0;
private:
    static unsigned cost(unsigned i,const char* mode) {
        if(i!=3) return i==0?10:i==1?6:19;
        unsigned len=0; while(mode[len]&&len<15) ++len; return len+5;
    }
    TelemConfig _cfg;
    uint32_t _last=0,_when[4]{},_fix=0;
    bool _started=false,_sent[4]{};
    unsigned _next=0;
    float _tokens=0;
    char _mode[16]{};
};
}
