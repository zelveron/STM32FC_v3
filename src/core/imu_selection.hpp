#pragma once
#include <cstdint>
namespace core {
class ImuSelection {
public:
    void update(bool primary,bool secondary,bool disagree,uint32_t now_ms) {
        if(primary && secondary && disagree) {
            if(!_timing) { _timing=true; _since=now_ms; }
            else if(uint32_t(now_ms-_since)>=100) _ambiguous=true;
        } else _timing=false;
        if(_active==0 && !primary && secondary) _active=1;
        // No automatic return to a previously failed primary in this flight.
        _healthy=!_ambiguous && (_active==0?primary:secondary);
    }
    unsigned active() const { return _active; }
    bool healthy() const { return _healthy; }
    bool ambiguous() const { return _ambiguous; }
private:
    unsigned _active=0;
    bool _timing=false,_ambiguous=false,_healthy=false;
    uint32_t _since=0;
};
}
