#pragma once
#include "mixer.hpp"
#include "command_shape.hpp"
namespace control {
class SurfaceTransition {
public:
    void configure(float rate,float seconds) { _rate=bound(rate,1,50); _duration=bound(seconds,.02f,2); }
    void enter(const Outputs& out) { _entry=_last=out; _elapsed=0; }
    void apply(Outputs& out,float dt,bool roll_only=false) {
        const float h=std::isfinite(dt)&&dt>0&&dt<=.02f?dt:0;
        const float x=bound(_elapsed/_duration,0,1);
        const float blend=x*x*(3-2*x);
        for(int i=0;i<6;++i) {
            if(!is_surface_output(i)) continue; // never blend or slew throttle
            if(roll_only && !is_roll_output(i)) continue;
            const float target=_entry.ch[i]+blend*(out.ch[i]-_entry.ch[i]);
            out.ch[i]=_last.ch[i]+bound(target-_last.ch[i],-_rate*h,_rate*h);
        }
        _last=out; _elapsed+=h;
    }
private:
    Outputs _entry{},_last{};
    float _elapsed=0,_rate=12,_duration=.25f;
};
}
