#pragma once
#include <cmath>
namespace estimation {
// Optional notch: disabled until a measured vibration peak is configured.
class Notch {
public:
    bool configure(float sample_hz,float center_hz,float q) {
        _enabled=false;
        if(center_hz==0) return true;
        if(!std::isfinite(sample_hz)||!std::isfinite(center_hz)||!std::isfinite(q)||
           sample_hz<=0||center_hz<=0||center_hz>=.45f*sample_hz||q<.5f||q>20) return false;
        const float w=6.28318530718f*center_hz/sample_hz;
        const float alpha=std::sin(w)/(2*q),a0=1+alpha;
        _b0=1/a0; _b1=-2*std::cos(w)/a0; _b2=_b0;
        _a1=_b1; _a2=(1-alpha)/a0; _primed=false; _enabled=true; return true;
    }
    float apply(float x) {
        if(!_enabled) return x;
        if(!_primed) { _x1=_x2=_y1=_y2=x; _primed=true; }
        const float y=_b0*x+_b1*_x1+_b2*_x2-_a1*_y1-_a2*_y2;
        _x2=_x1; _x1=x; _y2=_y1; _y1=y; return y;
    }
private:
    bool _enabled=false,_primed=false;
    float _b0=1,_b1=0,_b2=0,_a1=0,_a2=0,_x1=0,_x2=0,_y1=0,_y2=0;
};
}
