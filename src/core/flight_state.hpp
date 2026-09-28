#pragma once
#include <cmath>
#include <cstdint>
namespace core {
// Integration eligibility heuristic, not an airspeed/stall detector. A glide
// preserves eligibility. Explicit disarm clears it; RC failsafe preserves it.
class FlightState {
public:
    bool update(bool armed,bool failsafe,float throttle,bool gps_valid,float speed,
                bool baro_valid,float climb,float rate_dps,uint32_t now) {
        if(!armed&&!failsafe) { _active=false; _launch=_landing=false; return false; }
        const bool launch=armed && (throttle>.25f || (gps_valid&&speed>8) ||
                                                   (baro_valid&&std::fabs(climb)>1.5f));
        if(!_active) {
            if(!launch) _launch=false;
            else if(!_launch) { _launch=true; _since=now; }
            else if(uint32_t(now-_since)>=500) { _active=true; _launch=false; }
        }
        const bool ground=!failsafe&&throttle<.1f&&gps_valid&&speed<2&&baro_valid&&
                          std::fabs(climb)<.3f&&rate_dps<3;
        if(_active) {
            if(!ground) _landing=false;
            else if(!_landing) { _landing=true; _land_since=now; }
            else if(uint32_t(now-_land_since)>=3000) { _active=false; _landing=false; }
        }
        return _active;
    }
private:
    bool _active=false,_launch=false,_landing=false;
    uint32_t _since=0,_land_since=0;
};
}
