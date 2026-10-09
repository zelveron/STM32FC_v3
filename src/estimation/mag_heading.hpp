#pragma once
#include <cstdint>

namespace estimation {
struct MagHeadingConfig {
    bool enabled=true;
    // Factory compensation is not installed-aircraft compass calibration.
    bool orientation_confirmed=false, calibrated=false;
    int8_t rotation[3]={1,2,3}; // sensor -> body FRD; unconfirmed placeholder
    float offset_ut[3]={0,0,0}; // hard iron, in compensated sensor axes
    float correction[3][3]={{1,0,0},{0,1,0},{0,0,1}}; // symmetric soft iron
    float field_ut=50; // calibration normalization; not a location estimate
    float field_tolerance=.30f, min_horizontal_ut=8;
    float tau_s=2, max_correction_dps=5, max_innovation_deg=45;
};

// Portable magnetic observation validator. Gyro propagation stays in AHRS;
// returned corrections rotate world yaw only, leaving roll/pitch unchanged.
class MagHeading {
public:
    enum class State { disabled, setup, waiting, no_attitude, field, innovation,
                       qualifying, aligning, tracking, stale, driver };
    explicit MagHeading(const MagHeadingConfig& config={}) : _config(config) {}
    void configure(const MagHeadingConfig& config) { _config=config; reset(); }
    void reset();
    void invalidate_attitude();
    void driver_failed();
    float observe(float x,float y,float z,float roll,float pitch,float yaw,
                  bool attitude_valid,bool allow_initial_alignment,uint32_t now_ms);
    bool aiding(uint32_t now_ms,bool attitude_valid,bool driver_healthy) const;
    bool heading_valid(uint32_t now_ms,bool attitude_valid,bool driver_healthy) const;
    State state(uint32_t now_ms,bool attitude_valid,bool driver_healthy) const;
    static const char* state_name(State state);
    bool configured() const;
    float measured_rad() const { return _measured; }
    float field_ut() const { return _field; }
    float innovation_rad() const { return _innovation; }
private:
    float reject(State state);
    MagHeadingConfig _config;
    State _state=State::waiting;
    bool _candidate=false,_accepted=false,_aligned=false,_tracking=false;
    uint32_t _first_ms=0,_last_ms=0;
    unsigned _count=0;
    float _previous_innovation=0,_measured=0,_field=0,_innovation=0;
};
}
