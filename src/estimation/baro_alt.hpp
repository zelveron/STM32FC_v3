#pragma once
//
// baro_alt.hpp -- barometric altitude above a ground reference.
//
// set_ground() latches the ground pressure (call it while disarmed; the last
// pre-arm value wins). alt_m() then returns height above that. Uses the
// standard-atmosphere hypsometric constant referenced to the ground pressure
// -- the few-% scale error over the first few hundred metres AGL is
// acceptable for altitude hold; a temperature-referenced form comes later.
//
// Portable.
//
namespace estimation {

class BaroAlt {
public:
    void  set_ground(float press_pa);
    bool  referenced() const { return _p0 > 0.0f; }
    float alt_m(float press_pa) const;   // 0 until referenced

private:
    float _p0 = 0.0f;   // ground pressure, Pa
};

} // namespace estimation
