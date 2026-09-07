#pragma once
//
// mode_manual.hpp -- MANUAL: direct passthrough, no stabilisation.
//
// This path must be as short and independent as possible and must work even
// if the IMU driver has faulted. It is where the aircraft falls back when
// anything goes wrong. Do not add sensor dependencies here.
//
#include "mode.hpp"

namespace modes {

class ModeManual : public Mode {
public:
    Id          id()   const override { return Id::manual; }
    const char* name() const override { return "MANUAL"; }
    void enter(const control::Outputs&) override {}   // nothing to preload

    void update(const ModeInput& in, control::Outputs& out) override;

    control::MixParams params;   // tunable mix gains
};

} // namespace modes
