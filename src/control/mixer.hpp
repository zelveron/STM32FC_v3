#pragma once
//
// mixer.hpp -- pilot commands -> the 8 output channels.
//
// Output channel map (CLAUDE.md):
//   0 aileron L   1 aileron R   2 elevator L   3 elevator R
//   4 rudder      5 nosewheel   6 ESC L        7 ESC R
//
// Surfaces are normalized [-1,+1]; ESC outputs are [0,1]. Left/right surface
// opposition is handled by each SrvChannel's `reversed` flag, so both ailerons
// (and both elevators) receive the SAME signed command here.
//
// Sign conventions (CLAUDE.md): +roll cmd -> roll right, +pitch -> nose up,
// +yaw -> yaw right, +aileron out -> roll right, +elevator out -> pitch up,
// +rudder out -> yaw right.
//
// Portable.
//
namespace control {

struct Sticks {
    float roll     = 0.0f;   // [-1,+1]
    float pitch    = 0.0f;   // [-1,+1]
    float yaw      = 0.0f;   // [-1,+1]
    float throttle = 0.0f;   // [0,1]
};

struct Outputs {
    float ch[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
};

struct MixParams {
    float aileron_gain             = 1.0f;
    float elevator_gain            = 1.0f;
    float rudder_gain              = 1.0f;
    float nosewheel_gain           = 1.0f;   // ground-speed falloff is added later
    float diff_thrust_gain         = 0.0f;   // yaw -> L/R throttle split; 0 = off
    float diff_thrust_min_throttle = 0.15f;  // no differential below this
};

// MANUAL: direct stick -> surface passthrough, no stabilisation.
void mix_manual(const Sticks& in, const MixParams& p, Outputs& out);

} // namespace control
