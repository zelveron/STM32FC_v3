#pragma once
//
// mixer.hpp -- pilot commands -> the 8 output channels.
//
// Output channel map (CLAUDE.md):
//   0 aileron     1 elevator   2 throttle   3 rudder
//   4 neutral     5 aileron    6 ESC idle   7 ESC idle
//
// Surfaces are normalized [-1,+1]; throttle is [0,1]. SERVO6 reversal is
// applied once by SrvChannel, after stabilization and slew limiting. Physical
// SERVO5 remains centered; legacy ESC headers SERVO8/9 remain at idle.
//
// Sign conventions (CLAUDE.md): +roll cmd -> roll right, +pitch -> nose up,
// +yaw -> yaw right, +aileron out -> roll right, +elevator out -> pitch up,
// +rudder out -> yaw right.
//
// Portable.
//
namespace control {

// Physical surface outputs requested for the current aircraft (zero-based).
constexpr unsigned kRollOutput = 0, kPitchOutput = 1, kThrottleOutput = 2, kYawOutput = 3;
constexpr unsigned kAxisOutputs[3] = {kRollOutput, kPitchOutput, kYawOutput};
constexpr bool is_roll_output(unsigned channel) { return channel == 0 || channel == 5; }
constexpr bool is_surface_output(unsigned channel) { return is_roll_output(channel) || channel == 1 || channel == 3; }
// Include the reserved ESC headers in all idle/arming/reversal protections.
constexpr bool is_motor_output(unsigned channel) { return channel == 2 || channel == 6 || channel == 7; }

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
};

// MANUAL: direct stick -> surface passthrough, no stabilisation.
void mix_manual(const Sticks& in, const MixParams& p, Outputs& out);

} // namespace control
