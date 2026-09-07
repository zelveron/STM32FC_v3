#pragma once
//
// aircraft.hpp -- simplified 6DOF fixed-wing model for SITL.
//
// Rigid body + linear aero (lift/drag/side + roll/pitch/yaw moments with
// control terms and rate damping). Coefficients are ROUGH order-of-magnitude
// values for a ~1.5 m foam/PLA twin-EDF, NOT validated against the real
// airframe. This is for exercising control logic (does aileron -> roll ->
// bank -> turn behave right, does the AHRS track), not performance prediction.
//
// Frames: body FRD (x fwd, y right, z down); world NED. Attitude as a unit
// quaternion (w,x,y,z) body->world.
//
// Portable (no hal, no Arduino). C++17, <cmath>.
//
namespace sitl {

struct Vec3 { double x = 0, y = 0, z = 0; };
struct Quat { double w = 1, x = 0, y = 0, z = 0; };

struct Controls {
    double aileron  = 0.0;   // [-1,+1], + = roll right
    double elevator = 0.0;   // [-1,+1], + = pitch up
    double rudder   = 0.0;   // [-1,+1], + = yaw right
    double throttle = 0.0;   // [0,1]
};

struct State {
    Vec3 pos_ned;            // m
    Vec3 vel_ned;            // m/s
    Quat q;                  // body -> world
    Vec3 omega_body;         // rad/s (p,q,r)
    Vec3 accel_body;         // m/s^2 specific force (what an accelerometer reads)
};

class Aircraft {
public:
    // Trimmed level flight, heading north, at `airspeed_mps` and `alt_m`.
    // Solves for the trim angle of attack, elevator and throttle.
    void reset(double airspeed_mps = 18.0, double alt_m = 100.0);

    void step(const Controls& u, double dt_s);

    const State& state() const { return _s; }

    // Truth Euler angles (rad), for CSV / comparison with the AHRS.
    void euler(double& roll, double& pitch, double& yaw) const;
    double airspeed_mps() const;

    // Trim the pilot / autopilot commands sit on top of.
    double trim_elevator() const { return _trim_de; }   // stick-equivalent [-1,+1]
    double trim_throttle() const { return _trim_thr; }  // [0,1]

private:
    State  _s;
    double _trim_de  = 0.0;
    double _trim_thr = 0.5;
    void   derivatives(const State& s, const Controls& u, State& d) const;
};

} // namespace sitl
