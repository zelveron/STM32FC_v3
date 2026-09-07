#include "aircraft.hpp"
#include <cmath>

namespace sitl {
namespace {

// --- environment / airframe (rough; see header) ---
constexpr double kG     = 9.80665;
constexpr double kRho   = 1.225;    // kg/m^3

constexpr double kMass  = 3.2;      // kg
constexpr double kIxx   = 0.24, kIyy = 0.32, kIzz = 0.50;   // kg m^2
constexpr double kS     = 0.35;    // wing area m^2
constexpr double kB     = 1.5;     // span m
constexpr double kC     = 0.24;    // mean chord m
constexpr double kTmax  = 45.0;    // N total static thrust (2 EDF)

// aero coefficients
constexpr double kCL0   = 0.25,  kCLa = 5.2;
constexpr double kCD0   = 0.045, kCDk = 0.06;
constexpr double kCYb   = -0.35;
constexpr double kCl_b  = -0.08, kCl_p = -0.45, kCl_da = 0.18;
constexpr double kCm0   = 0.04,  kCm_a = -0.7, kCm_q = -12.0, kCm_de = -1.1;
constexpr double kCn_b  = 0.07,  kCn_r = -0.10, kCn_dr = -0.08;
constexpr double kDefl  = 0.35;   // rad of surface travel at full stick

Quat qmul(const Quat& a, const Quat& b)
{
    return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
             a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
             a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
             a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
Quat qnorm(Quat q)
{
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12) return { 1,0,0,0 };
    return { q.w/n, q.x/n, q.y/n, q.z/n };
}
// rotate world-frame v into body frame (v_body = R(q)^T v_world)
Vec3 world_to_body(const Quat& q, const Vec3& v)
{
    const double w=q.w,x=q.x,y=q.y,z=q.z;
    const double r00=1-2*(y*y+z*z), r01=2*(x*y+w*z),   r02=2*(x*z-w*y);
    const double r10=2*(x*y-w*z),   r11=1-2*(x*x+z*z), r12=2*(y*z+w*x);
    const double r20=2*(x*z+w*y),   r21=2*(y*z-w*x),   r22=1-2*(x*x+y*y);
    return { r00*v.x + r01*v.y + r02*v.z,
             r10*v.x + r11*v.y + r12*v.z,
             r20*v.x + r21*v.y + r22*v.z };
}
Vec3 body_to_world(const Quat& q, const Vec3& v)
{
    const double w=q.w,x=q.x,y=q.y,z=q.z;
    const double r00=1-2*(y*y+z*z), r01=2*(x*y+w*z),   r02=2*(x*z-w*y);
    const double r10=2*(x*y-w*z),   r11=1-2*(x*x+z*z), r12=2*(y*z+w*x);
    const double r20=2*(x*z+w*y),   r21=2*(y*z-w*x),   r22=1-2*(x*x+y*y);
    return { r00*v.x + r10*v.y + r20*v.z,
             r01*v.x + r11*v.y + r21*v.z,
             r02*v.x + r12*v.y + r22*v.z };
}

} // namespace

void Aircraft::reset(double airspeed_mps, double alt_m)
{
    const double V    = airspeed_mps;
    const double qbar  = 0.5 * kRho * V * V;
    const double W     = kMass * kG;

    // --- solve trim (wings level, no sideslip) ---
    // lift ~ weight -> CL_trim -> alpha_trim; pitch moment 0 -> de_trim; T = D.
    const double CL_trim = W / (qbar * kS);
    const double alpha   = (CL_trim - kCL0) / kCLa;
    const double de_rad  = -(kCm0 + kCm_a * alpha) / kCm_de;
    const double CD       = kCD0 + kCDk * CL_trim * CL_trim;
    const double D        = qbar * kS * CD;

    _trim_de  = de_rad / kDefl;
    _trim_thr = D / (kTmax > 0 ? kTmax : 1.0);
    if (_trim_thr < 0) _trim_thr = 0; else if (_trim_thr > 1) _trim_thr = 1;

    _s = State{};
    _s.pos_ned    = { 0, 0, -alt_m };
    _s.vel_ned    = { V, 0, 0 };                 // horizontal, north
    _s.q          = { std::cos(alpha / 2), 0, std::sin(alpha / 2), 0 };  // pitched up alpha
    _s.omega_body = { 0, 0, 0 };
    _s.accel_body = { 0, 0, -kG };
}

double Aircraft::airspeed_mps() const
{
    return std::sqrt(_s.vel_ned.x*_s.vel_ned.x +
                     _s.vel_ned.y*_s.vel_ned.y +
                     _s.vel_ned.z*_s.vel_ned.z);
}

void Aircraft::euler(double& roll, double& pitch, double& yaw) const
{
    const double w=_s.q.w,x=_s.q.x,y=_s.q.y,z=_s.q.z;
    roll  = std::atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
    double sp = 2*(w*y - z*x);
    sp = sp > 1 ? 1 : (sp < -1 ? -1 : sp);
    pitch = std::asin(sp);
    yaw   = std::atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

void Aircraft::derivatives(const State& s, const Controls& u, State& d) const
{
    const Vec3 vb = world_to_body(s.q, s.vel_ned);         // body velocity
    const double V = std::sqrt(vb.x*vb.x + vb.y*vb.y + vb.z*vb.z);
    const double Vs = V < 1.0 ? 1.0 : V;                   // guard divides
    const double alpha = std::atan2(vb.z, vb.x);
    const double beta  = std::asin(vb.y / Vs);
    const double qbar  = 0.5 * kRho * V * V;

    const double da = u.aileron  * kDefl;
    const double de = u.elevator * kDefl;
    const double dr = u.rudder   * kDefl;
    const double p = s.omega_body.x, q = s.omega_body.y, r = s.omega_body.z;

    // --- aero force (stability axes -> body), small-angle ---
    const double CL = kCL0 + kCLa * alpha;
    const double CD = kCD0 + kCDk * CL * CL;
    const double CY = kCYb * beta;
    const double lift = qbar * kS * CL;
    const double drag = qbar * kS * CD;
    const double side = qbar * kS * CY;

    // lift acts ~ -z_body, drag ~ -x_body (for small alpha/beta)
    Vec3 F_aero = { -drag, side, -lift };
    Vec3 F_thr  = { u.throttle * kTmax, 0, 0 };
    Vec3 F_grav = world_to_body(s.q, Vec3{ 0, 0, kMass * kG });
    Vec3 F = { F_aero.x + F_thr.x + F_grav.x,
               F_aero.y + F_thr.y + F_grav.y,
               F_aero.z + F_thr.z + F_grav.z };

    d.accel_body = { (F.x - F_grav.x) / kMass,      // specific force (no gravity)
                     (F.y - F_grav.y) / kMass,
                     (F.z - F_grav.z) / kMass };

    // v_dot in world = R (F/m)
    const Vec3 a_body = { F.x / kMass, F.y / kMass, F.z / kMass };
    d.vel_ned = body_to_world(s.q, a_body);
    d.pos_ned = s.vel_ned;

    // --- moments ---
    const double Lm = qbar*kS*kB * (kCl_b*beta + kCl_p*p*kB/(2*Vs) + kCl_da*da);
    const double Mm = qbar*kS*kC * (kCm0 + kCm_a*alpha + kCm_q*q*kC/(2*Vs) + kCm_de*de);
    const double Nm = qbar*kS*kB * (kCn_b*beta + kCn_r*r*kB/(2*Vs) + kCn_dr*dr);

    // omega_dot = I^-1 (M - omega x I omega),  I diagonal
    const double Ipx = kIxx*p, Ipy = kIyy*q, Ipz = kIzz*r;
    d.omega_body = {
        (Lm - (q*Ipz - r*Ipy)) / kIxx,
        (Mm - (r*Ipx - p*Ipz)) / kIyy,
        (Nm - (p*Ipy - q*Ipx)) / kIzz,
    };

    // q_dot = 0.5 * q (x) [0, omega]
    const Quat wq { 0, s.omega_body.x, s.omega_body.y, s.omega_body.z };
    const Quat qd = qmul(s.q, wq);
    d.q = { 0.5*qd.w, 0.5*qd.x, 0.5*qd.y, 0.5*qd.z };
}

void Aircraft::step(const Controls& u, double dt_s)
{
    // fixed sub-stepped Euler for stability at large dt
    const int    n  = 8;
    const double h  = dt_s / n;
    for (int i = 0; i < n; i++) {
        State d;
        derivatives(_s, u, d);
        _s.pos_ned.x   += h * d.pos_ned.x;
        _s.pos_ned.y   += h * d.pos_ned.y;
        _s.pos_ned.z   += h * d.pos_ned.z;
        _s.vel_ned.x   += h * d.vel_ned.x;
        _s.vel_ned.y   += h * d.vel_ned.y;
        _s.vel_ned.z   += h * d.vel_ned.z;
        _s.omega_body.x += h * d.omega_body.x;
        _s.omega_body.y += h * d.omega_body.y;
        _s.omega_body.z += h * d.omega_body.z;
        _s.q.w += h * d.q.w; _s.q.x += h * d.q.x;
        _s.q.y += h * d.q.y; _s.q.z += h * d.q.z;
        _s.q = qnorm(_s.q);
        _s.accel_body = d.accel_body;
    }
}

} // namespace sitl
