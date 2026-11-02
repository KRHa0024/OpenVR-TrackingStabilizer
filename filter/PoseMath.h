#pragma once
#include <openvr_driver.h>
#include <algorithm>
#include <cmath>

namespace ots::filter::math {
struct Vec { double x = 0, y = 0, z = 0; };
using Quat = vr::HmdQuaternion_t;
inline Vec operator+(Vec a, Vec b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec operator-(Vec a, Vec b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec operator*(Vec a, double s) { return {a.x*s, a.y*s, a.z*s}; }
inline double Length(Vec a) { return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z); }
inline bool Finite(Vec a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }
inline Quat Conjugate(Quat q) { return {q.w,-q.x,-q.y,-q.z}; }
inline Quat Multiply(Quat a, Quat b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z, a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x, a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline bool Unit(Quat q) {
    const double n = q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z;
    return std::isfinite(n) && std::abs(n-1) < .02;
}
inline Vec Rotate(Quat q, Vec v) {
    const Quat r = Multiply(Multiply(q, {0,v.x,v.y,v.z}), Conjugate(q));
    return {r.x,r.y,r.z};
}
inline Quat WorldRotation(const vr::DriverPose_t& p) { return Multiply(p.qWorldFromDriverRotation,p.qRotation); }
inline Vec WorldPosition(const vr::DriverPose_t& p) {
    return Rotate(p.qWorldFromDriverRotation, {p.vecPosition[0],p.vecPosition[1],p.vecPosition[2]}) +
           Vec{p.vecWorldFromDriverTranslation[0],p.vecWorldFromDriverTranslation[1],p.vecWorldFromDriverTranslation[2]};
}
inline void SetRotation(vr::DriverPose_t& p, Quat world) { p.qRotation = Multiply(Conjugate(p.qWorldFromDriverRotation),world); }
inline double Angle(Quat a, Quat b) {
    return 2*std::acos(std::clamp(std::abs(a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z),0.0,1.0));
}
inline Quat Interpolate(Quat a, Quat b, double u) {
    double d = a.w*b.w+a.x*b.x+a.y*b.y+a.z*b.z;
    if (d < 0) { b = {-b.w,-b.x,-b.y,-b.z}; d = -d; }
    double x = 1-u, y = u;
    if (d < .9995) { const double theta = std::acos(std::clamp(d,0.0,1.0)); x = std::sin((1-u)*theta)/std::sin(theta); y = std::sin(u*theta)/std::sin(theta); }
    Quat q{a.w*x+b.w*y,a.x*x+b.x*y,a.y*x+b.y*y,a.z*x+b.z*y};
    const double n = std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    return {q.w/n,q.x/n,q.y/n,q.z/n};
}
inline bool Usable(const vr::DriverPose_t& p) {
    return p.deviceIsConnected && p.poseIsValid && Unit(p.qWorldFromDriverRotation) && Unit(p.qRotation) &&
           Finite(WorldPosition(p)) && (p.vecPosition[0] != 0 || p.vecPosition[1] != 0 || p.vecPosition[2] != 0);
}
inline bool Good(const vr::DriverPose_t& p) { return Usable(p) && p.result == vr::TrackingResult_Running_OK; }
} // namespace ots::filter::math
