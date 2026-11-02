#include "VrModelOverlays.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace ots {
namespace {

Vec3 Sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 Add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 Mul(const Vec3& a, float k) { return {a.x * k, a.y * k, a.z * k}; }
float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 Cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float Len(const Vec3& a) { return std::sqrt(Dot(a, a)); }
Vec3 Norm(const Vec3& a, const Vec3& fallback) {
    const float n = Len(a);
    return n > 1e-5f ? Mul(a, 1 / n) : fallback;
}
Vec3 Transform(const float m[3][4], const Vec3& p) {
    return {m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + m[0][3], m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + m[1][3],
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + m[2][3]};
}

// White RGBA textures: a soft dot, and a strip 32 times as long as it is wide (its overlay is as tall as
// the bone when its width is the bone's length / 32).
constexpr int kDot = 32, kStripW = 4, kStripH = 128;
std::vector<uint8_t> DotPixels() {
    std::vector<uint8_t> p(kDot * kDot * 4);
    for (int y = 0; y < kDot; ++y)
        for (int x = 0; x < kDot; ++x) {
            const float dx = x + .5f - kDot / 2.f, dy = y + .5f - kDot / 2.f, r = std::sqrt(dx * dx + dy * dy) / (kDot / 2.f);
            const uint8_t a = uint8_t(std::clamp((1 - r) * 4, 0.f, 1.f) * 255);
            uint8_t* px = &p[(y * kDot + x) * 4];
            px[0] = px[1] = px[2] = 255, px[3] = a;
        }
    return p;
}
std::vector<uint8_t> StripPixels() {
    std::vector<uint8_t> p(kStripW * kStripH * 4, 255);
    for (int y = 0; y < kStripH; ++y) p[(y * kStripW + 0) * 4 + 3] = p[(y * kStripW + kStripW - 1) * 4 + 3] = 90; // soft edges
    return p;
}

} // namespace

VrModelOverlays::~VrModelOverlays() {
    vr::IVROverlay* ov = vr::VROverlay();
    if (!ov) return;
    for (auto* pool : {&joints_, &bones_})
        for (Quad& q : *pool)
            if (q.handle != vr::k_ulOverlayHandleInvalid) ov->DestroyOverlay(q.handle);
}

VrModelOverlays::Quad* VrModelOverlays::Get(std::vector<Quad>& pool, size_t i, bool bone) {
    while (pool.size() <= i) {
        if (failed_) return nullptr;
        vr::IVROverlay* ov = vr::VROverlay();
        Quad q;
        const std::string key = std::string("ots.trackingstabilizer.model.") + (bone ? "bone" : "joint") + std::to_string(pool.size());
        if (!ov || ov->CreateOverlay(key.c_str(), "Tracking Stabilizer body model", &q.handle) != vr::VROverlayError_None) {
            failed_ = true;
            return nullptr;
        }
        static const std::vector<uint8_t> dot = DotPixels(), strip = StripPixels();
        const auto& px = bone ? strip : dot;
        ov->SetOverlayRaw(q.handle, const_cast<uint8_t*>(px.data()), bone ? kStripW : kDot, bone ? kStripH : kDot, 4);
        ov->SetOverlayAlpha(q.handle, .9f);
        pool.push_back(q);
    }
    return &pool[i];
}

void VrModelOverlays::Place(Quad& q, const Vec3& c, const Vec3& x, const Vec3& y, const Vec3& z, float width, bool estimating) {
    vr::IVROverlay* ov = vr::VROverlay();
    vr::HmdMatrix34_t m{{{x.x, y.x, z.x, c.x}, {x.y, y.y, z.y, c.y}, {x.z, y.z, z.z, c.z}}};
    ov->SetOverlayTransformAbsolute(q.handle, vr::TrackingUniverseStanding, &m);
    ov->SetOverlayWidthInMeters(q.handle, width);
    if (estimating) ov->SetOverlayColor(q.handle, 1.f, .62f, .2f); // the map's "correcting" orange
    else ov->SetOverlayColor(q.handle, .45f, .72f, 1.f);
    if (!q.shown) ov->ShowOverlay(q.handle), q.shown = true;
}

void VrModelOverlays::HideFrom(std::vector<Quad>& pool, size_t first) {
    vr::IVROverlay* ov = vr::VROverlay();
    for (size_t i = first; i < pool.size(); ++i)
        if (pool[i].shown) ov->HideOverlay(pool[i].handle), pool[i].shown = false;
}

void VrModelOverlays::Update(const std::vector<ModelPose>& models, const float rawToStanding[3][4], const Vec3& eye, bool show) {
    if (!vr::VROverlay()) return;
    size_t nj = 0, nb = 0;
    if (show)
        for (const ModelPose& m : models) {
            std::vector<Vec3> chain{Transform(rawToStanding, m.neck)};
            if (m.haveShoulder) chain.push_back(Transform(rawToStanding, m.shoulder));
            if (m.haveElbow) chain.push_back(Transform(rawToStanding, m.elbow));
            chain.push_back(Transform(rawToStanding, m.end));
            for (const Vec3& p : chain) { // dot facing the HMD
                Quad* q = Get(joints_, nj, false);
                if (!q) return;
                const Vec3 z = Norm(Sub(eye, p), {0, 0, 1}), x = Norm(Cross({0, 1, 0}, z), {1, 0, 0}), y = Cross(z, x);
                Place(*q, p, x, y, z, &p == &chain.back() ? .045f : .03f, m.estimating);
                ++nj;
            }
            for (size_t i = 0; i + 1 < chain.size(); ++i) { // strip along the bone, turned about it towards the HMD
                const Vec3 a = chain[i], b = chain[i + 1], c = Mul(Add(a, b), .5f);
                const float len = Len(Sub(b, a));
                if (len < .01f) continue;
                Quad* q = Get(bones_, nb, true);
                if (!q) return;
                const Vec3 y = Mul(Sub(b, a), 1 / len), toEye = Sub(eye, c);
                const Vec3 z = Norm(Sub(toEye, Mul(y, Dot(toEye, y))), {0, 0, 1}), x = Cross(y, z);
                Place(*q, c, x, y, z, len / (kStripH / kStripW), m.estimating);
                ++nb;
            }
        }
    HideFrom(joints_, nj);
    HideFrom(bones_, nb);
}

} // namespace ots
