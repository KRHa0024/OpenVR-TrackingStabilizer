#include "UiElements.h"

#include <RmlUi/Core/Context.h>
#include <RmlUi/Core/ElementDocument.h>
#include <RmlUi/Core/ElementInstancer.h>
#include <RmlUi/Core/Factory.h>
#include <RmlUi/Core/Geometry.h>
#include <RmlUi/Core/Mesh.h>
#include <RmlUi/Core/RenderManager.h>

#include <algorithm>
#include <cmath>

namespace ots {
namespace {

constexpr float kPi = 3.14159265f;

Rml::ColourbPremultiplied Col(const Rgb& c, float a = 1) {
    auto b = [](float v) { return Rml::byte(std::clamp(int(std::lround(v * 255)), 0, 255)); };
    return Rml::Colourb(b(c.r), b(c.g), b(c.b), b(a)).ToPremultiplied();
}

// The context's dp ratio: the custom elements lay out and draw in dp, like the stylesheet, so the UI
// looks the same at any render resolution.
float DpRatio(const Rml::Element* e) {
    const Rml::Context* ctx = e->GetContext();
    return ctx ? ctx->GetDensityIndependentPixelRatio() : 1.0f;
}

// Collects flat-colored triangles in element-local dp; vertices are scaled to pixels by `k`.
class MeshBuilder {
public:
    explicit MeshBuilder(float k) : k_(k) {}
    void Quad(Rml::Vector2f a, Rml::Vector2f b, Rml::Vector2f c, Rml::Vector2f d, Rml::ColourbPremultiplied col) {
        const int base = int(mesh_.vertices.size());
        for (auto p : {a, b, c, d}) Vertex(p, col);
        for (int i : {0, 1, 2, 0, 2, 3}) mesh_.indices.push_back(base + i);
    }
    void Rect(float x, float y, float w, float h, Rml::ColourbPremultiplied col) { Quad({x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}, col); }
    void Line(Rml::Vector2f a, Rml::Vector2f b, float width, Rml::ColourbPremultiplied col) {
        Rml::Vector2f d = b - a;
        const float len = std::sqrt(d.x * d.x + d.y * d.y);
        if (len < 1e-3f) return;
        const Rml::Vector2f n = Rml::Vector2f(-d.y, d.x) * (0.5f * width / len);
        const Rml::Vector2f e = d * (0.35f * width / len); // small overlap so joints do not crack
        Quad(a - e + n, b + e + n, b + e - n, a - e - n, col);
    }
    // Dashed when dash > 0 (dash and gap of that length, continuing across segments).
    void Polyline(const std::vector<Rml::Vector2f>& pts, float width, Rml::ColourbPremultiplied col, float dash = 0) {
        float phase = 0;
        for (size_t i = 1; i < pts.size(); ++i) {
            const Rml::Vector2f a = pts[i - 1], b = pts[i];
            if (dash <= 0) {
                Line(a, b, width, col);
                continue;
            }
            const Rml::Vector2f d = b - a;
            const float len = std::sqrt(d.x * d.x + d.y * d.y);
            float s = 0;
            while (s < len) {
                const float cycle = std::fmod(phase, 2 * dash);
                const float step = std::min(len - s, (cycle < dash ? dash : 2 * dash) - cycle);
                if (cycle < dash && len > 0) Line(a + d * (s / len), a + d * ((s + step) / len), width, col);
                s += step;
                phase += step;
            }
        }
    }
    void Circle(Rml::Vector2f c, float r, Rml::ColourbPremultiplied col) {
        const int n = 24, base = int(mesh_.vertices.size());
        Vertex(c, col);
        for (int i = 0; i < n; ++i) {
            const float a = 2 * kPi * i / n;
            Vertex(c + Rml::Vector2f(std::cos(a), std::sin(a)) * r, col);
        }
        for (int i = 0; i < n; ++i) mesh_.indices.insert(mesh_.indices.end(), {base, base + 1 + i, base + 1 + (i + 1) % n});
    }
    void Ring(Rml::Vector2f c, float r, float width, Rml::ColourbPremultiplied col, bool dashed = false) {
        const int n = 32;
        for (int i = 0; i < n; ++i) {
            if (dashed && i % 2) continue;
            const float a0 = 2 * kPi * i / n, a1 = 2 * kPi * (i + 1) / n;
            const Rml::Vector2f u0(std::cos(a0), std::sin(a0)), u1(std::cos(a1), std::sin(a1));
            Quad(c + u0 * (r - width / 2), c + u0 * (r + width / 2), c + u1 * (r + width / 2), c + u1 * (r - width / 2), col);
        }
    }
    // Convex polygon (or a fan around its first point).
    void Fan(const std::vector<Rml::Vector2f>& pts, Rml::ColourbPremultiplied col) {
        if (pts.size() < 3) return;
        const int base = int(mesh_.vertices.size());
        for (auto p : pts) Vertex(p, col);
        for (int i = 1; i + 1 < int(pts.size()); ++i) mesh_.indices.insert(mesh_.indices.end(), {base, base + i, base + i + 1});
    }
    void Render(Rml::RenderManager& rm, Rml::Vector2f offset) {
        if (mesh_.indices.empty()) return;
        Rml::Geometry g = rm.MakeGeometry(std::move(mesh_));
        g.Render(offset);
        mesh_ = {};
    }

private:
    void Vertex(Rml::Vector2f p, Rml::ColourbPremultiplied col) { mesh_.vertices.push_back({p * k_, col, {0, 0}}); }

    float k_;
    Rml::Mesh mesh_;
};

float Dist(const Vec3& a, const Vec3& b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z)); }

// ---------------------------------------------------------------------------------------------
// <lanegraph>: one lane per device. Both lines are the distance (cm) from the device's average
// corrected position over the window, so a fly-away in any direction shows up; the gap between
// them is shaded and a thin strip on top marks correction / loss.
class ElementLaneGraph : public Rml::Element {
public:
    using Rml::Element::Element;

protected:
    void OnRender() override {
        const UiScene& sc = Scene();
        const float dp = DpRatio(this);
        const Rml::Vector2f size = GetBox().GetSize(Rml::BoxArea::Content) / dp;
        const Rml::Vector2f origin = GetAbsoluteOffset(Rml::BoxArea::Content);
        const size_t n = sc.lanes.size();
        if (n == 0 || size.x < 50 || size.y < 50) return;
        const SceneColors& c = sc.colors;
        constexpr float kLeft = 10, kRight = 14, kAxis = 28, kMaxCm = 40;
        const float laneH = (size.y - kAxis) / float(n);
        const double t0 = sc.now - sc.span;
        auto X = [&](double t) { return kLeft + float((t - t0) / sc.span) * (size.x - kLeft - kRight); };

        MeshBuilder mb(dp);
        for (size_t li = 0; li < n; ++li) {
            const LaneSeries& lane = sc.lanes[li];
            const float top = li * laneH, bottom = top + laneH - 12, plotTop = top + 22;
            auto Y = [&](float cm) { return bottom - std::min(cm, kMaxCm) / kMaxCm * (bottom - plotTop); };
            if (li > 0) mb.Rect(0, top, size.x, 1, Col(c.line));
            mb.Rect(kLeft, bottom, size.x - kLeft - kRight, 1, Col(c.text, 0.14f));
            mb.Polyline({{kLeft, Y(kMaxCm / 2)}, {size.x - kRight, Y(kMaxCm / 2)}}, 1, Col(c.text, 0.07f), 3);

            // Visible samples and the anchor (mean corrected position).
            std::vector<const LaneSeries::Sample*> vis;
            Vec3 mean;
            for (const auto& s : lane.samples)
                if (s.t >= t0 - 0.2 && s.t <= sc.now + 0.05) {
                    vis.push_back(&s);
                    mean.x += s.out.x, mean.y += s.out.y, mean.z += s.out.z;
                }
            if (vis.size() < 2) continue;
            mean.x /= vis.size(), mean.y /= vis.size(), mean.z /= vis.size();

            // Shaded gap + state strip per run of non-tracking samples.
            for (size_t i = 0; i < vis.size();) {
                if (vis[i]->state == 'T') { ++i; continue; }
                const bool lost = vis[i]->state == 'L';
                size_t j = i;
                while (j < vis.size() && vis[j]->state != 'T' && (vis[j]->state == 'L') == lost) ++j;
                const float x0 = X(vis[i]->t), x1 = X(vis[j - 1]->t);
                const Rgb& sc0 = lost ? c.lost : c.hold;
                const float stripA = lost ? 1.0f : vis[i]->state == 'H' ? 1.0f : vis[i]->state == 'P' ? 0.55f : 0.3f;
                mb.Rect(x0, top + 8, std::max(2.0f, x1 - x0), 4, Col(sc0, stripA));
                for (size_t k = i + 1; k < j; ++k) {
                    const auto* a = vis[k - 1];
                    const auto* b = vis[k];
                    if (!a->rawValid || !b->rawValid) continue;
                    const float ya = Y(Dist(a->raw, mean) * 100), yb = Y(Dist(b->raw, mean) * 100);
                    const float oa = Y(Dist(a->out, mean) * 100), ob = Y(Dist(b->out, mean) * 100);
                    mb.Quad({X(a->t), ya}, {X(b->t), yb}, {X(b->t), ob}, {X(a->t), oa}, Col(sc0, 0.22f));
                }
                i = j;
            }

            // Raw (dashed, broken where the driver sent nothing) and corrected (solid) lines.
            std::vector<Rml::Vector2f> raw, out;
            for (const auto* s : vis) {
                out.push_back({X(s->t), Y(Dist(s->out, mean) * 100)});
                if (s->rawValid) {
                    raw.push_back({X(s->t), Y(Dist(s->raw, mean) * 100)});
                } else {
                    mb.Polyline(raw, 1.5f, Col(lane.corrected ? c.dim : c.lost), 5);
                    raw.clear();
                }
            }
            mb.Polyline(raw, 1.5f, Col(lane.corrected ? c.dim : c.lost), 5);
            mb.Polyline(out, 2.5f, Col(c.accentText));
        }
        mb.Render(*GetRenderManager(), origin);
    }
};

// ---------------------------------------------------------------------------------------------
// <mapview>
class ElementMapView : public Rml::Element, public Rml::EventListener {
public:
    using Rml::Element::Element;
    ~ElementMapView() override {
        RemoveEventListener(Rml::EventId::Mousedown, this);
        RemoveEventListener(Rml::EventId::Mousemove, this);
        RemoveEventListener(Rml::EventId::Mouseup, this);
        RemoveEventListener(Rml::EventId::Mousescroll, this);
    }

protected:
    void OnChildAdd(Rml::Element* child) override {
        Rml::Element::OnChildAdd(child);
        if (child == this) {
            AddEventListener(Rml::EventId::Mousedown, this);
            AddEventListener(Rml::EventId::Mousemove, this);
            AddEventListener(Rml::EventId::Mouseup, this);
            AddEventListener(Rml::EventId::Mousescroll, this);
        }
    }

    void ProcessEvent(Rml::Event& ev) override {
        UiScene& sc = Scene();
        if (sc.mapMode != MapMode::ThreeD) return;
        const Rml::Vector2f mouse = Rml::Vector2f(ev.GetParameter("mouse_x", 0.0f), ev.GetParameter("mouse_y", 0.0f)) / DpRatio(this);
        switch (ev.GetId()) {
        case Rml::EventId::Mousedown:
            dragging_ = true;
            panning_ = sc.mapPan || ev.GetParameter("button", 0) != 0;
            last_ = mouse;
            break;
        case Rml::EventId::Mousemove:
            // The drag lasts while the button is held, wherever the pointer went in between (RmlUi keeps
            // the pressed element "active" until the button comes up, even outside it).
            if (dragging_ && !IsPseudoClassSet("active")) dragging_ = false;
            if (dragging_) {
                const float dx = mouse.x - last_.x, dy = mouse.y - last_.y;
                if (panning_) Pan(sc, dx, dy);
                else {
                    sc.camYaw -= dx * 0.008f;
                    sc.camPitch = std::clamp(sc.camPitch + dy * 0.006f, 0.05f, 1.5f);
                }
                sc.cameraChanged = true;
                last_ = mouse;
            }
            break;
        case Rml::EventId::Mouseup:
            dragging_ = false;
            break;
        case Rml::EventId::Mousescroll: {
            const float d = ev.GetParameter("wheel_delta_y", 0.0f);
            if (d != 0) {
                sc.camDistance = std::clamp(sc.camDistance * (d > 0 ? 1.1f : 1 / 1.1f), 1.5f, 25.0f);
                sc.cameraChanged = true;
            }
            ev.StopPropagation();
            break;
        }
        default:
            break;
        }
    }

    // Moves the orbit target so the floor follows the pointer (at the target's depth).
    void Pan(UiScene& sc, float dx, float dy) {
        const float k = sc.camDistance / std::max(1.0f, GetBox().GetSize().y / DpRatio(this) * 0.95f); // metres per dp, as Build3D's focal
        const float sy = std::sin(sc.camYaw), cy = std::cos(sc.camYaw), sp = std::sin(sc.camPitch), cp = std::cos(sc.camPitch);
        const Vec3 right{cy, 0, -sy}, up{-sp * sy, cp, -sp * cy};
        Vec3& t = sc.camTarget;
        t.x = std::clamp(t.x - (right.x * dx - up.x * dy) * k, -15.0f, 15.0f);
        t.y = std::clamp(t.y - (right.y * dx - up.y * dy) * k, -2.0f, 5.0f);
        t.z = std::clamp(t.z - (right.z * dx - up.z * dy) * k, -15.0f, 15.0f);
    }

    void OnUpdate() override {
        Build();
        SetClass("orbit", Scene().mapMode == MapMode::ThreeD && !Scene().mapPan);
        SetClass("pan", Scene().mapMode == MapMode::ThreeD && Scene().mapPan);
        // Labels are child elements so they use the UI font; reuse them between frames.
        while (int(labels_.size()) < int(texts_.size())) {
            Rml::ElementPtr e = GetOwnerDocument()->CreateElement("span");
            e->SetClass("label", true);
            labels_.push_back(AppendChild(std::move(e)));
        }
        for (size_t i = 0; i < labels_.size(); ++i) {
            Rml::Element* e = labels_[i];
            if (i >= texts_.size()) {
                e->SetProperty("display", "none");
                continue;
            }
            const Text& t = texts_[i];
            e->SetProperty("display", "inline-block");
            if (e->GetInnerRML() != t.text) e->SetInnerRML(t.text);
            e->SetProperty("left", Rml::ToString(t.pos.x) + "dp");
            e->SetProperty("top", Rml::ToString(t.pos.y - 16) + "dp");
            e->SetProperty("color", t.color);
        }
    }

    void OnRender() override {
        const Rml::Vector2f origin = GetAbsoluteOffset(Rml::BoxArea::Content);
        MeshBuilder mb(DpRatio(this));
        for (const Prim& p : prims_) {
            switch (p.kind) {
            case Prim::LineK: mb.Polyline(p.pts, p.width, p.color, p.dash); break;
            case Prim::FillK: mb.Fan(p.pts, p.color); break;
            case Prim::DotK: mb.Circle(p.pts[0], p.width, p.color); break;
            case Prim::RingK: mb.Ring(p.pts[0], p.width, 2.5f, p.color, p.dash > 0); break;
            case Prim::SquareK: mb.Rect(p.pts[0].x - 7, p.pts[0].y - 7, 14, 14, p.color); break;
            }
        }
        mb.Render(*GetRenderManager(), origin);
    }

private:
    struct Prim {
        enum Kind { LineK, FillK, DotK, RingK, SquareK } kind;
        std::vector<Rml::Vector2f> pts;
        Rml::ColourbPremultiplied color;
        float width = 1;
        float dash = 0;
    };
    struct Text {
        Rml::Vector2f pos;
        std::string text, color;
    };

    static std::string Css(const Rgb& c) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "#%02x%02x%02x", int(c.r * 255 + 0.5f), int(c.g * 255 + 0.5f), int(c.b * 255 + 0.5f));
        return buf;
    }
    Rgb StateColor(char s) const {
        const SceneColors& c = Scene().colors;
        return s == 'T' ? c.ok : s == 'L' ? c.lost : c.hold;
    }

    void Line(std::vector<Rml::Vector2f> pts, const Rgb& c, float a, float w, float dash = 0) {
        prims_.push_back({Prim::LineK, std::move(pts), Col(c, a), w, dash});
    }
    void Device(const MapItem& it, Rml::Vector2f p, float r, Rml::Vector2f* rawP, const std::string& text) {
        const SceneColors& c = Scene().colors;
        if (rawP) {
            Line({p, *rawP}, c.lost, 0.8f, 1.5f, 4);
            prims_.push_back({Prim::RingK, {*rawP}, Col(c.lost), r * 0.8f, 1});
        }
        if (it.state != 'T' && it.state != 'L') prims_.push_back({Prim::RingK, {p}, Col(c.hold), r + 4, 0});
        prims_.push_back({Prim::DotK, {p}, Col(StateColor(it.state)), r, 0});
        texts_.push_back({{p.x + r + (it.state == 'T' ? 4 : 10), p.y - r + 4}, text, Css(c.text)});
    }

    void Build() {
        prims_.clear();
        texts_.clear();
        const UiScene& sc = Scene();
        const SceneColors& c = sc.colors;
        const Rml::Vector2f size = GetBox().GetSize(Rml::BoxArea::Content) / DpRatio(this);
        if (size.x < 50 || size.y < 50) return;
        if (sc.mapMode == MapMode::ThreeD) Build3D(sc, size);
        else Build2D(sc, size, sc.mapMode == MapMode::Side);
        // Devices close together would print their labels on top of each other: move a label down a
        // line while it overlaps one placed before it (labels are short serial tails, ~8 dp a glyph).
        for (size_t i = 0; i < texts_.size(); ++i) {
            const float wi = 8.0f * texts_[i].text.size();
            for (int tries = 0; tries < 6; ++tries) {
                bool overlap = false;
                for (size_t j = 0; j < i && !overlap; ++j) {
                    const float wj = 8.0f * texts_[j].text.size();
                    overlap = texts_[i].pos.x < texts_[j].pos.x + wj && texts_[j].pos.x < texts_[i].pos.x + wi &&
                              std::fabs(texts_[i].pos.y - texts_[j].pos.y) < 15;
                }
                if (!overlap) break;
                texts_[i].pos.y += 15;
            }
        }
        (void)c;
    }

    void Build2D(const UiScene& sc, Rml::Vector2f size, bool side) {
        const SceneColors& c = sc.colors;
        auto vert = [&](const Vec3& p) { return side ? -p.y : p.z; };
        float minH = -1, maxH = 1, minV = -1, maxV = 1;
        auto ext = [&](const Vec3& p) {
            minH = std::min(minH, p.x), maxH = std::max(maxH, p.x), minV = std::min(minV, vert(p)), maxV = std::max(maxV, vert(p));
        };
        for (const Vec3& p : sc.playArea) ext(p);
        for (const MapItem& it : sc.map) ext(it.pos);
        if (side) minV = std::min(minV, 0.0f), maxV = std::max(maxV, 0.0f);
        minH -= 0.6f, maxH += 0.6f, minV -= 0.6f, maxV += 0.6f;
        const float plotH = size.y - 36;
        const float scale = std::min(size.x / (maxH - minH), plotH / (maxV - minV));
        const float cx = size.x / 2, cy = plotH / 2, mh = (minH + maxH) / 2, mv = (minV + maxV) / 2;
        auto S = [&](float h, float v) { return Rml::Vector2f(cx + (h - mh) * scale, cy + (v - mv) * scale); };

        for (int g = int(std::floor(minH)); g <= maxH; ++g) Line({S(float(g), minV), S(float(g), maxV)}, c.text, 0.08f, 1);
        for (int g = int(std::floor(minV)); g <= maxV; ++g) Line({S(minH, float(g)), S(maxH, float(g))}, c.text, 0.08f, 1);
        if (side) {
            Line({S(minH, 0), S(maxH, 0)}, c.text, 0.4f, 2);
        } else if (sc.playArea.size() == 4) {
            std::vector<Rml::Vector2f> pa;
            for (const Vec3& p : sc.playArea) pa.push_back(S(p.x, p.z));
            prims_.push_back({Prim::FillK, pa, Col(c.accent, 0.06f)});
            pa.push_back(pa.front());
            Line(pa, c.accent, 0.9f, 2);
        }
        for (const MapItem& it : sc.map) {
            if (it.kind != MapItem::BaseStation) continue;
            const Rml::Vector2f p = S(it.pos.x, vert(it.pos));
            const float fh = it.forward.x, fv = side ? -it.forward.y : it.forward.z;
            const float head = std::atan2(fv, fh), half = (side ? it.fovV : it.fovH) / 2 * kPi / 180, reach = 2.5f * scale;
            std::vector<Rml::Vector2f> fan{p};
            for (int i = 0; i <= 16; ++i) {
                const float a = head - half + 2 * half * i / 16;
                fan.push_back(p + Rml::Vector2f(std::cos(a), std::sin(a)) * reach);
            }
            prims_.push_back({Prim::FillK, fan, Col(c.beacon, it.sleeping ? 0.03f : 0.10f)});
            const Rml::Vector2f tip = p + Rml::Vector2f(std::cos(head), std::sin(head)) * 20.0f;
            const Rml::Vector2f l = p + Rml::Vector2f(std::cos(head + 2.5f), std::sin(head + 2.5f)) * 13.0f;
            const Rml::Vector2f r = p + Rml::Vector2f(std::cos(head - 2.5f), std::sin(head - 2.5f)) * 13.0f;
            prims_.push_back({Prim::FillK, {tip, l, r}, Col(it.sleeping ? c.dim : c.beacon)});
            texts_.push_back({{p.x + 14, p.y + 20}, it.label, Css(c.beacon)});
        }
        for (const MapItem& it : sc.map) {
            if (it.kind != MapItem::Device) continue;
            const Rml::Vector2f p = S(it.pos.x, vert(it.pos));
            Rml::Vector2f raw = S(it.raw.x, vert(it.raw));
            Device(it, p, 5, it.showRaw ? &raw : nullptr, it.label);
        }
        for (const Skeleton& k : sc.skeletons) { // body model joints
            const Rgb& col = k.estimating ? c.hold : c.accentText;
            std::vector<Rml::Vector2f> pts;
            for (const Vec3& j : k.joints) pts.push_back(S(j.x, vert(j)));
            Line(pts, col, 0.9f, 2);
            for (size_t i = 0; i + 1 < pts.size(); ++i) prims_.push_back({Prim::DotK, {pts[i]}, Col(col), 3, 0}); // the hand has its own dot
        }
    }

    void Build3D(const UiScene& sc, Rml::Vector2f size) {
        const SceneColors& c = sc.colors;
        struct V3 {
            float x, y, z;
        };
        auto sub = [](V3 a, V3 b) { return V3{a.x - b.x, a.y - b.y, a.z - b.z}; };
        auto add = [](V3 a, V3 b) { return V3{a.x + b.x, a.y + b.y, a.z + b.z}; };
        auto mul = [](V3 a, float k) { return V3{a.x * k, a.y * k, a.z * k}; };
        auto dot = [](V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; };
        auto cross = [](V3 a, V3 b) { return V3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; };
        auto norm = [&](V3 a) {
            const float l = std::sqrt(dot(a, a));
            return l > 1e-6f ? mul(a, 1 / l) : a;
        };
        auto v3 = [](const Vec3& p) { return V3{p.x, p.y, p.z}; };

        const V3 target = v3(sc.camTarget);
        const float cp = std::cos(sc.camPitch);
        const V3 eye = add(target, mul(V3{cp * std::sin(sc.camYaw), std::sin(sc.camPitch), cp * std::cos(sc.camYaw)}, sc.camDistance));
        const V3 fwd = norm(sub(target, eye)), right = norm(cross(fwd, V3{0, 1, 0})), up = cross(right, fwd);
        const float focal = size.y * 0.95f, cx = size.x / 2, cy = size.y / 2 - 8, kNear = 0.1f;
        auto view = [&](V3 p) {
            const V3 d = sub(p, eye);
            return V3{dot(d, right), dot(d, up), dot(d, fwd)};
        };
        auto proj = [&](V3 v) { return Rml::Vector2f(cx + v.x / v.z * focal, cy - v.y / v.z * focal); };
        auto line = [&](V3 a, V3 b, const Rgb& col, float w, float alpha) {
            V3 va = view(a), vb = view(b);
            if (va.z < kNear && vb.z < kNear) return;
            if (va.z < kNear) va = add(va, mul(sub(vb, va), (kNear - va.z) / (vb.z - va.z)));
            if (vb.z < kNear) vb = add(vb, mul(sub(va, vb), (kNear - vb.z) / (va.z - vb.z)));
            Line({proj(va), proj(vb)}, col, alpha, w);
        };

        for (int i = -4; i <= 4; ++i) {
            line({float(i), 0, -4}, {float(i), 0, 4}, c.text, 1, 0.10f);
            line({-4, 0, float(i)}, {4, 0, float(i)}, c.text, 1, 0.10f);
        }
        if (sc.playArea.size() == 4) {
            std::vector<Rml::Vector2f> pa;
            bool ok = true;
            for (const Vec3& p : sc.playArea) {
                const V3 v = view(v3(p));
                ok = ok && v.z > kNear;
                pa.push_back(ok ? proj(v) : Rml::Vector2f());
            }
            if (ok) {
                prims_.push_back({Prim::FillK, pa, Col(c.accent, 0.08f)});
                pa.push_back(pa.front());
                Line(pa, c.accent, 0.9f, 2.5f);
            }
        }
        for (const MapItem& it : sc.map) {
            if (it.kind != MapItem::BaseStation) continue;
            const V3 apex = v3(it.pos), f = norm(v3(it.forward));
            const V3 r = norm(cross(f, V3{0, 1, 0})), u = cross(r, f);
            const float len = 1.2f, th = std::tan(it.fovH / 2 * kPi / 180) * len, tv = std::tan(it.fovV / 2 * kPi / 180) * len;
            const V3 base = add(apex, mul(f, len));
            const V3 cs[4] = {add(add(base, mul(r, th)), mul(u, tv)), add(sub(base, mul(r, th)), mul(u, tv)),
                              sub(sub(base, mul(r, th)), mul(u, tv)), sub(add(base, mul(r, th)), mul(u, tv))};
            const Rgb& col = it.sleeping ? c.dim : c.beacon;
            for (int i = 0; i < 4; ++i) {
                line(apex, cs[i], col, 1.2f, 0.45f);
                line(cs[i], cs[(i + 1) % 4], col, 1.2f, 0.45f);
            }
            line(apex, add(apex, mul(f, 0.35f)), col, 3, 1);
            line(apex, {apex.x, 0, apex.z}, col, 1, 0.25f);
            const V3 v = view(apex);
            if (v.z > kNear) {
                const Rml::Vector2f p = proj(v);
                prims_.push_back({Prim::SquareK, {p}, Col(col)});
                texts_.push_back({{p.x + 10, p.y - 12}, it.label, Css(col)});
            }
        }
        std::vector<std::pair<float, const MapItem*>> order;
        for (const MapItem& it : sc.map)
            if (it.kind == MapItem::Device) {
                const V3 v = view(v3(it.pos));
                if (v.z > kNear) order.push_back({v.z, &it});
            }
        std::sort(order.begin(), order.end(), [](auto& a, auto& b) { return a.first > b.first; }); // far first
        for (auto [z, it] : order) {
            line(v3(it->pos), {it->pos.x, 0, it->pos.z}, c.text, 1, 0.25f);
            const Rml::Vector2f p = proj(view(v3(it->pos)));
            const float rad = std::clamp(28 / z, 3.5f, 8.0f); // small enough to tell devices apart when they bunch up
            const V3 rv = view(v3(it->raw));
            Rml::Vector2f raw = rv.z > kNear ? proj(rv) : p;
            Device(*it, p, rad, it->showRaw && rv.z > kNear ? &raw : nullptr, it->label);
        }
        for (const Skeleton& k : sc.skeletons) { // body model joints
            const Rgb& col = k.estimating ? c.hold : c.accentText;
            for (size_t i = 0; i + 1 < k.joints.size(); ++i) line(v3(k.joints[i]), v3(k.joints[i + 1]), col, 2.5f, 0.9f);
            for (size_t i = 0; i + 1 < k.joints.size(); ++i) { // the hand / waist has its own dot
                const V3 v = view(v3(k.joints[i]));
                if (v.z > kNear) prims_.push_back({Prim::DotK, {proj(v)}, Col(col), std::clamp(18 / v.z, 2.5f, 5.0f), 0});
            }
        }
    }

    std::vector<Prim> prims_;
    std::vector<Text> texts_;
    std::vector<Rml::Element*> labels_;
    bool dragging_ = false, panning_ = false;
    Rml::Vector2f last_;
};

} // namespace

UiScene& Scene() {
    static UiScene scene;
    return scene;
}

void RegisterUiElements() {
    static Rml::ElementInstancerGeneric<ElementLaneGraph> lanes;
    static Rml::ElementInstancerGeneric<ElementMapView> map;
    Rml::Factory::RegisterElementInstancer("lanegraph", &lanes);
    Rml::Factory::RegisterElementInstancer("mapview", &map);
}

} // namespace ots
