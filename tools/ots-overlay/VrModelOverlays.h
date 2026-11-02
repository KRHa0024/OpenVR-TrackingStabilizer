#pragma once
// The body model's joints as world overlays around the user (debugging): a dot per joint and a thin strip
// per bone, both turned towards the HMD. Overlays are flat quads, so this is the cheap way to draw a
// stick figure in the world; SteamVR composites them over the game.
#include "State.h"

#include <openvr.h>

#include <vector>

namespace ots {

class VrModelOverlays {
public:
    ~VrModelOverlays();
    // Every frame. `models` in driver world space, `rawToStanding` maps it to the standing universe where
    // `eye` (the HMD) is. show = false hides everything (the overlays are kept for the next time).
    void Update(const std::vector<ModelPose>& models, const float rawToStanding[3][4], const Vec3& eye, bool show);

private:
    struct Quad {
        vr::VROverlayHandle_t handle = vr::k_ulOverlayHandleInvalid;
        bool shown = false;
    };
    Quad* Get(std::vector<Quad>& pool, size_t i, bool bone);
    void Place(Quad& q, const Vec3& center, const Vec3& x, const Vec3& y, const Vec3& z, float width, bool estimating);
    static void HideFrom(std::vector<Quad>& pool, size_t first);

    std::vector<Quad> joints_, bones_;
    bool failed_ = false; // overlay creation failed once: stop trying
};

} // namespace ots
