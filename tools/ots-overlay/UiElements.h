#pragma once
// Custom RmlUi elements that draw straight into the UI's render target: <lanegraph> (raw vs corrected
// lines per device) and <mapview> (top / side / 3D view of base stations and devices). UiApp fills
// UiScene every update; the elements only read it.
#include "Settings.h"
#include "State.h"

#include <RmlUi/Core/Element.h>
#include <RmlUi/Core/EventListener.h>

#include <string>
#include <vector>

namespace ots {

struct SceneColors {
    Rgb text, dim, line, accent, accentText, ok, hold, lost, beacon, background;
};

// One row of the lane graph: positions over time, in any rigid frame (only distances are used).
struct LaneSeries {
    struct Sample {
        double t;       // seconds, same clock as LaneScene::now
        Vec3 raw, out;  // meters
        bool rawValid;  // false while the tracking driver sends nothing (reset)
        char state;     // 'T' tracking, 'H' hold, 'P' probation, 'B' blend, 'L' lost (uncorrected)
    };
    std::vector<Sample> samples; // oldest first
    bool corrected = false;      // correction is on for this device
};

struct MapItem {
    enum Kind { Device, BaseStation } kind = Device;
    Vec3 pos, raw;       // raw: where the uncorrected pose is (devices); == pos when not correcting
    bool showRaw = false;
    Vec3 forward, right, up;
    float fovH = 150, fovV = 110;
    char state = 'T';    // devices: as LaneSeries::Sample::state
    bool sleeping = false; // base stations SteamVR does not see (asleep or off)
    std::string label;
};

// A body model's joints (standing universe), drawn as a stick figure: neck, shoulder, elbow, hand / waist.
struct Skeleton {
    std::vector<Vec3> joints;
    bool estimating = false; // estimating a hidden device (drawn in the hold colour)
};

struct UiScene {
    SceneColors colors;
    std::vector<Skeleton> skeletons;
    double now = 0;
    double span = 20;                     // seconds shown by the lane graph
    std::vector<LaneSeries> lanes;
    std::vector<MapItem> map;
    std::vector<Vec3> playArea;           // 4 corners, standing universe
    MapMode mapMode = MapMode::ThreeD;
    float camYaw = 0.7f, camPitch = 0.55f, camDistance = 6.0f;
    Vec3 camTarget{0, 1, 0};              // the point the 3D camera orbits around
    bool mapPan = false;                  // left drag pans instead of orbiting (middle / right drag always pan)
    bool cameraChanged = false;           // set by the map when the user orbits / pans / zooms
};

UiScene& Scene();

// Registers <lanegraph> and <mapview> with RmlUi. Call once after Rml::Initialise().
void RegisterUiElements();

} // namespace ots
