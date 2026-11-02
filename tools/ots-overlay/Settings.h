#pragma once
// Overlay-only preferences (theme, language, map view, log filters, desktop window), stored as JSON in
// %LOCALAPPDATA%\OpenVR-TrackingStabilizer\overlay.json. Filter settings live in steamvr.vrsettings.
#include <filesystem>
#include <map>
#include <string>

namespace ots {

struct Rgb {
    float r = 0, g = 0, b = 0; // 0..1
};

struct Theme {
    const char* nameEn;
    const char* nameJa;
    Rgb accent, background, panel, text, ok, hold, lost, beacon;
};

enum class MapMode : int { Top = 0, Side = 1, ThreeD = 2 };

struct OverlaySettings {
    int theme = 0;                 // index into Themes(); 0 = Dark Blue
    std::string lang = "en";       // "en" or "ja"
    MapMode mapMode = MapMode::ThreeD;
    float camYaw = 0.7f, camPitch = 0.55f, camDistance = 6.0f; // 3D map orbit camera
    float camTargetX = 0, camTargetY = 1, camTargetZ = 0;        // the point it orbits around (panned by the user)
    bool logDriver = true, logOts = true, logFilter = true, logUi = true;
    bool desktopWindow = true;
    bool modelOnMap = true; // the body model's skeleton on the map
    bool modelInVr = false; // and as world overlays around the user (debugging)
};

int ThemeCount();
const Theme& GetTheme(int i); // out-of-range -> the default theme

// Colors derived from a theme, as "#rrggbb" / "rgba(...)" strings keyed by the ${name} tokens used in
// app.rcss and app.rml (bg, panel, text, dim, line, button, accent, accentText, onAccent, ...).
std::map<std::string, std::string> ThemeTokens(const Theme& t);
// Same derivations as ThemeTokens, for code that draws directly (lane graph, map).
Rgb Mix(const Rgb& a, const Rgb& b, float t);
bool IsLight(const Theme& t);
Rgb AccentText(const Theme& t);

std::filesystem::path OverlaySettingsPath();
OverlaySettings LoadOverlaySettings();            // defaults when the file is missing or broken
bool SaveOverlaySettings(const OverlaySettings& s);

} // namespace ots
