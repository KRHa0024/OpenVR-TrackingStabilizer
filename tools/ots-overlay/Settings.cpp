#include "Settings.h"

#include "Json.h"

#include <windows.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <sstream>

namespace ots {
namespace {

constexpr Rgb Hex(unsigned v) { return {((v >> 16) & 255) / 255.0f, ((v >> 8) & 255) / 255.0f, (v & 255) / 255.0f}; }

// Status colors avoid each theme's accent hue (Crimson uses orange for "lost", for example).
const Theme kThemes[] = {
    {"Dark Blue", "ダーク・ブルー", Hex(0x408ce6), Hex(0x121419), Hex(0x1c2129), Hex(0xebedf2), Hex(0x59d973), Hex(0xff9e33), Hex(0xf25959), Hex(0x73b3ff)},
    {"Crimson", "クリムゾン", Hex(0xee0040), Hex(0x140d10), Hex(0x1f1519), Hex(0xf6edf0), Hex(0x3ddc97), Hex(0xffd166), Hex(0xff8a3d), Hex(0x8ab4ff)},
    {"Dark Purple", "ダーク・パープル", Hex(0x9e66f2), Hex(0x14121c), Hex(0x211c2e), Hex(0xf0ebf7), Hex(0x66e099), Hex(0xffa64d), Hex(0xff6180), Hex(0xbf99ff)},
    {"Light", "ライト", Hex(0x2160c4), Hex(0xedf0f5), Hex(0xffffff), Hex(0x1a1f26), Hex(0x13843a), Hex(0xb85a00), Hex(0xc41f1f), Hex(0x1a5bc4)},
};

float Luminance(const Rgb& c) {
    auto lin = [](float v) { return v <= 0.03928f ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f); };
    return 0.2126f * lin(c.r) + 0.7152f * lin(c.g) + 0.0722f * lin(c.b);
}

std::string HexStr(const Rgb& c) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%02x%02x%02x", int(std::lround(c.r * 255)), int(std::lround(c.g * 255)), int(std::lround(c.b * 255)));
    return buf;
}

std::string Rgba(const Rgb& c, float a) {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "rgba(%d,%d,%d,%d)", int(std::lround(c.r * 255)), int(std::lround(c.g * 255)), int(std::lround(c.b * 255)),
                  int(std::lround(a * 255)));
    return buf;
}

} // namespace

int ThemeCount() { return int(std::size(kThemes)); }
const Theme& GetTheme(int i) { return kThemes[i >= 0 && i < ThemeCount() ? i : 0]; }

Rgb Mix(const Rgb& a, const Rgb& b, float t) { return {a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t}; }
bool IsLight(const Theme& t) { return t.background.r + t.background.g + t.background.b > 1.5f; }
// Headings in the accent color, lifted toward white on dark themes so they stay readable.
Rgb AccentText(const Theme& t) { return IsLight(t) ? t.accent : Mix(t.accent, {1, 1, 1}, 0.28f); }

std::map<std::string, std::string> ThemeTokens(const Theme& t) {
    const bool light = IsLight(t);
    std::map<std::string, std::string> m;
    m["bg"] = HexStr(t.background);
    m["panel"] = HexStr(t.panel);
    m["text"] = HexStr(t.text);
    m["dim"] = HexStr(Mix(t.text, t.background, 0.42f));
    m["line"] = HexStr(Mix(t.panel, t.text, 0.14f));
    m["button"] = HexStr(Mix(t.panel, t.accent, light ? 0.12f : 0.28f));
    m["rowAlt"] = HexStr(Mix(t.panel, t.text, 0.035f));
    m["accent"] = HexStr(t.accent);
    m["accentText"] = HexStr(AccentText(t));
    m["onAccent"] = Luminance(t.accent) < 0.35f ? "#ffffff" : "#111111";
    m["accentSoft"] = Rgba(t.accent, 0.22f);
    m["accentSoftSolid"] = HexStr(Mix(t.panel, t.accent, 0.35f));
    m["ok"] = HexStr(t.ok);
    m["hold"] = HexStr(t.hold);
    m["lost"] = HexStr(t.lost);
    m["beacon"] = HexStr(t.beacon);
    m["okSoft"] = Rgba(t.ok, 0.16f);
    m["holdSoft"] = Rgba(t.hold, 0.16f);
    m["lostSoft"] = Rgba(t.lost, 0.16f);
    m["holdTint"] = Rgba(t.hold, 0.10f);
    m["lostTint"] = Rgba(t.lost, 0.10f);
    m["okEdge"] = Rgba(t.ok, 0.55f);
    m["holdEdge"] = Rgba(t.hold, 0.70f);
    return m;
}

std::filesystem::path OverlaySettingsPath() {
    const wchar_t* local = _wgetenv(L"LOCALAPPDATA");
    std::filesystem::path base = local ? std::filesystem::path(local) : std::filesystem::temp_directory_path();
    return base / L"OpenVR-TrackingStabilizer" / L"overlay.json";
}

OverlaySettings LoadOverlaySettings() {
    OverlaySettings s;
    std::ifstream f(OverlaySettingsPath(), std::ios::binary);
    if (!f) return s;
    std::stringstream ss;
    ss << f.rdbuf();
    const Json j = Json::Parse(ss.str());
    if (j.type() != Json::Type::Object) return s;
    const int theme = int(j["theme"].Num(0));
    s.theme = theme >= 0 && theme < ThemeCount() ? theme : 0;
    if (j["lang"].Str() == "ja" || j["lang"].Str() == "en") s.lang = j["lang"].Str();
    const int mode = int(j["mapMode"].Num(int(s.mapMode)));
    if (mode >= 0 && mode <= 2) s.mapMode = MapMode(mode);
    s.camYaw = float(j["camYaw"].Num(s.camYaw));
    s.camPitch = float(j["camPitch"].Num(s.camPitch));
    s.camDistance = float(j["camDistance"].Num(s.camDistance));
    s.camTargetX = float(j["camTargetX"].Num(s.camTargetX));
    s.camTargetY = float(j["camTargetY"].Num(s.camTargetY));
    s.camTargetZ = float(j["camTargetZ"].Num(s.camTargetZ));
    s.logDriver = j["logDriver"].Bool(s.logDriver);
    s.logOts = j["logOts"].Bool(s.logOts);
    s.logFilter = j["logFilter"].Bool(s.logFilter);
    s.logUi = j["logUi"].Bool(s.logUi);
    s.desktopWindow = j["desktopWindow"].Bool(s.desktopWindow);
    s.modelOnMap = j["modelOnMap"].Bool(s.modelOnMap);
    s.modelInVr = j["modelInVr"].Bool(s.modelInVr);
    return s;
}

bool SaveOverlaySettings(const OverlaySettings& s) {
    auto b = [](bool v) { return v ? "true" : "false"; };
    std::ostringstream o;
    o << "{\n  \"theme\":" << s.theme << ",\n  \"lang\":\"" << s.lang << "\",\n  \"mapMode\":" << int(s.mapMode)
      << ",\n  \"camYaw\":" << s.camYaw << ",\n  \"camPitch\":" << s.camPitch << ",\n  \"camDistance\":" << s.camDistance
      << ",\n  \"camTargetX\":" << s.camTargetX << ",\n  \"camTargetY\":" << s.camTargetY << ",\n  \"camTargetZ\":" << s.camTargetZ
      << ",\n  \"logDriver\":" << b(s.logDriver) << ",\n  \"logOts\":" << b(s.logOts) << ",\n  \"logFilter\":" << b(s.logFilter)
      << ",\n  \"logUi\":" << b(s.logUi) << ",\n  \"desktopWindow\":" << b(s.desktopWindow)
      << ",\n  \"modelOnMap\":" << b(s.modelOnMap) << ",\n  \"modelInVr\":" << b(s.modelInVr) << "\n}\n";

    const auto path = OverlaySettingsPath();
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    // Write to a temp file and swap, so a crash never leaves a half-written settings file.
    const auto tmp = path.wstring() + L".tmp";
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f << o.str();
        if (!f) return false;
    }
    return MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING) != 0;
}

} // namespace ots
