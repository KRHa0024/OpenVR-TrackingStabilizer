#include "UiApp.h"

#include "Gfx.h"
#include "UiElements.h"
#include "UiText.h"

#include <RmlUi/Core.h>

#include <openvr.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>

#ifndef OTS_VERSION
#define OTS_VERSION "0.0.0"
#endif

namespace ots {
namespace {

using Clock = std::chrono::steady_clock;

constexpr const char* kGithubUrl = "https://github.com/KRHa0024";
constexpr int kLogLines = 200;

// Device-specific steamvr.vrsettings keys and their slider ranges, in display order.
struct ParamDef {
    const char* key;   // pipe / vrsettings key
    const char* text;  // UiText key prefix
    const char* group; // UiText key of the heading shown above the first slider of a group
    float min, max, step, def;
    const char* unit;  // "%", "ms" (shown in seconds), "cm"
};
const ParamDef kParams[] = {
    {"smoothing", "pSmoothing", "groupJitter", 0, 100, 5, 0, "%"},
    {"probationMs", "pProbation", "groupLoss", 0, 1000, 10, 400, "ms"},
    {"shortProbationMs", "pShortProbation", nullptr, 0, 500, 10, 100, "ms"},
    {"blendMs", "pBlend", nullptr, 0, 500, 10, 200, "ms"},
    {"stepLimitCm", "pStepLimit", "groupWarp", 1, 20, 0.5f, 4.5f, "cm"},
    {"stepBlendMs", "pStepBlend", nullptr, 0, 500, 10, 150, "ms"},
    {"handShoulderWidthCm", "pHandShoulder", "groupHand", 20, 60, 1, 36, "cm"},
    {"handArmReachCm", "pHandReach", nullptr, 30, 120, 1, 70, "cm"},
    {"handMaxPredictionMs", "pHandDuration", nullptr, 100, 3000, 100, 3000, "ms"},
};

float ConfigValue(const FilterConfig& f, const std::string& key) {
    if (key == "smoothing") return f.smoothing;
    if (key == "probationMs") return f.probationMs;
    if (key == "shortProbationMs") return f.shortProbationMs;
    if (key == "blendMs") return f.blendMs;
    if (key == "stepLimitCm") return f.stepLimitCm;
    if (key == "stepBlendMs") return f.stepBlendMs;
    if (key == "handShoulderWidthCm") return f.handShoulderWidthCm;
    if (key == "handArmReachCm") return f.handArmReachCm;
    if (key == "handMaxPredictionMs") return f.handMaxPredictionMs;
    return 0;
}

std::string Num(double v, int decimals) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return buf;
}

std::string Upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string JoinCsv(const std::vector<std::string>& v) {
    std::string s;
    for (const auto& x : v) s += (s.empty() ? "" : ",") + x;
    return s;
}

bool Contains(const std::vector<std::string>& v, const std::string& x) { return std::find(v.begin(), v.end(), x) != v.end(); }
void Remove(std::vector<std::string>& v, const std::string& x) { v.erase(std::remove(v.begin(), v.end(), x), v.end()); }
void Add(std::vector<std::string>& v, const std::string& x) {
    if (!Contains(v, x)) v.push_back(x);
}

std::string ReadFile(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string ReplaceTokens(std::string s, const std::map<std::string, std::string>& tokens) {
    for (const auto& [k, v] : tokens) {
        const std::string t = "${" + k + "}";
        for (size_t p; (p = s.find(t)) != std::string::npos;) s.replace(p, t.size(), v);
    }
    return s;
}

bool IsTracked(int cls) {
    return cls == vr::TrackedDeviceClass_HMD || cls == vr::TrackedDeviceClass_Controller || cls == vr::TrackedDeviceClass_GenericTracker;
}

const char* IconFor(int cls) {
    return cls == vr::TrackedDeviceClass_HMD ? "icons/hmd.svg" : cls == vr::TrackedDeviceClass_Controller ? "icons/controller.svg" : "icons/tracker.svg";
}

// HMD first, then controllers (left before right), then trackers, each by device index.
std::vector<const DeviceInfo*> TrackedDevices(const StateData& s) {
    std::vector<const DeviceInfo*> out;
    for (const DeviceInfo& d : s.devices)
        if (IsTracked(d.deviceClass) && d.connected) out.push_back(&d);
    auto rank = [](const DeviceInfo* d) {
        return d->deviceClass == vr::TrackedDeviceClass_HMD ? 0 : d->deviceClass == vr::TrackedDeviceClass_Controller ? 1 : 2;
    };
    std::stable_sort(out.begin(), out.end(), [&](auto* a, auto* b) {
        if (rank(a) != rank(b)) return rank(a) < rank(b);
        if (a->deviceClass == vr::TrackedDeviceClass_Controller && a->role != b->role) return a->role < b->role;
        return a->index < b->index;
    });
    return out;
}

char StateChar(const std::string& filterState) {
    if (filterState == "HOLD") return 'H';
    if (filterState == "PROBATION") return 'P';
    if (filterState == "BLEND") return 'B';
    return 'T';
}

const char* StateKey(char c) {
    switch (c) {
    case 'H': return "stHold";
    case 'P': return "stVerify";
    case 'B': return "stReturn";
    case 'L': return "stLost";
    default: return "stTracking";
    }
}

const char* StateClass(char c) { return c == 'T' ? "s-ok" : c == 'L' ? "s-lost" : "s-hold"; }

Vec3 Rotate(const float m[3][4], const Vec3& v) {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
}
Vec3 Transform(const float m[3][4], const Vec3& p) {
    const Vec3 r = Rotate(m, p);
    return {r.x + m[0][3], r.y + m[1][3], r.z + m[2][3]};
}

std::string Css(const Rgb& c) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "#%02x%02x%02x", int(c.r * 255 + 0.5f), int(c.g * 255 + 0.5f), int(c.b * 255 + 0.5f));
    return buf;
}

} // namespace

UiApp::UiApp(Gfx& gfx, OverlaySettings& settings, UiActions actions, std::filesystem::path uiDir)
    : gfx_(gfx), settings_(settings), act_(std::move(actions)), uiDir_(std::move(uiDir)) {}

UiApp::~UiApp() { Shutdown(); }

void UiApp::Shutdown() {
    if (context_) Rml::RemoveContext(context_->GetName());
    context_ = nullptr;
}

const std::string& UiApp::T(const char* key) const { return Text(settings_.lang, key); }

bool UiApp::Init() {
    RegisterUiElements();
    context_ = Rml::CreateContext("main", Rml::Vector2i(gfx_.Width(), gfx_.Height()));
    if (!context_) return false;
    version_ = OTS_VERSION;
    const std::u8string path = OverlaySettingsPath().u8string();
    settingsPath_.assign(path.begin(), path.end());
    desktopWindow_ = settings_.desktopWindow;
    BindModel();
    return LoadDocument();
}

void UiApp::BindModel() {
    Rml::DataModelConstructor c = context_->CreateDataModel("app");
    if (auto s = c.RegisterStruct<Lane>()) {
        s.RegisterMember("index", &Lane::index);
        s.RegisterMember("name", &Lane::name);
        s.RegisterMember("serial", &Lane::serial);
        s.RegisterMember("icon", &Lane::icon);
        s.RegisterMember("state", &Lane::state);
        s.RegisterMember("stateClass", &Lane::stateClass);
        s.RegisterMember("drift", &Lane::drift);
        s.RegisterMember("tone", &Lane::tone);
        s.RegisterMember("on", &Lane::on);
        s.RegisterMember("locked", &Lane::locked);
    }
    if (auto s = c.RegisterStruct<LogLine>()) {
        s.RegisterMember("time", &LogLine::time);
        s.RegisterMember("text", &LogLine::text);
        s.RegisterMember("cls", &LogLine::cls);
    }
    if (auto s = c.RegisterStruct<LogKind>()) {
        s.RegisterMember("id", &LogKind::id);
        s.RegisterMember("label", &LogKind::label);
        s.RegisterMember("help", &LogKind::help);
        s.RegisterMember("on", &LogKind::on);
    }
    if (auto s = c.RegisterStruct<Device>()) {
        s.RegisterMember("index", &Device::index);
        s.RegisterMember("model", &Device::model);
        s.RegisterMember("serial", &Device::serial);
        s.RegisterMember("icon", &Device::icon);
        s.RegisterMember("batt", &Device::batt);
        s.RegisterMember("battClass", &Device::battClass);
        s.RegisterMember("battIcon", &Device::battIcon);
        s.RegisterMember("state", &Device::state);
        s.RegisterMember("stateClass", &Device::stateClass);
        s.RegisterMember("sub", &Device::sub);
        s.RegisterMember("holds", &Device::holds);
        s.RegisterMember("hasBatt", &Device::hasBatt);
        s.RegisterMember("on", &Device::on);
        s.RegisterMember("locked", &Device::locked);
        s.RegisterMember("alt", &Device::alt);
    }
    if (auto s = c.RegisterStruct<Base>()) {
        s.RegisterMember("serial", &Base::serial);
        s.RegisterMember("ch", &Base::ch);
        s.RegisterMember("height", &Base::height);
        s.RegisterMember("tilt", &Base::tilt);
        s.RegisterMember("state", &Base::state);
        s.RegisterMember("stateClass", &Base::stateClass);
        s.RegisterMember("alt", &Base::alt);
    }
    if (auto s = c.RegisterStruct<Param>()) {
        s.RegisterMember("key", &Param::key);
        s.RegisterMember("label", &Param::label);
        s.RegisterMember("help", &Param::help);
        s.RegisterMember("group", &Param::group);
        s.RegisterMember("display", &Param::display);
        s.RegisterMember("value", &Param::value);
        s.RegisterMember("slider", &Param::slider);
        s.RegisterMember("min", &Param::min);
        s.RegisterMember("max", &Param::max);
        s.RegisterMember("step", &Param::step);
    }
    if (auto s = c.RegisterStruct<ThemeCard>()) {
        s.RegisterMember("index", &ThemeCard::index);
        s.RegisterMember("name", &ThemeCard::name);
        s.RegisterMember("bg", &ThemeCard::bg);
        s.RegisterMember("panel", &ThemeCard::panel);
        s.RegisterMember("accent", &ThemeCard::accent);
        s.RegisterMember("text", &ThemeCard::text);
        s.RegisterMember("ok", &ThemeCard::ok);
        s.RegisterMember("hold", &ThemeCard::hold);
        s.RegisterMember("lost", &ThemeCard::lost);
        s.RegisterMember("edge", &ThemeCard::edge);
        s.RegisterMember("active", &ThemeCard::active);
        s.RegisterMember("isDefault", &ThemeCard::isDefault);
    }
    c.RegisterArray<std::vector<Lane>>();
    c.RegisterArray<std::vector<LogLine>>();
    c.RegisterArray<std::vector<LogKind>>();
    c.RegisterArray<std::vector<Device>>();
    c.RegisterArray<std::vector<Base>>();
    c.RegisterArray<std::vector<Param>>();
    c.RegisterArray<std::vector<ThemeCard>>();

    c.Bind("view", &view_);
    c.Bind("lang", &lang_);
    c.Bind("version", &version_);
    c.Bind("warning", &warning_);
    c.Bind("heroLine", &heroLine_);
    c.Bind("heroWord", &heroWord_);
    c.Bind("sessionText", &sessionText_);
    c.Bind("lanesEmpty", &lanesEmpty_);
    c.Bind("mapMode", &mapMode_);
    c.Bind("toast", &toast_);
    c.Bind("settingsPath", &settingsPath_);
    c.Bind("recordLabel", &recordLabel_);
    c.Bind("recordIcon", &recordIcon_);
    c.Bind("logFilterCount", &logFilterCount_);
    c.Bind("mapPan", &mapPan_);
    c.Bind("filterOn", &filterOn_);
    c.Bind("recording", &recording_);
    c.Bind("logMenu", &logMenu_);
    c.Bind("logFiltered", &logFiltered_);
    c.Bind("allowHmd", &allowHmd_);
    c.Bind("bodyEnabled", &bodyEnabled_);
    c.Bind("handEnabled", &handEnabled_);
    c.Bind("handStatus", &handStatus_);
    c.Bind("imuEnabled", &imuEnabled_);
    c.Bind("imuStatus", &imuStatus_);
    c.Bind("bodyStatus", &bodyStatus_);
    c.Bind("bodyWaist", &bodyWaist_);
    c.Bind("bodyHmd", &bodyHmd_);
    c.Bind("bodyWaistChoices", &bodyWaistChoices_);
    c.Bind("bodyHmdChoices", &bodyHmdChoices_);
    c.Bind("desktopWindow", &desktopWindow_);
    c.Bind("modelOnMap", &modelOnMap_);
    c.Bind("modelInVr", &modelInVr_);
    c.Bind("deviceCount", &deviceCount_);
    c.Bind("holdingCount", &holdingCount_);
    c.Bind("lanes", &lanes_);
    c.Bind("log", &log_);
    c.Bind("logKinds", &logKinds_);
    c.Bind("devices", &devices_);
    c.Bind("bases", &bases_);
    c.Bind("params", &params_);
    c.Bind("themes", &themes_);

    using Args = const Rml::VariantList&;
    auto arg = [](Args a, size_t i) { return i < a.size() ? a[i] : Rml::Variant(); };
    c.BindEventCallback("setView", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) { SetView(arg(a, 0).Get<Rml::String>()); });
    c.BindEventCallback("toggleFilter", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        const bool on = !filterOn_;
        filterWanted_ = on;
        filterWantedAt_ = Clock::now();
        Set(filterOn_, on, "filterOn");
        act_.driver(on ? "filter on" : "filter off");
        SaveDriverSetting("filterEnabled", on ? "true" : "false");
        Toast(T(on ? "toastFilterOn" : "toastFilterOff"));
    });
    c.BindEventCallback("toggleDevice", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) { OnToggleDevice(arg(a, 0).Get<int>()); });
    c.BindEventCallback("resetCamera", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        UiScene& sc = Scene();
        sc.camYaw = 0.7f, sc.camPitch = 0.55f, sc.camDistance = 6.0f;
        sc.camTarget = Vec3{0, 1, 0};
        sc.cameraChanged = true;
    });
    c.BindEventCallback("setMapTool", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        Scene().mapPan = arg(a, 0).Get<Rml::String>() == "pan";
        Set(mapPan_, Scene().mapPan, "mapPan");
    });
    c.BindEventCallback("setMapMode", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        const std::string m = arg(a, 0).Get<Rml::String>();
        settings_.mapMode = m == "top" ? MapMode::Top : m == "side" ? MapMode::Side : MapMode::ThreeD;
        settingsDirty_ = true;
    });
    c.BindEventCallback("toggleLogMenu", [this](Rml::DataModelHandle, Rml::Event&, Args) { SetLogMenu(!logMenu_); });
    c.BindEventCallback("closeLogMenu", [this](Rml::DataModelHandle, Rml::Event&, Args) { SetLogMenu(false); });
    c.BindEventCallback("toggleLogKind", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        const std::string id = arg(a, 0).Get<Rml::String>();
        bool* flag = id == "driver" ? &settings_.logDriver : id == "ots" ? &settings_.logOts : id == "filter" ? &settings_.logFilter : &settings_.logUi;
        *flag = !*flag;
        settingsDirty_ = true;
    });
    c.BindEventCallback("toggleAllowHmd", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        const bool on = !allowHmd_;
        allowHmdWanted_ = on;
        allowHmdWantedAt_ = Clock::now();
        Set(allowHmd_, on, "allowHmd");
        act_.driver(std::string("set filterHmd ") + (on ? "true" : "false"));
        SaveDriverSetting("filterHmd", on ? "true" : "false");
        if (!on) { // the HMD switch goes back to off and stays locked
            for (const DeviceInfo& d : latestDevices_)
                if (d.deviceClass == vr::TrackedDeviceClass_HMD) Remove(include_, Upper(d.serial));
            SendDeviceLists();
        }
    });
    c.BindEventCallback("toggleBody", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        const bool on = !bodyEnabled_;
        bodyWanted_ = on; bodyWantedAt_ = Clock::now();
        Set(bodyEnabled_,on,"bodyEnabled");
        act_.driver(std::string("set bodyEnabled ") + (on ? "true" : "false"));
        SaveDriverSetting("bodyEnabled",on ? "true" : "false");
    });
    c.BindEventCallback("setBodyDevice", [this,arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        const bool head = arg(a,0).Get<Rml::String>() == "head";
        const int index = arg(a,1).Get<int>();
        std::string serial;
        for (const DeviceInfo& d : latestDevices_)
            if (int(d.index) == index && d.deviceClass == (head ? vr::TrackedDeviceClass_HMD : vr::TrackedDeviceClass_GenericTracker)) serial = Upper(d.serial);
        if (index != -1 && serial.empty()) return;
        const std::string key = head ? "bodyHmdSerial" : "bodyWaistSerial";
        act_.driver("set " + key + " " + (serial.empty() ? "-" : serial));
        SaveDriverSetting(key,serial);
    });
    c.BindEventCallback("toggleImu", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        const bool on = !imuEnabled_;
        imuWanted_ = on; imuWantedAt_ = Clock::now();
        Set(imuEnabled_,on,"imuEnabled");
        act_.driver(std::string("set imuEnabled ")+(on ? "true" : "false"));
        SaveDriverSetting("imuEnabled",on ? "true" : "false");
        Toast(T(on ? "toastImuOn" : "toastImuOff"));
    });
    c.BindEventCallback("toggleHand", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        const bool on = !handEnabled_;
        handWanted_ = on; handWantedAt_ = Clock::now();
        Set(handEnabled_,on,"handEnabled");
        act_.driver(std::string("set handEnabled ")+(on ? "true" : "false"));
        SaveDriverSetting("handEnabled",on ? "true" : "false");
    });
    // The sliders all run 0..1000 (fixed in app.rml): RmlUi fires "change" whenever the value attribute
    // is set, so per-parameter min/max attributes would clamp and re-send values while loading.
    c.BindEventCallback("setParam", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        const std::string key = arg(a, 0).Get<Rml::String>();
        for (const Param& p : params_) {
            if (p.key != key) continue;
            float v = p.min + arg(a, 1).Get<float>() / 1000 * (p.max - p.min);
            v = std::round(v / p.step) * p.step;
            if (std::fabs(v - p.value) >= p.step / 2) OnSetParam(key, v); // ignore our own attribute updates
        }
    });
    c.BindEventCallback("resetParams", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        for (const ParamDef& p : kParams) OnSetParam(p.key, p.def);
        Toast(T("toastDefaults"));
    });
    c.BindEventCallback("toggleRecording", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        act_.driver(recording_ ? "stop" : "start overlay");
        Toast(T(recording_ ? "toastRecStop" : "toastRecStart"));
    });
    c.BindEventCallback("addMarker", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        act_.driver("mark marker from the overlay");
        Toast(T("toastMarker"));
    });
    c.BindEventCallback("setLang", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        const std::string l = arg(a, 0).Get<Rml::String>();
        if (l == settings_.lang || (l != "en" && l != "ja")) return;
        settings_.lang = l;
        settingsDirty_ = reload_ = true;
    });
    c.BindEventCallback("setTheme", [this, arg](Rml::DataModelHandle, Rml::Event&, Args a) {
        const int t = arg(a, 0).Get<int>();
        if (t == settings_.theme || t < 0 || t >= ThemeCount()) return;
        settings_.theme = t;
        settingsDirty_ = reload_ = true;
    });
    c.BindEventCallback("toggleDesktop", [this](Rml::DataModelHandle, Rml::Event&, Args) { SetDesktopWindow(!settings_.desktopWindow); });
    c.BindEventCallback("toggleModelMap", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        settings_.modelOnMap = !settings_.modelOnMap;
        settingsDirty_ = true;
    });
    c.BindEventCallback("toggleModelVr", [this](Rml::DataModelHandle, Rml::Event&, Args) {
        settings_.modelInVr = !settings_.modelInVr;
        settingsDirty_ = true;
    });
    c.BindEventCallback("openGithub", [this](Rml::DataModelHandle, Rml::Event&, Args) { act_.openUrl(kGithubUrl); });
    model_ = c.GetModelHandle();
}

bool UiApp::LoadDocument() {
    if (document_) {
        document_->Close();
        document_ = nullptr;
        context_->Update(); // actually releases the old document
    }
    const auto tokens = ThemeTokens(GetTheme(settings_.theme));
    const std::string rcss = ReplaceTokens(ReadFile(uiDir_ / "app.rcss"), tokens);
    std::string rml = ReplaceTokens(TranslateRml(ReadFile(uiDir_ / "app.rml"), settings_.lang), tokens);
    // The stylesheet is inlined so it carries the theme colors.
    const std::string link = "<link type=\"text/rcss\" href=\"app.rcss\"/>";
    if (const size_t p = rml.find(link); p != std::string::npos) rml.replace(p, link.size(), "<style>" + rcss + "</style>");
    // Relative URL (resolved by Gfx's file interface) so images and icons load from the ui folder.
    document_ = context_->LoadDocumentFromMemory(rml, "ui/app.rml");
    if (!document_) return false;
    document_->Show();
    lang_ = settings_.lang;
    model_.DirtyAllVariables();
    return true;
}

void UiApp::Toast(const std::string& text, bool log) {
    Set(toast_, Rml::String(text), "toast");
    toastUntil_ = Clock::now() + std::chrono::milliseconds(2600);
    if (log && act_.log) act_.log(text);
}

void UiApp::SetView(const std::string& view) {
    if (view != "main" && view != "devices" && view != "settings") return;
    Set(view_, Rml::String(view), "view");
    SetLogMenu(false);
}

void UiApp::SetLogMenu(bool open) { Set(logMenu_, open, "logMenu"); }

void UiApp::SetDesktopWindow(bool show) {
    settings_.desktopWindow = show;
    settingsDirty_ = true;
    Set(desktopWindow_, show, "desktopWindow");
}

void UiApp::SetScale(float scale) {
    scale_ = scale;
    context_->SetDimensions(Rml::Vector2i(gfx_.Width(), gfx_.Height()));
    context_->SetDensityIndependentPixelRatio(scale);
}

void UiApp::MouseMove(float x, float y) { context_->ProcessMouseMove(int(std::lround(x * scale_)), int(std::lround(y * scale_)), 0); }
void UiApp::MouseButton(int button, bool down) {
    if (down) context_->ProcessMouseButtonDown(button, 0);
    else context_->ProcessMouseButtonUp(button, 0);
}
void UiApp::MouseWheel(float linesDown) { context_->ProcessMouseWheel(Rml::Vector2f(0, linesDown), 0); }
void UiApp::MouseLeave() { context_->ProcessMouseLeave(); }

void UiApp::SaveDriverSetting(const std::string& key, const std::string& value) {
    pendingSave_[key] = {value, Clock::now() + std::chrono::milliseconds(800)};
}

void UiApp::SendDeviceLists() {
    if (!listsLoaded_) return; // saving now would replace the user's lists with empty ones
    act_.driver("set filterIncludeSerials " + (include_.empty() ? std::string("-") : JoinCsv(include_)));
    act_.driver("set filterExcludeSerials " + (exclude_.empty() ? std::string("-") : JoinCsv(exclude_)));
    SaveDriverSetting("filterIncludeSerials", JoinCsv(include_));
    SaveDriverSetting("filterExcludeSerials", JoinCsv(exclude_));
}

void UiApp::OnToggleDevice(int index) {
    if (!listsLoaded_) return;
    const DeviceInfo* dev = nullptr;
    for (const DeviceInfo& d : latestDevices_)
        if (int(d.index) == index) dev = &d;
    if (!dev || dev->serial.empty()) return;
    const bool hmd = dev->deviceClass == vr::TrackedDeviceClass_HMD;
    bool current = false, locked = false;
    for (const Lane& l : lanes_)
        if (l.index == index) current = l.on, locked = l.locked;
    if (locked) return;
    const bool on = !current;
    const std::string serial = Upper(dev->serial);
    if (on) {
        Remove(exclude_, serial);
        if (hmd) Add(include_, serial);
    } else {
        Remove(include_, serial);
        Add(exclude_, serial);
    }
    deviceOverride_[serial] = {on, Clock::now() + std::chrono::seconds(2)};
    SendDeviceLists();
    Toast(Fmt(settings_.lang, on ? "toastDeviceOn" : "toastDeviceOff", dev->model + " (" + dev->serial + ")"));
}

void UiApp::OnSetParam(const std::string& key, float value) {
    const ParamDef* def = nullptr;
    for (const ParamDef& p : kParams)
        if (key == p.key) def = &p;
    if (!def) return;
    value = std::clamp(value, def->min, def->max);
    paramEdit_[key] = value;
    paramEditAt_ = Clock::now();
    const std::string v = Num(value, def->step < 1 ? 1 : 0);
    act_.driver("set " + key + " " + v);
    SaveDriverSetting(key, v);
}

// ---------------------------------------------------------------------------------------------

void UiApp::Update(const StateData& s) {
    latestDevices_ = s.devices;
    const auto now = Clock::now();
    if (reload_) {
        reload_ = false;
        LoadDocument();
    }
    if (!toast_.empty() && now > toastUntil_) Set(toast_, Rml::String(), "toast");
    if (!filterWanted_ || now - filterWantedAt_ > std::chrono::seconds(2)) filterWanted_.reset();
    if (!allowHmdWanted_ || now - allowHmdWantedAt_ > std::chrono::seconds(2)) allowHmdWanted_.reset();
    for (auto it = deviceOverride_.begin(); it != deviceOverride_.end();)
        it = now > it->second.second ? deviceOverride_.erase(it) : std::next(it);
    // Our include/exclude lists follow the driver unless we changed them a moment ago.
    if (s.filter.valid && deviceOverride_.empty()) {
        include_ = s.filter.includeSerials;
        exclude_ = s.filter.excludeSerials;
        listsLoaded_ = true;
    }

    const std::string warning = !s.vrConnected ? T("warnNoVr") : !s.driverConnected ? T("warnNoDriver") : "";
    Set(warning_, Rml::String(warning), "warning");
    Set(recording_, s.recording, "recording");
    Set(filterOn_, filterWanted_ ? *filterWanted_ : (s.filter.valid && s.filter.enabled), "filterOn");
    Set(allowHmd_, allowHmdWanted_ ? *allowHmdWanted_ : s.filter.hmd, "allowHmd");
    Set(heroWord_, Rml::String(T(filterOn_ ? "on" : "off")), "heroWord");
    const char* mode = settings_.mapMode == MapMode::Top ? "top" : settings_.mapMode == MapMode::Side ? "side" : "3d";
    Set(mapMode_, Rml::String(mode), "mapMode");
    Set(desktopWindow_, settings_.desktopWindow, "desktopWindow");
    Set(modelOnMap_, settings_.modelOnMap, "modelOnMap");
    Set(modelInVr_, settings_.modelInVr, "modelInVr");

    BuildMain(s);
    if (view_ == "devices") BuildDevices(s);
    if (view_ == "settings") BuildSettings(s);
    BuildScene(s);

    UiScene& sc = Scene();
    if (sc.cameraChanged) {
        sc.cameraChanged = false;
        settings_.camYaw = sc.camYaw, settings_.camPitch = sc.camPitch, settings_.camDistance = sc.camDistance;
        settings_.camTargetX = sc.camTarget.x, settings_.camTargetY = sc.camTarget.y, settings_.camTargetZ = sc.camTarget.z;
        cameraSaveAt_ = now + std::chrono::seconds(2);
    }
    context_->Update();
}

void UiApp::SaveDue(bool all) {
    const auto now = Clock::now();
    for (auto it = pendingSave_.begin(); it != pendingSave_.end();) {
        if (!all && now < it->second.second) {
            ++it;
            continue;
        }
        if (act_.saveSetting && !act_.saveSetting(it->first, it->second.first)) Toast(T("toastSaveFailed"));
        it = pendingSave_.erase(it);
    }
    if (cameraSaveAt_ != Clock::time_point{} && (all || now > cameraSaveAt_)) {
        cameraSaveAt_ = {};
        settingsDirty_ = true;
    }
    if (settingsDirty_) {
        settingsDirty_ = false;
        SaveOverlaySettings(settings_);
    }
}

void UiApp::BuildMain(const StateData& s) {
    const auto devs = TrackedDevices(s);
    const std::string& lang = settings_.lang;
    int targeted = 0, holding = 0, holds = 0;
    std::vector<Lane> lanes;
    for (size_t i = 0; i < devs.size(); ++i) {
        const DeviceInfo& d = *devs[i];
        const std::string serial = Upper(d.serial);
        auto fit = s.filterDevices.find(d.index);
        const FilterDeviceInfo* fd = fit != s.filterDevices.end() ? &fit->second : nullptr;
        const bool hmd = d.deviceClass == vr::TrackedDeviceClass_HMD;
        bool on = fd ? fd->mode == "filtered" : false;
        if (auto o = deviceOverride_.find(serial); o != deviceOverride_.end()) on = o->second.first;
        const bool corrected = on && filterOn_;
        char st = corrected && fd ? StateChar(fd->state) : 'T';
        if (!corrected && (!d.poseValid || d.trackingResult != vr::TrackingResult_Running_OK)) st = 'L';
        targeted += on;
        holding += corrected && st != 'T';
        if (fd) holds += fd->holds;

        Lane l;
        l.index = int(d.index);
        l.name = d.model.empty() ? d.serial : d.model;
        l.serial = d.serial;
        l.icon = IconFor(d.deviceClass);
        l.on = on;
        l.locked = (hmd && !allowHmd_) || (!hmd && fd && !fd->lighthouse);
        l.state = on || st == 'L' ? T(StateKey(st)) : T("notTarget");
        l.stateClass = on || st == 'L' ? StateClass(st) : "s-dim";
        l.tone = st == 'T' ? "" : st == 'L' ? "lost" : "busy";
        auto tr = s.trace.find(d.serial);
        if (tr != s.trace.end() && !tr->second.empty()) {
            const TraceSample& last = tr->second.back();
            const float dx = last.raw.x - last.out.x, dy = last.raw.y - last.out.y, dz = last.raw.z - last.out.z;
            const float gap = std::sqrt(dx * dx + dy * dy + dz * dz) * 100;
            if (last.rawValid && gap > 0.5f) l.drift = Fmt(lang, "drift", Num(gap, 1));
        }
        lanes.push_back(std::move(l));
    }
    Set(lanes_, lanes, "lanes");
    Set(deviceCount_, int(devs.size()), "deviceCount");
    Set(holdingCount_, holding, "holdingCount");
    Set(heroLine_, Rml::String(filterOn_ ? Fmt(lang, "targetsLine", std::to_string(targeted), std::to_string(devs.size())) : T("heroOff")),
        "heroLine");
    Set(sessionText_, Rml::String(Fmt(lang, "session", std::to_string(holds))), "sessionText");
    Set(lanesEmpty_, Rml::String(T(s.driverConnected && !s.traceSupported ? "lanesNoTrace" : "lanesEmpty")), "lanesEmpty");

    // Log, newest first.
    const std::map<std::string, bool> show = {{"driver", settings_.logDriver}, {"ots", settings_.logOts}, {"filter", settings_.logFilter},
                                              {"ui", settings_.logUi}};
    std::vector<LogLine> log;
    for (auto it = s.log.rbegin(); it != s.log.rend() && int(log.size()) < kLogLines; ++it) {
        auto sh = show.find(it->kind);
        if (sh == show.end() || !sh->second) continue;
        const time_t secs = time_t(it->unixMs / 1000);
        tm local{};
        localtime_s(&local, &secs);
        char hms[16];
        std::strftime(hms, sizeof(hms), "%H:%M:%S", &local);
        log.push_back({hms, it->text, "k-" + it->kind});
    }
    Set(log_, log, "log");
    std::vector<LogKind> kinds;
    int onCount = 0;
    for (auto [id, key] : {std::pair{"driver", "logDriver"}, {"ots", "logOts"}, {"filter", "logFilter"}, {"ui", "logUi"}}) {
        const bool on = show.at(id);
        onCount += on;
        kinds.push_back({id, T(key), T((std::string(key) + "Help").c_str()), on});
    }
    Set(logKinds_, kinds, "logKinds");
    Set(logFiltered_, onCount < 4, "logFiltered");
    Set(logFilterCount_, Rml::String(std::to_string(onCount) + " / 4"), "logFilterCount");
}

void UiApp::BuildDevices(const StateData& s) {
    const std::string& lang = settings_.lang;
    std::vector<Device> devices;
    for (const DeviceInfo* dp : TrackedDevices(s)) {
        const DeviceInfo& d = *dp;
        const Lane* lane = nullptr;
        for (const Lane& l : lanes_)
            if (l.index == int(d.index)) lane = &l;
        auto fit = s.filterDevices.find(d.index);
        Device v;
        v.index = int(d.index);
        v.model = d.model.empty() ? d.serial : d.model;
        v.serial = d.serial;
        v.icon = IconFor(d.deviceClass);
        v.on = lane && lane->on;
        v.locked = lane && lane->locked;
        v.hasBatt = d.hasBattery;
        v.batt = d.hasBattery ? std::to_string(int(std::lround(d.battery * 100))) + "%" : "—";
        // Charging: a battery with a bolt, in the "fine" colour whatever the level.
        v.battClass = !d.hasBattery ? "s-dim" : d.charging ? "s-ok" : d.battery < 0.2f ? "s-lost" : d.battery < 0.4f ? "s-hold" : "";
        v.battIcon = d.charging ? "icons/battery-charging.svg" : "icons/battery.svg";
        v.state = lane ? lane->state : "";
        v.stateClass = lane ? lane->stateClass : "";
        if (fit != s.filterDevices.end() && (fit->second.state == "HOLD") && v.on) v.sub = Fmt(lang, "lostFor", Num(fit->second.stateSeconds, 1));
        else if (v.locked) v.sub = T(d.deviceClass == vr::TrackedDeviceClass_HMD ? "hmdLocked" : "notLighthouse");
        else if (!v.on) v.sub = T("notTarget");
        if (v.sub == v.state) v.sub.clear();
        v.holds = fit != s.filterDevices.end() && fit->second.mode == "filtered" ? std::to_string(fit->second.holds) : "—";
        v.alt = devices.size() % 2 == 1;
        devices.push_back(std::move(v));
    }
    Set(devices_, devices, "devices");

    std::vector<Base> bases;
    for (const DeviceInfo& d : s.devices) {
        if (d.deviceClass != vr::TrackedDeviceClass_TrackingReference) continue;
        Base b;
        b.serial = d.serial;
        b.ch = d.modeLabel.empty() ? "" : "ch" + d.modeLabel;
        b.height = Num(d.position.y, 2) + "m";
        b.tilt = std::to_string(int(std::lround(-std::asin(std::clamp(d.forward.y, -1.0f, 1.0f)) * 180 / 3.14159265))) + "°";
        b.state = T(d.poseValid ? "bsSeen" : "bsUnseen");
        b.stateClass = d.poseValid ? "s-ok" : "s-dim";
        b.alt = bases.size() % 2 == 1;
        bases.push_back(std::move(b));
    }
    Set(bases_, bases, "bases");
}

void UiApp::BuildSettings(const StateData& s) {
    const std::string& lang = settings_.lang;
    if (bodyWanted_ && Clock::now()-bodyWantedAt_ > std::chrono::seconds(2)) bodyWanted_.reset();
    Set(bodyEnabled_,bodyWanted_.value_or(s.filter.bodyEnabled),"bodyEnabled");
    if (imuWanted_ && Clock::now()-imuWantedAt_ > std::chrono::seconds(2)) imuWanted_.reset();
    Set(imuEnabled_,imuWanted_.value_or(s.filter.imuEnabled),"imuEnabled");
    {
        int ready = 0, aligning = 0, failed = 0, active = 0, error = -1;
        for (const auto& [index,f] : s.filterDevices) {
            (void)index;
            if (f.mode != "filtered") continue;
            if (f.imuAligned) ++ready;
            else if (f.imuOpen) ++aligning;
            else if (f.imuError > 0) ++failed, error = f.imuError;
            active += f.imuActive;
        }
        std::string text = !imuEnabled_ ? T("imuOff") : Fmt(lang, "imuReady", std::to_string(ready));
        if (imuEnabled_ && aligning) text += " " + Fmt(lang, "imuLearning", std::to_string(aligning));
        if (imuEnabled_ && failed) text += " " + Fmt(lang, "imuFailed", std::to_string(failed), std::to_string(error));
        if (imuEnabled_ && active) text += " " + Fmt(lang, "imuActiveNow", std::to_string(active));
        Set(imuStatus_,Rml::String(text),"imuStatus");
    }
    if (handWanted_ && Clock::now()-handWantedAt_ > std::chrono::seconds(2)) handWanted_.reset();
    Set(handEnabled_,handWanted_.value_or(s.filter.handEnabled),"handEnabled");
    int handsReady = 0, handsActive = 0, handsAssigned = 0;
    for (const auto& [index,f] : s.filterDevices) {
        (void)index;
        if (f.handSide == 0) continue;
        ++handsAssigned;
        handsReady += f.handReady; handsActive += f.handActive;
    }
    // The reference HMD: the chosen one, or the driver's automatic pick when there is only one HMD.
    const std::string refHmd = !s.filter.bodyHmdSerial.empty() ? s.filter.bodyHmdSerial : s.filter.bodyHmdAuto;
    const std::string handStatus = !handEnabled_ ? T("handOff") : refHmd.empty() ? T("handHeadNeeded") :
        handsAssigned == 0 ? T("handRolesNeeded") : handsActive > 0 ? T("handEstimating") : handsReady > 0 ? T("handReady") : T("handWaiting");
    Set(handStatus_,Rml::String(handStatus),"handStatus");
    Set(bodyWaist_,Rml::String(s.filter.bodyWaistSerial.empty() ? T("bodyUnset") : s.filter.bodyWaistSerial),"bodyWaist");
    Set(bodyHmd_,Rml::String(!s.filter.bodyHmdSerial.empty() ? s.filter.bodyHmdSerial :
                             !s.filter.bodyHmdAuto.empty() ? Fmt(lang,"bodyHmdAutoLabel",s.filter.bodyHmdAuto) : T("bodyUnset")),"bodyHmd");
    std::vector<Device> waists, heads;
    std::string bodyStatus = T("bodyLearning");
    for (const auto& d : s.devices) {
        if (d.deviceClass != vr::TrackedDeviceClass_GenericTracker && d.deviceClass != vr::TrackedDeviceClass_HMD) continue;
        const bool head = d.deviceClass == vr::TrackedDeviceClass_HMD;
        Device choice; choice.index = int(d.index); choice.model = d.model; choice.serial = d.serial;
        choice.on = Upper(d.serial) == (head ? s.filter.bodyHmdSerial : s.filter.bodyWaistSerial);
        (head ? heads : waists).push_back(choice);
        if (!head && choice.on) {
            const auto f = s.filterDevices.find(d.index);
            if (f != s.filterDevices.end() && f->second.bodyReady)
                bodyStatus = T(f->second.bodyActive ? "bodyEstimating" : "bodyReady") + " (" + Num(f->second.torsoLengthCm,1) + " cm)";
        }
    }
    if (!bodyEnabled_) bodyStatus = T("bodyOff");
    else if (s.filter.bodyWaistSerial.empty() || refHmd.empty()) bodyStatus = T("bodyMappingNeeded");
    Set(bodyStatus_,Rml::String(bodyStatus),"bodyStatus");
    Set(bodyWaistChoices_,waists,"bodyWaistChoices"); Set(bodyHmdChoices_,heads,"bodyHmdChoices");
    const bool editing = Clock::now() - paramEditAt_ < std::chrono::milliseconds(1500);
    if (!editing && s.filter.valid) paramEdit_.clear();
    std::vector<Param> params;
    for (const ParamDef& d : kParams) {
        if (std::string(d.key).rfind("hand",0) == 0 && !handEnabled_) continue;
        Param p;
        p.key = d.key;
        p.label = T(d.text);
        p.help = T((std::string(d.text) + "Help").c_str());
        p.group = d.group ? T(d.group) : "";
        p.min = d.min, p.max = d.max, p.step = d.step;
        auto e = paramEdit_.find(d.key);
        p.value = e != paramEdit_.end() ? e->second : s.filter.valid ? ConfigValue(s.filter, d.key) : d.def;
        p.slider = std::round((p.value - d.min) / (d.max - d.min) * 1000);
        const std::string unit = d.unit;
        p.display = unit == "ms" ? Fmt(lang, "sec", Num(p.value / 1000, 2)) : unit == "%" ? Num(p.value, 0) + " %" : Num(p.value, 1) + " cm";
        params.push_back(std::move(p));
    }
    Set(params_, params, "params");
    Set(recordLabel_, Rml::String(T(recording_ ? "recordStop" : "recordStart")), "recordLabel");
    Set(recordIcon_, Rml::String(recording_ ? "icons/square.svg" : "icons/circle-dot.svg"), "recordIcon");

    std::vector<ThemeCard> themes;
    for (int i = 0; i < ThemeCount(); ++i) {
        const Theme& t = GetTheme(i);
        ThemeCard c;
        c.index = i;
        c.name = lang == "ja" ? t.nameJa : t.nameEn;
        c.bg = Css(t.background), c.panel = Css(t.panel), c.accent = Css(t.accent), c.text = Css(t.text);
        c.ok = Css(t.ok), c.hold = Css(t.hold), c.lost = Css(t.lost), c.edge = Css(Mix(t.background, t.text, 0.12f));
        c.active = i == settings_.theme;
        c.isDefault = i == 0;
        themes.push_back(std::move(c));
    }
    Set(themes_, themes, "themes");
}

// Data for <lanegraph> and <mapview>.
void UiApp::BuildScene(const StateData& s) {
    UiScene& sc = Scene();
    const Theme& th = GetTheme(settings_.theme);
    sc.colors = {th.text, Mix(th.text, th.background, 0.42f), Mix(th.panel, th.text, 0.14f), th.accent, AccentText(th),
                 th.ok, th.hold, th.lost, th.beacon, th.background};
    sc.mapMode = settings_.mapMode;
    if (!sc.cameraChanged) {
        sc.camYaw = settings_.camYaw, sc.camPitch = settings_.camPitch, sc.camDistance = settings_.camDistance;
        sc.camTarget = Vec3{settings_.camTargetX, settings_.camTargetY, settings_.camTargetZ};
    }
    const int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
    sc.now = s.traceNow + std::clamp((nowMs - s.traceNowLocalMs) / 1000.0, 0.0, 0.3);
    sc.playArea = s.playArea;

    const auto devs = TrackedDevices(s);
    sc.lanes.assign(devs.size(), {});
    sc.map.clear();
    for (size_t i = 0; i < devs.size(); ++i) {
        const DeviceInfo& d = *devs[i];
        const bool corrected = i < lanes_.size() && lanes_[i].on && filterOn_;
        LaneSeries& lane = sc.lanes[i];
        lane.corrected = corrected;
        auto tr = s.trace.find(d.serial);
        const TraceSample* last = nullptr;
        if (tr != s.trace.end()) {
            lane.samples.reserve(tr->second.size());
            for (const TraceSample& t : tr->second) lane.samples.push_back({t.t, t.raw, t.out, t.rawValid, t.state});
            if (!tr->second.empty()) last = &tr->second.back();
        }
        MapItem m;
        m.kind = MapItem::Device;
        m.pos = m.raw = d.position;
        m.forward = d.forward, m.right = d.right, m.up = d.up;
        const char st = i < lanes_.size() ? (lanes_[i].tone == "lost" ? 'L' : lanes_[i].tone == "busy" ? 'H' : 'T') : 'T';
        m.state = st;
        if (last && last->rawValid && st == 'H') {
            const Vec3 off = Rotate(s.rawToStanding, {last->raw.x - last->out.x, last->raw.y - last->out.y, last->raw.z - last->out.z});
            m.raw = {d.position.x + off.x, d.position.y + off.y, d.position.z + off.z};
            m.showRaw = std::sqrt(off.x * off.x + off.y * off.y + off.z * off.z) > 0.03f;
        }
        m.label = d.deviceClass == vr::TrackedDeviceClass_HMD ? "HMD" : d.serial.size() > 4 ? d.serial.substr(d.serial.size() - 4) : d.serial;
        sc.map.push_back(std::move(m));
    }
    sc.skeletons.clear();
    if (settings_.modelOnMap)
        for (const ModelPose& m : s.models) {
            Skeleton k;
            k.estimating = m.estimating;
            k.joints.push_back(Transform(s.rawToStanding, m.neck)); // driver world -> standing, like the devices
            if (m.haveShoulder) k.joints.push_back(Transform(s.rawToStanding, m.shoulder));
            if (m.haveElbow) k.joints.push_back(Transform(s.rawToStanding, m.elbow));
            k.joints.push_back(Transform(s.rawToStanding, m.end));
            sc.skeletons.push_back(std::move(k));
        }
    for (const DeviceInfo& d : s.devices) {
        if (d.deviceClass != vr::TrackedDeviceClass_TrackingReference) continue;
        MapItem m;
        m.kind = MapItem::BaseStation;
        m.pos = m.raw = d.position;
        m.forward = d.forward, m.right = d.right, m.up = d.up;
        m.fovH = d.fovH, m.fovV = d.fovV;
        m.sleeping = !d.poseValid;
        m.label = d.serial.size() > 4 ? d.serial.substr(d.serial.size() - 4) : d.serial;
        sc.map.push_back(std::move(m));
    }
}

} // namespace ots
