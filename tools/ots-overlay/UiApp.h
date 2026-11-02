#pragma once
// The UI: one RmlUi context showing ui/app.rml, driven by the "app" data model. The SteamVR dashboard
// overlay and the desktop window show the same render target and both feed their mouse input here.
#include "Settings.h"
#include "State.h"

#include <RmlUi/Core/DataModelHandle.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace Rml {
class Context;
class ElementDocument;
} // namespace Rml

namespace ots {

class Gfx;

// What the UI can ask for. Each is non-blocking (queued to a worker or a quick OpenVR call).
struct UiActions {
    std::function<void(const std::string&)> driver;                                   // pipe command
    std::function<bool(const std::string& key, const std::string& value)> saveSetting; // steamvr.vrsettings
    std::function<void(const std::string&)> openUrl;
    std::function<void(const std::string&)> log; // "this window" entries in the log (optional)
};

class UiApp {
public:
    UiApp(Gfx& gfx, OverlaySettings& settings, UiActions actions, std::filesystem::path uiDir);
    ~UiApp();

    bool Init();
    void Update(const StateData& s); // once per frame while the UI is shown, before Gfx::Render
    // Writes settings that are due, or all that are waiting. Every frame, shown or not: the UI may close
    // right after a change.
    void SaveDue(bool all = false);
    // Frees the UI; call before Gfx::Shutdown, which takes RmlUi down.
    void Shutdown();
    Rml::Context* Context() const { return context_; }

    // Renders at `scale` pixels per dp (the layout is always kWidth x kHeight dp); call after Gfx::Resize.
    void SetScale(float scale);

    // Mouse position in dp (origin top-left), whatever the render scale.
    void MouseMove(float x, float y);
    void MouseButton(int button, bool down);
    void MouseWheel(float linesDown);
    void MouseLeave();

    bool DesktopWindowWanted() const { return settings_.desktopWindow; }
    bool ModelInVrWanted() const { return settings_.modelInVr; }
    void SetDesktopWindow(bool show);

    // Screenshot mode.
    void SetView(const std::string& view);
    void SetLogMenu(bool open);

    struct Lane {
        int index = 0;
        Rml::String name, serial, icon, state, stateClass, drift, tone;
        bool on = false, locked = false;
        bool operator==(const Lane&) const = default;
    };
    struct LogLine {
        Rml::String time, text, cls;
        bool operator==(const LogLine&) const = default;
    };
    struct LogKind {
        Rml::String id, label, help;
        bool on = false;
        bool operator==(const LogKind&) const = default;
    };
    struct Device {
        int index = 0;
        Rml::String model, serial, icon, batt, battClass, battIcon, state, stateClass, sub, holds;
        bool hasBatt = false, on = false, locked = false, alt = false;
        bool operator==(const Device&) const = default;
    };
    struct Base {
        Rml::String serial, ch, height, tilt, state, stateClass;
        bool alt = false;
        bool operator==(const Base&) const = default;
    };
    struct Param {
        Rml::String key, label, help, group, display;
        float value = 0, slider = 0; // slider: value mapped to the input's fixed 0..1000 range
        float min = 0, max = 1, step = 1;
        bool operator==(const Param&) const = default;
    };
    struct ThemeCard {
        int index = 0;
        Rml::String name, bg, panel, accent, text, ok, hold, lost, edge;
        bool active = false, isDefault = false;
        bool operator==(const ThemeCard&) const = default;
    };

private:
    bool LoadDocument();
    void BindModel();
    void Toast(const std::string& text, bool log = true);
    const std::string& T(const char* key) const;

    void BuildMain(const StateData& s);
    void BuildDevices(const StateData& s);
    void BuildSettings(const StateData& s);
    void BuildScene(const StateData& s);

    // Event handlers (names match data-event-* in app.rml).
    void OnToggleDevice(int index);
    void OnSetParam(const std::string& key, float value);
    void SaveDriverSetting(const std::string& key, const std::string& value);
    void SendDeviceLists();

    template <class T>
    void Set(T& field, const T& value, const char* name) {
        if (field == value) return;
        field = value;
        model_.DirtyVariable(name);
    }

    Gfx& gfx_;
    OverlaySettings& settings_;
    UiActions act_;
    std::filesystem::path uiDir_;
    Rml::Context* context_ = nullptr;
    Rml::ElementDocument* document_ = nullptr;
    Rml::DataModelHandle model_;
    bool reload_ = false;
    bool settingsDirty_ = false;
    std::chrono::steady_clock::time_point cameraSaveAt_{};
    float scale_ = 1; // render pixels per dp

    // The devices of the latest snapshot: mouse events call the handlers between Updates.
    std::vector<DeviceInfo> latestDevices_;

    // Bound to the data model.
    Rml::String view_ = "main", lang_, version_, warning_, heroLine_, heroWord_, sessionText_, lanesEmpty_, mapMode_;
    Rml::String toast_, settingsPath_, recordLabel_, recordIcon_, logFilterCount_;
    bool mapPan_ = false, filterOn_ = false, recording_ = false, logMenu_ = false, logFiltered_ = false, allowHmd_ = false;
    bool desktopWindow_ = false, modelOnMap_ = true, modelInVr_ = false;
    bool bodyEnabled_ = false;
    bool handEnabled_ = false;
    Rml::String handStatus_;
    std::optional<bool> handWanted_;
    std::optional<bool> imuWanted_;
    std::chrono::steady_clock::time_point imuWantedAt_{};
    bool imuEnabled_ = false;
    Rml::String imuStatus_;
    std::chrono::steady_clock::time_point handWantedAt_{};
    Rml::String bodyStatus_, bodyWaist_, bodyHmd_;
    std::vector<Device> bodyWaistChoices_, bodyHmdChoices_;
    std::optional<bool> bodyWanted_;
    std::chrono::steady_clock::time_point bodyWantedAt_{};
    int deviceCount_ = 0, holdingCount_ = 0;
    std::vector<Lane> lanes_;
    std::vector<LogLine> log_;
    std::vector<LogKind> logKinds_;
    std::vector<Device> devices_;
    std::vector<Base> bases_;
    std::vector<Param> params_;
    std::vector<ThemeCard> themes_;

    // UI-side state.
    std::chrono::steady_clock::time_point toastUntil_{};
    std::map<std::string, float> paramEdit_;                                         // values the user set recently
    std::chrono::steady_clock::time_point paramEditAt_{};
    std::map<std::string, std::pair<std::string, std::chrono::steady_clock::time_point>> pendingSave_; // debounced
    std::map<std::string, std::pair<bool, std::chrono::steady_clock::time_point>> deviceOverride_; // serial -> wanted
    bool listsLoaded_ = false; // include_ / exclude_ hold the driver's lists
    std::vector<std::string> include_, exclude_;                                   // what we last asked the driver for
    std::optional<bool> filterWanted_;
    std::chrono::steady_clock::time_point filterWantedAt_{};
    std::optional<bool> allowHmdWanted_;
    std::chrono::steady_clock::time_point allowHmdWantedAt_{};
};

} // namespace ots
