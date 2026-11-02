// ots-overlay: SteamVR dashboard UI for OpenVR-TrackingStabilizer (docs/phase2.md), also shown in a
// desktop window.
//
//   ots-overlay                     run as a dashboard overlay (registers itself for auto-launch)
//   ots-overlay --unregister        remove the auto-launch registration
//   ots-overlay --screenshot <dir>  render every screen with demo data to PNG files (no SteamVR needed)
#include "DesktopWindow.h"
#include "DriverClient.h"
#include "Gfx.h"
#include "OtsConstants.h"
#include "Settings.h"
#include "UiApp.h"
#include "VrModelOverlays.h"
#include "VrSource.h"

#include <RmlUi/Core.h>
#include <openvr.h>

#include <windows.h>
#include <shellapi.h>
#include <tlhelp32.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kAppKey = "ots.trackingstabilizer.manager";
constexpr const char* kOverlayKey = "ots.trackingstabilizer.dashboard";
// Layout size in dp (app.rcss). The UI renders at kMinScale or more pixels per dp: at least 1600 px wide
// for the VR overlay, more when the desktop window is bigger than that.
constexpr int kWidth = 1120, kHeight = 700;
constexpr float kMinScale = 1600.0f / kWidth;
int Px(float dp, float scale) { return int(std::lround(dp * scale)); }

void AttachParentConsole() {
    if (AttachConsole(ATTACH_PARENT_PROCESS)) {
        FILE* f;
        freopen_s(&f, "CONOUT$", "w", stdout);
        freopen_s(&f, "CONOUT$", "w", stderr);
    }
}

void Fatal(const std::wstring& message) {
    std::fwprintf(stderr, L"%s\n", message.c_str());
    MessageBoxW(nullptr, message.c_str(), L"OpenVR Tracking Stabilizer", MB_ICONERROR);
}

bool SteamVrRunning() {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    PROCESSENTRY32W e{sizeof(e)};
    bool found = false;
    for (BOOL ok = Process32FirstW(snap, &e); ok; ok = Process32NextW(snap, &e))
        if (!_wcsicmp(e.szExeFile, L"vrserver.exe")) found = true;
    CloseHandle(snap);
    return found;
}

std::filesystem::path ExeDir() {
    wchar_t buf[MAX_PATH];
    GetModuleFileNameW(nullptr, buf, MAX_PATH);
    return std::filesystem::path(buf).parent_path();
}

std::string Utf8(const std::filesystem::path& p) {
    const std::u8string s = p.u8string();
    return std::string(s.begin(), s.end());
}

// Demo state for --screenshot: a typical setup with made-up serials, the waist tracker mid-loss.
void FillDemo(ots::StateData& s, double now) {
    using namespace ots;
    s.vrConnected = s.driverConnected = true;
    s.playArea = {{-1.5f, 0, -1.2f}, {1.5f, 0, -1.2f}, {1.5f, 0, 1.2f}, {-1.5f, 0, 1.2f}};
    struct Demo {
        uint32_t index;
        int cls;
        const char* serial;
        const char* model;
        int role;
        Vec3 pos;
        float batt;
        double period, offset, dur; // scripted tracking-loss cycle
        Vec3 dir;
    };
    const Demo demo[] = {
        {0, vr::TrackedDeviceClass_HMD, "LHR-1A2B3C01", "Beyond", 0, {0.1f, 1.62f, 0.05f}, -1, 0, 0, 0, {}},
        {3, vr::TrackedDeviceClass_GenericTracker, "LHR-1A2B3C02", "Tundra Tracker", 0, {-0.12f, 0.08f, 0.1f}, 0.72f, 19, 5, 0.6, {0.4f, -0.3f, 1}},
        {4, vr::TrackedDeviceClass_GenericTracker, "LHR-1A2B3C03", "Tundra Tracker", 0, {0.14f, 0.08f, 0.12f}, 0.35f, 23, 13, 1.2, {-1, 0.2f, 0.5f}},
        {6, vr::TrackedDeviceClass_Controller, "LHR-1A2B3C04", "Knuckles Right", 2, {0.35f, 1.05f, -0.2f}, 0.53f, 29, 3, 0.4, {0.8f, 0.6f, 0}},
        {7, vr::TrackedDeviceClass_GenericTracker, "LHR-1A2B3C05", "Tundra Tracker", 0, {0.08f, 0.98f, 0.12f}, 0.15f, 12, 0, 2.0, {0.5f, 1, -0.6f}},
        {8, vr::TrackedDeviceClass_Controller, "LHR-1A2B3C06", "Knuckles Left", 1, {-0.3f, 1.0f, -0.25f}, 0.49f, 37, 20, 0.5, {-0.6f, 0.8f, 0.2f}},
    };
    auto truth = [](const Demo& d, int axis, double t) {
        const double k = axis * 1.7 + d.index;
        const float base = axis == 0 ? d.pos.x : axis == 1 ? d.pos.y : d.pos.z;
        return base + 0.03 * std::sin(2 * 3.14159 * t / 2.3 + k) + 0.012 * std::sin(2 * 3.14159 * t / 0.87 + 2 * k);
    };
    for (const Demo& d : demo) {
        DeviceInfo di;
        di.index = d.index, di.deviceClass = d.cls, di.serial = d.serial, di.model = d.model, di.role = d.role, di.connected = true;
        di.poseValid = true, di.trackingResult = vr::TrackingResult_Running_OK;
        di.position = d.pos, di.forward = {0, 0, -1};
        di.hasBattery = d.batt >= 0, di.battery = d.batt;
        di.charging = d.index == 8; // Knuckles Left on its cable
        s.devices.push_back(di);
        FilterDeviceInfo f;
        f.serial = d.serial;
        f.mode = d.cls == vr::TrackedDeviceClass_HMD ? "bypass" : "filtered";
        f.state = "TRACKING";
        f.holds = int(d.index % 5);
        if (d.period <= 0) {
            s.filterDevices[d.index] = f;
            continue;
        }
        // 30 Hz trace with the scripted loss: raw drifts away, output holds, then blends back.
        auto& tr = s.trace[d.serial];
        for (double t = now - 22; t <= now; t += 1.0 / 30) {
            const double ph = std::fmod(t + d.offset, d.period), dt = ph - (d.period - 6);
            TraceSample smp;
            smp.t = t;
            const float tx = float(truth(d, 0, t)), ty = float(truth(d, 1, t)), tz = float(truth(d, 2, t));
            smp.raw = smp.out = {tx, ty, tz};
            const double pr = d.dur >= 2 ? 0.4 : 0.1;
            if (dt >= 0 && dt < d.dur + pr + 0.2) {
                const Vec3 held = {float(truth(d, 0, t - dt)), float(truth(d, 1, t - dt)), float(truth(d, 2, t - dt))};
                if (dt < d.dur) {
                    const float a = float(0.5 * 0.30 * dt * dt);
                    smp.raw = {tx + d.dir.x * a, ty + d.dir.y * a, tz + d.dir.z * a};
                    smp.out = held, smp.state = 'H';
                } else if (dt < d.dur + pr) {
                    smp.out = held, smp.state = 'P';
                } else {
                    const float k = float((dt - d.dur - pr) / 0.2);
                    smp.out = {held.x + (tx - held.x) * k, held.y + (ty - held.y) * k, held.z + (tz - held.z) * k}, smp.state = 'B';
                }
            }
            tr.push_back(smp);
        }
        const char last = tr.back().state;
        f.state = last == 'H' ? "HOLD" : last == 'P' ? "PROBATION" : last == 'B' ? "BLEND" : "TRACKING";
        if (last == 'H') f.stateSeconds = std::fmod(now + d.offset, d.period) - (d.period - 6);
        s.filterDevices[d.index] = f;
    }
    s.traceSupported = true;
    s.traceNow = now;
    s.traceNowLocalMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto bs = [&](uint32_t i, const char* serial, Vec3 pos, Vec3 fwd, const char* ch) {
        DeviceInfo d;
        d.index = i, d.deviceClass = vr::TrackedDeviceClass_TrackingReference, d.serial = serial, d.model = "Valve SR Imp";
        d.connected = d.poseValid = true, d.trackingResult = vr::TrackingResult_Running_OK, d.position = pos, d.forward = fwd;
        d.modeLabel = ch;
        s.devices.push_back(d);
    };
    bs(1, "LHB-5D6E7F01", {1.9f, 2.3f, -1.8f}, {-0.55f, -0.55f, 0.62f}, "1");
    bs(2, "LHB-5D6E7F02", {-1.9f, 2.2f, -1.7f}, {0.6f, -0.5f, 0.62f}, "13");
    bs(5, "LHB-5D6E7F03", {0.2f, 2.4f, 2.1f}, {-0.05f, -0.55f, -0.83f}, "4");
    s.filter.valid = s.filter.enabled = true;
    // Both arms of the hand model; the right hand is hidden and estimated (rawToStanding is identity here).
    s.models.push_back({"LHR-1A2B3C04", true, true, true, true, {0.1f, 1.5f, 0.12f}, {0.28f, 1.4f, 0.12f}, {0.38f, 1.16f, 0.02f},
                        {0.35f, 1.05f, -0.2f}});
    s.models.push_back({"LHR-1A2B3C06", true, false, true, true, {0.1f, 1.5f, 0.12f}, {-0.08f, 1.4f, 0.12f}, {-0.2f, 1.15f, 0.0f},
                        {-0.3f, 1.0f, -0.25f}});
    const int64_t t0 = 1792831200000;
    auto log = [&](int64_t dt, const char* kind, const char* text) { s.log.push_back({0, t0 + dt, kind, text}); };
    log(0, "driver", "LHR-1A2B3C05 C: SOB: drop S-4 seeing S-1 (5D6E7F01) S-13 (5D6E7F02)");
    log(250, "filter", "LHR-1A2B3C05 TRACKING -> HOLD (result not OK)");
    log(2250, "driver", "LHR-1A2B3C05 C: Resetting tracking: no optical samples for 2002ms");
    log(6900, "driver", "LHR-1A2B3C05 C: ----- BOOTSTRAPPED base 5D6E7F01 (immediate) distance 2.22m -----");
    log(6901, "filter", "LHR-1A2B3C05 HOLD -> PROBATION (tracking resumed)");
    log(7301, "filter", "LHR-1A2B3C05 PROBATION -> BLEND (probation passed)");
    log(7501, "filter", "LHR-1A2B3C05 BLEND -> TRACKING (blend done)");
    log(9000, "ots", "recording started: sessions\\20261025-212015_overlay.otsrec");
}

// Mouse events arrive between frames, outside UiApp::Update: click like that and report what reached the driver.
void ClickChecks(ots::Gfx& gfx, ots::OverlaySettings& settings, FILE* report) {
    std::vector<std::string> sent;
    ots::UiActions record{[&](const std::string& c) { sent.push_back(c); }, [](const std::string&, const std::string&) { return true; },
                          [](const std::string&) {}};
    ots::UiApp ui(gfx, settings, record, ExeDir() / "ui");
    if (!ui.Init()) return;
    gfx.Resize(Px(kWidth, kMinScale), Px(kHeight, kMinScale));
    ui.SetScale(kMinScale);
    ots::StateData s;
    FillDemo(s, 103.0);
    s.filter.bodyEnabled = true;
    auto frames = [&] {
        for (int f = 0; f < 3; ++f) {
            ui.Update(s);
            gfx.Render(ui.Context());
        }
    };
    auto click = [&](const char* what, Rml::Element* e) {
        sent.clear();
        if (e) {
            e->ScrollIntoView();
            frames();
            const Rml::Vector2f at = e->GetAbsoluteOffset(Rml::BoxArea::Border) + e->GetBox().GetSize(Rml::BoxArea::Border) / 2.0f;
            ui.MouseMove(at.x / kMinScale, at.y / kMinScale);
            ui.MouseButton(0, true);
            ui.MouseButton(0, false);
        }
        std::fprintf(report, "click %s:%s\n", what, e ? "" : " NOT FOUND");
        for (const std::string& c : sent) std::fprintf(report, "  %s\n", c.c_str());
    };
    auto byClass = [&](const char* name) {
        Rml::ElementList all;
        ui.Context()->GetDocument(0)->GetElementsByClassName(all, name);
        return all;
    };
    auto button = [&](const char* text) -> Rml::Element* {
        for (Rml::Element* e : byClass("btn"))
            if (e->GetInnerRML().find(text) != Rml::String::npos) return e;
        return nullptr;
    };
    ui.SetView("settings");
    frames();
    click("waist tracker", button("LHR-1A2B3C05"));
    click("reference HMD", button("LHR-1A2B3C01"));
    ui.SetView("main");
    frames();
    Rml::Element* lane = nullptr;
    for (Rml::Element* e : byClass("switch"))
        if (!lane && e->IsClassSet("small") && e->IsVisible(true) && !e->IsClassSet("locked")) lane = e;
    click("device switch", lane);

    ui.Shutdown(); // only one UI at a time

    // Before the driver's lists are known, a switch must not save empty ones over the user's.
    ots::UiApp fresh(gfx, settings, record, ExeDir() / "ui");
    if (fresh.Init()) {
        fresh.SetScale(kMinScale);
        s.filter.valid = false;
        for (int f = 0; f < 3; ++f) {
            fresh.Update(s);
            gfx.Render(fresh.Context());
        }
        lane = nullptr;
        Rml::ElementList all;
        fresh.Context()->GetDocument(0)->GetElementsByClassName(all, "switch");
        for (Rml::Element* e : all)
            if (!lane && e->IsClassSet("small") && e->IsVisible(true) && !e->IsClassSet("locked")) lane = e;
        sent.clear();
        if (lane) {
            const Rml::Vector2f at = lane->GetAbsoluteOffset(Rml::BoxArea::Border) + lane->GetBox().GetSize(Rml::BoxArea::Border) / 2.0f;
            fresh.MouseMove(at.x / kMinScale, at.y / kMinScale);
            fresh.MouseButton(0, true);
            fresh.MouseButton(0, false);
        }
        std::fprintf(report, "click device switch, driver lists unknown:%s\n", lane ? "" : " NOT FOUND");
        for (const std::string& c : sent) std::fprintf(report, "  %s\n", c.c_str());
    }
}

// Takes the UI down in the order RunOverlay does.
void ExitCheck(ots::Gfx& gfx, ots::OverlaySettings& settings, FILE* report) {
    ots::UiActions none{[](const std::string&) {}, [](const std::string&, const std::string&) { return true; }, [](const std::string&) {}};
    ots::UiApp ui(gfx, settings, none, ExeDir() / "ui");
    if (!ui.Init()) return;
    ots::StateData s;
    ui.Update(s);
    ui.Shutdown();
    gfx.Shutdown();
    std::fprintf(report, "exit order ok\n");
}

int Screenshots(const std::filesystem::path& dir) {
    ots::Gfx gfx;
    if (!gfx.Init(Px(kWidth, kMinScale), Px(kHeight, kMinScale), ExeDir())) return 1;
    std::filesystem::create_directories(dir);
    FILE* report = _wfopen((dir / L"report.txt").c_str(), L"w"); // no console in a WIN32 app
    ots::OverlaySettings settings;
    ots::UiActions none{[](const std::string&) {}, [](const std::string&, const std::string&) { return true; }, [](const std::string&) {}};
    struct Shot {
        const wchar_t* name;
        const char* view;
        const char* lang;
        int theme;
        bool logMenu;
        ots::MapMode map;
        float mouseX = -1, mouseY = -1; // hover here (tooltips)
        float scale = kMinScale;          // render pixels per dp (more: a high-DPI desktop window)
        bool body = false;
        bool hand = false;
    };
    const Shot shots[] = {
        {L"main_ja", "main", "ja", 0, false, ots::MapMode::ThreeD},  {L"main_en", "main", "en", 0, false, ots::MapMode::ThreeD},
        {L"main_logmenu", "main", "ja", 0, true, ots::MapMode::Top}, {L"devices_ja", "devices", "ja", 0, false, ots::MapMode::ThreeD},
        {L"settings_ja", "settings", "ja", 0, false, ots::MapMode::ThreeD}, {L"settings_en", "settings", "en", 0, false, ots::MapMode::Side},
        {L"main_crimson", "main", "ja", 1, false, ots::MapMode::Side}, {L"main_light", "main", "en", 3, false, ots::MapMode::ThreeD},
        {L"tooltip_tab", "main", "ja", 0, false, ots::MapMode::ThreeD, 1031, 28},
        {L"tooltip_map", "main", "en", 0, false, ots::MapMode::ThreeD, 992, 95},
        {L"tooltip_maptool", "main", "ja", 0, false, ots::MapMode::ThreeD, 711, 144},
        {L"main_ja_2x", "main", "ja", 0, false, ots::MapMode::ThreeD, -1, -1, 2.0f},
        {L"devices_ja_2x", "devices", "ja", 0, false, ots::MapMode::ThreeD, -1, -1, 2.0f},
        {L"settings_ja_2x", "settings", "ja", 0, false, ots::MapMode::ThreeD, -1, -1, 2.0f},
        {L"settings_body_ja", "settings", "ja", 0, false, ots::MapMode::ThreeD, -1, -1, kMinScale, true},
        {L"settings_body_en", "settings", "en", 0, false, ots::MapMode::ThreeD, -1, -1, kMinScale, true},
        {L"settings_hand_ja", "settings", "ja", 0, false, ots::MapMode::ThreeD, -1, -1, kMinScale, false, true},
        {L"settings_hand_en", "settings", "en", 0, false, ots::MapMode::ThreeD, -1, -1, kMinScale, false, true},
    };
    for (const Shot& shot : shots) {
        settings.lang = shot.lang;
        settings.theme = shot.theme;
        settings.mapMode = shot.map;
        ots::UiApp ui(gfx, settings, none, ExeDir() / "ui");
        if (!ui.Init()) {
            if (report) std::fprintf(report, "UiApp::Init failed\n");
            break;
        }
        gfx.Resize(Px(kWidth, shot.scale), Px(kHeight, shot.scale));
        ui.SetScale(shot.scale);
        ui.SetView(shot.view);
        ui.SetLogMenu(shot.logMenu);
        ots::StateData s;
        FillDemo(s, 103.0);
        if (shot.body) {
            s.filter.imuEnabled = true;
            for (auto& [index, f] : s.filterDevices)
                if (f.mode == "filtered") f.imuOpen = true, f.imuAligned = index != 5, f.imuRate = 1000;
            s.filterDevices[7].imuActive = true;
            s.filter.bodyEnabled = true;
            s.filter.bodyWaistSerial = "LHR-1A2B3C05"; s.filter.bodyHmdSerial = "LHR-1A2B3C01";
            s.filterDevices[7].bodyReady = s.filterDevices[7].bodyActive = true; s.filterDevices[7].torsoLengthCm = 52;
        }
        if (shot.hand) {
            s.filter.handEnabled = true; s.filter.bodyHmdSerial = "LHR-1A2B3C01";
            s.filterDevices[6].handSide = 1; s.filterDevices[8].handSide = -1;
            s.filterDevices[6].handReady = s.filterDevices[8].handReady = true;
            s.filterDevices[6].handActive = true;
        }
        for (int f = 0; f < 3; ++f) { // a few frames so layout and data bindings settle
            if (shot.mouseX >= 0) ui.MouseMove(shot.mouseX, shot.mouseY);
            ui.Update(s);
            gfx.Render(ui.Context());
        }
        const std::filesystem::path path = dir / (std::wstring(shot.name) + L".png");
        const bool ok = gfx.SavePng(path.wstring());
        if (report) std::fwprintf(report, L"%s %s\n", ok ? L"wrote" : L"FAILED", path.c_str());
    }
    if (report) {
        ClickChecks(gfx, settings, report);
        for (const std::string& line : gfx.TakeLog()) std::fprintf(report, "%s\n", line.c_str());
        ExitCheck(gfx, settings, report);
        std::fclose(report);
    }
    return 0;
}

// Registers this copy's manifest. If SteamVR knows the app from another folder (a build folder, an older
// install), that registration is replaced so SteamVR launches the copy that is running now.
bool Register() {
    vr::IVRApplications* apps = vr::VRApplications();
    if (!apps) return false;
    const std::string manifest = Utf8(ExeDir() / "manifest.vrmanifest");
    if (apps->IsApplicationInstalled(kAppKey)) {
        char dir[MAX_PATH * 4] = {};
        vr::EVRApplicationError e = vr::VRApplicationError_None;
        apps->GetApplicationPropertyString(kAppKey, vr::VRApplicationProperty_WorkingDirectory_String, dir, sizeof(dir), &e);
        std::error_code ec;
        const std::filesystem::path registered(std::u8string(reinterpret_cast<const char8_t*>(dir)));
        if (e == vr::VRApplicationError_None && dir[0] && !std::filesystem::equivalent(registered, ExeDir(), ec)) {
            apps->RemoveApplicationManifest(Utf8(registered / "manifest.vrmanifest").c_str());
            if (apps->AddApplicationManifest(manifest.c_str(), false) != vr::VRApplicationError_None) return false;
        }
    } else if (apps->AddApplicationManifest(manifest.c_str(), false) != vr::VRApplicationError_None) {
        return false;
    }
    apps->SetApplicationAutoLaunch(kAppKey, true);
    return true;
}

bool SaveVrSetting(const std::string& key, const std::string& value) {
    vr::IVRSettings* st = vr::VRSettings();
    if (!st) return false;
    vr::EVRSettingsError e = vr::VRSettingsError_None;
    if (value == "true" || value == "false") st->SetBool(ots::kSettingsSection, key.c_str(), value == "true", &e);
    else if (key.find("Serial") != std::string::npos) st->SetString(ots::kSettingsSection, key.c_str(), value.c_str(), &e);
    else st->SetFloat(ots::kSettingsSection, key.c_str(), std::strtof(value.c_str(), nullptr), &e);
    return e == vr::VRSettingsError_None;
}

int RunOverlay(bool unregister) {
    if (!SteamVrRunning()) {
        Fatal(L"SteamVR が起動していません。SteamVR を起動してから実行してください。\nSteamVR is not running.");
        return 1;
    }
    // SteamVR starts this app itself, so opening it from the Start menu usually finds it running.
    const HANDLE instance = unregister ? nullptr : CreateMutexW(nullptr, FALSE, L"Local\\OpenVR-TrackingStabilizer-Overlay");
    if (instance && GetLastError() == ERROR_ALREADY_EXISTS) {
        ots::DesktopWindow::RequestShow();
        CloseHandle(instance);
        return 0;
    }
    vr::EVRInitError err = vr::VRInitError_None;
    vr::VR_Init(&err, vr::VRApplication_Overlay);
    if (err != vr::VRInitError_None) {
        const char* why = vr::VR_GetVRInitErrorAsEnglishDescription(err);
        Fatal(L"OpenVR の初期化に失敗しました: " + std::wstring(why, why + std::strlen(why)));
        return 1;
    }
    if (unregister) {
        vr::VRApplications()->SetApplicationAutoLaunch(kAppKey, false);
        vr::VRApplications()->RemoveApplicationManifest(Utf8(ExeDir() / "manifest.vrmanifest").c_str());
        std::printf("unregistered\n");
        vr::VR_Shutdown();
        return 0;
    }
    if (!Register()) std::fprintf(stderr, "registration for auto-launch failed (continuing)\n");

    ots::Gfx gfx;
    if (!gfx.Init(Px(kWidth, kMinScale), Px(kHeight, kMinScale), ExeDir())) {
        Fatal(L"Direct3D 11 / RmlUi の初期化に失敗しました。");
        vr::VR_Shutdown();
        return 1;
    }
    vr::VROverlayHandle_t main = vr::k_ulOverlayHandleInvalid, thumb = vr::k_ulOverlayHandleInvalid;
    vr::IVROverlay* ov = vr::VROverlay();
    if (ov->CreateDashboardOverlay(kOverlayKey, "Tracking Stabilizer", &main, &thumb) != vr::VROverlayError_None) {
        Fatal(L"ダッシュボードオーバーレイを作れませんでした（すでに起動しているかもしれません）。");
        vr::VR_Shutdown();
        return 1;
    }
    ov->SetOverlayWidthInMeters(main, 2.6f);
    ov->SetOverlayInputMethod(main, vr::VROverlayInputMethod_Mouse);
    vr::HmdVector2_t mouseScale{{float(kWidth), float(kHeight)}};
    ov->SetOverlayMouseScale(main, &mouseScale);
    ov->SetOverlayFlag(main, vr::VROverlayFlags_SendVRSmoothScrollEvents, true);
    ov->SetOverlayFromFile(thumb, Utf8(ExeDir() / "ui" / "icon.png").c_str());

    ots::AppState state;
    ots::DriverClient driver(state);
    driver.Start();

    ots::OverlaySettings settings = ots::LoadOverlaySettings();
    ots::UiActions act;
    act.driver = [&](const std::string& c) { driver.Send(c); };
    act.saveSetting = SaveVrSetting;
    act.openUrl = [](const std::string& url) { ShellExecuteA(nullptr, "open", url.c_str(), nullptr, nullptr, SW_SHOWNORMAL); };
    act.log = [&](const std::string& text) { state.AddUiLog(text); };

    ots::UiApp ui(gfx, settings, act, ExeDir() / "ui");
    if (!ui.Init()) {
        Fatal(L"UI の読み込みに失敗しました（ui フォルダを確認してください）。");
        driver.Stop();
        vr::VR_Shutdown();
        return 1;
    }
    ui.SetScale(kMinScale);

    ots::DesktopWindow desktop;
    ots::DesktopWindow::Callbacks cb;
    cb.mouseMove = [&](float x, float y) { ui.MouseMove(x, y); };
    cb.mouseButton = [&](int b, bool down) { ui.MouseButton(b, down); };
    cb.mouseWheel = [&](float lines) { ui.MouseWheel(lines); };
    cb.mouseLeave = [&] { ui.MouseLeave(); };
    cb.closed = [&] { ui.SetDesktopWindow(false); };
    cb.showRequested = [&] { ui.SetDesktopWindow(true); };
    std::function<bool()> frame; // set below
    cb.tick = [&] { if (frame) frame(); };
    const bool haveDesktop = desktop.Create(gfx.Device(), kWidth, kHeight, cb);

    using clock = std::chrono::steady_clock;
    auto lastPoll = clock::now() - std::chrono::seconds(1);
    float renderScale = kMinScale, wantedScale = kMinScale;
    auto wantedSince = clock::now();
    bool running = true;
    std::optional<ots::VrModelOverlays> modelOverlays; // destroyed before VR_Shutdown
    modelOverlays.emplace();
    // One UI frame; returns whether anything is showing the UI (or the body model in VR). Also run from
    // the desktop window's timer while Windows' modal move / size loop has the message pump.
    frame = [&]() -> bool {
        vr::VREvent_t ev;
        while (vr::VRSystem()->PollNextEvent(&ev, sizeof(ev)))
            if (ev.eventType == vr::VREvent_Quit) {
                ui.SaveDue(true);
                vr::VRSystem()->AcknowledgeQuit_Exiting();
                running = false;
            }
        while (ov->PollNextOverlayEvent(main, &ev, sizeof(ev))) {
            switch (ev.eventType) {
            case vr::VREvent_MouseMove:
                ui.MouseMove(ev.data.mouse.x, float(kHeight) - ev.data.mouse.y); // OpenVR's origin is bottom-left
                break;
            case vr::VREvent_MouseButtonDown:
            case vr::VREvent_MouseButtonUp:
                ui.MouseMove(ev.data.mouse.x, float(kHeight) - ev.data.mouse.y);
                ui.MouseButton(ev.data.mouse.button == vr::VRMouseButton_Right ? 1 : 0, ev.eventType == vr::VREvent_MouseButtonDown);
                break;
            case vr::VREvent_ScrollSmooth:
            case vr::VREvent_ScrollDiscrete:
                ui.MouseWheel(-ev.data.scroll.ydelta * 3);
                break;
            case vr::VREvent_FocusLeave:
                ui.MouseLeave();
                break;
            default:
                break;
            }
        }

        const bool vrVisible = ov->IsOverlayVisible(main);
        const bool deskVisible = desktop.Visible();
        const bool visible = vrVisible || deskVisible;
        const bool modelInVr = ui.ModelInVrWanted();
        driver.SetActive(visible);
        driver.SetModelFast(modelInVr);
        const auto now = clock::now();
        if (visible || modelInVr || now - lastPoll > std::chrono::milliseconds(500)) {
            ots::PollVr(state);
            lastPoll = now;
        }
        // Render at the desktop window's pixel size, so its text is drawn at the size it is shown
        // rather than scaled; while the VR dashboard shows the UI too, at least kMinScale so it stays
        // sharp there. Applied once the size has settled, not on every frame of a drag. A minimised
        // window keeps its scale, so restoring it is sharp straight away.
        float want = wantedScale;
        if (deskVisible) want = vrVisible ? std::max(kMinScale, desktop.PixelScale()) : desktop.PixelScale();
        else if (!desktop.Minimized()) want = kMinScale;
        if (std::abs(want - wantedScale) > 0.001f) {
            wantedScale = want;
            wantedSince = now;
        } else if (std::abs(wantedScale - renderScale) > 0.001f && now - wantedSince > std::chrono::milliseconds(300) &&
                   gfx.Resize(Px(kWidth, wantedScale), Px(kHeight, wantedScale))) {
            renderScale = wantedScale;
            ui.SetScale(renderScale);
        }
        if (visible) {
            const ots::StateData snapshot = state.Snapshot();
            ui.Update(snapshot);
            gfx.Render(ui.Context());
            if (vrVisible) {
                vr::Texture_t tex{gfx.Texture(), vr::TextureType_DirectX, vr::ColorSpace_Auto};
                ov->SetOverlayTexture(main, &tex);
            }
            desktop.Present(gfx.Texture());
        }
        ui.SaveDue();
        { // the body model around the user (debugging); hidden when off
            std::vector<ots::ModelPose> models;
            float rawToStanding[3][4];
            ots::Vec3 eye;
            bool haveEye = false;
            {
                std::lock_guard lock(state.m);
                if (modelInVr) models = state.models;
                std::copy(&state.rawToStanding[0][0], &state.rawToStanding[0][0] + 12, &rawToStanding[0][0]);
                for (const auto& d : state.devices)
                    if (d.deviceClass == vr::TrackedDeviceClass_HMD && d.poseValid) eye = d.position, haveEye = true;
            }
            modelOverlays->Update(models, rawToStanding, eye, modelInVr && haveEye);
        }
        return visible || modelInVr;
    };

    while (running) {
        const auto frameStart = clock::now();
        if (haveDesktop) {
            desktop.SetVisible(ui.DesktopWindowWanted());
            if (!desktop.PumpMessages()) running = false;
        }
        // ~30 fps while shown: plenty for a status UI and keeps the GPU for the game.
        const bool visible = frame();
        std::this_thread::sleep_until(frameStart + std::chrono::milliseconds(visible ? 33 : 100));
    }

    ui.SaveDue(true);
    desktop.Destroy();
    driver.Stop();
    modelOverlays.reset();
    ov->DestroyOverlay(main);
    ui.Shutdown();
    gfx.Shutdown();
    vr::VR_Shutdown();
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    // Without this Windows renders the desktop window at 96 DPI and bitmap-stretches it on scaled displays (blurry).
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    AttachParentConsole();
    std::vector<std::string> args(argv + 1, argv + argc);
    if (!args.empty() && args[0] == "--screenshot") return Screenshots(args.size() > 1 ? std::filesystem::path(args[1]) : "screenshots");
    return RunOverlay(!args.empty() && args[0] == "--unregister");
}
