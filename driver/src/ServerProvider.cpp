#include "ServerProvider.h"

#include "DriverLog.h"
#include "Hooks.h"
#include "JsonUtil.h"
#include "LogRing.h"
#include "OtsConstants.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace ots {
namespace {

bool GetBoolSetting(const char* key, bool fallback) {
    vr::EVRSettingsError err = vr::VRSettingsError_None;
    const bool v = vr::VRSettings()->GetBool(kSettingsSection, key, &err);
    return err == vr::VRSettingsError_None ? v : fallback;
}

double GetNumberSetting(const char* key, double fallback) {
    vr::EVRSettingsError err = vr::VRSettingsError_None;
    const float v = vr::VRSettings()->GetFloat(kSettingsSection, key, &err);
    return err == vr::VRSettingsError_None && std::isfinite(v) ? double(v) : fallback;
}

std::string GetStringSetting(const char* key) {
    char buf[1024] = {};
    vr::EVRSettingsError err = vr::VRSettingsError_None;
    vr::VRSettings()->GetString(kSettingsSection, key, buf, sizeof(buf), &err);
    return err == vr::VRSettingsError_None ? std::string(buf) : std::string();
}

std::filesystem::path DefaultRecordDir() {
    const wchar_t* local = _wgetenv(L"LOCALAPPDATA");
    std::filesystem::path base = local ? std::filesystem::path(local) : std::filesystem::temp_directory_path();
    return base / L"OpenVR-TrackingStabilizer" / L"sessions";
}

std::string_view Trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.remove_suffix(1);
    return s;
}

std::string Error(std::string_view message) { return "{\"ok\":false,\"error\":" + JsonEscape(message) + "}"; }

} // namespace

FilterManager::Settings ServerProvider::ReadFilterSettings() {
    FilterManager::Settings s;
    s.enabled = GetBoolSetting("filterEnabled", s.enabled);
    s.controllers = GetBoolSetting("filterControllers", s.controllers);
    s.trackers = GetBoolSetting("filterTrackers", s.trackers);
    s.hmd = GetBoolSetting("filterHmd", s.hmd);
    s.emitHz = GetNumberSetting("emitHz", s.emitHz);
    auto serials = [](const char* key) {
        std::vector<std::string> out;
        std::stringstream ss(GetStringSetting(key));
        for (std::string item; std::getline(ss, item, ',');) {
            item = std::string(Trim(item));
            for (char& c : item) c = char(std::toupper(static_cast<unsigned char>(c)));
            if (!item.empty()) out.push_back(item);
        }
        return out;
    };
    s.includeSerials = serials("filterIncludeSerials");
    s.excludeSerials = serials("filterExcludeSerials");
    const auto waist = serials("bodyWaistSerial"), head = serials("bodyHmdSerial");
    if (waist.size() == 1) s.bodyWaistSerial = waist.front();
    if (head.size() == 1) s.bodyHmdSerial = head.front();
    filter::Config& c = s.filter;
    c.smoothing = std::clamp(GetNumberSetting("smoothing", 0), 0.0, 100.0) / 100.0;
    c.body.enabled = GetBoolSetting("bodyEnabled", false);
    s.imuEnabled = GetBoolSetting("imuEnabled", false);
    c.hand.enabled = GetBoolSetting("handEnabled", false);
    c.hand.shoulderWidth = std::clamp(GetNumberSetting("handShoulderWidthCm",36),20.0,60.0)*.01;
    c.hand.armReach = std::clamp(GetNumberSetting("handArmReachCm",70),30.0,120.0)*.01;
    c.hand.maxPrediction = std::clamp(GetNumberSetting("handMaxPredictionMs",3000),100.0,3000.0)*.001;
    c.hand.armFollow = std::clamp(GetNumberSetting("handArmFollowPct",c.hand.armFollow*100),0.0,100.0)*.01;
    c.hand.followDeadzone = std::clamp(GetNumberSetting("handFollowDeadzoneCm",c.hand.followDeadzone*100),0.0,50.0)*.01;
    c.body.neckDown = std::clamp(GetNumberSetting("bodyNeckDownCm", 12),0.0,30.0)*.01;
    c.body.neckBack = std::clamp(GetNumberSetting("bodyNeckBackCm", 8),0.0,30.0)*.01;
    c.suspectEnabled = GetBoolSetting("suspectEnabled", c.suspectEnabled);
    c.suspectWindow = GetNumberSetting("suspectWindowMs", c.suspectWindow * 1e3) * 1e-3;
    c.streamTimeout = GetNumberSetting("streamTimeoutMs", c.streamTimeout * 1e3) * 1e-3;
    c.probation = GetNumberSetting("probationMs", c.probation * 1e3) * 1e-3;
    c.shortProbation = GetNumberSetting("shortProbationMs", c.shortProbation * 1e3) * 1e-3;
    c.blend = GetNumberSetting("blendMs", c.blend * 1e3) * 1e-3;
    c.stepLimit = GetNumberSetting("stepLimitCm", c.stepLimit * 1e2) * 1e-2;
    c.stepBlend = GetNumberSetting("stepBlendMs", c.stepBlend * 1e3) * 1e-3;
    c.holdReportsValid = GetBoolSetting("holdReportsValid", c.holdReportsValid);
    c.maxHold = GetNumberSetting("maxHoldSec", c.maxHold);
    return s;
}

vr::EVRInitError ServerProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    DriverLog("OpenVR-TrackingStabilizer Phase1 (tracking-loss filter) starting");

    const std::string dir = GetStringSetting("recordDir");
    recordDir_ = dir.empty() ? DefaultRecordDir()
                             : std::filesystem::path(std::u8string(dir.begin(), dir.end())); // settings are UTF-8

    recorder_.onNewDevice = [this](uint32_t d) { WriteDeviceInfo(d); };
    recorder_.onTick = [this] { WriteRawPoseSnapshot(); };

    HookOptions options;
    options.poses = GetBoolSetting("hookPoses", true);
    options.driverLog = GetBoolSetting("hookDriverLog", true);
    if (options.poses || options.driverLog) {
        hooksActive_ = InstallHooks(pDriverContext, &recorder_, options);
        if (!hooksActive_) DriverLog("no hooks installed; recordings will only contain snapshots");
    }

    if (PoseHookInstalled()) {
        const FilterManager::Settings fs = ReadFilterSettings();
        filter_ = std::make_unique<FilterManager>(&recorder_, &ForwardPose, fs);
        filter_->Start();
        SetPoseFilter(filter_.get());
        DriverLog("filter %s: %s", fs.enabled ? "enabled" : "disabled (filterEnabled=false)", filter_->SettingsJson().c_str());
    } else {
        DriverLog("filter not started: pose hook is not installed");
    }

    pipe_.Start(kPipeName, [this](std::string_view line) { return HandleCommand(line); });

    if (GetBoolSetting("autoRecord", false)) {
        const std::string result = StartRecording("auto");
        DriverLog("autoRecord: %s", result.c_str());
    }
    return vr::VRInitError_None;
}

void ServerProvider::Cleanup() {
    // Order matters: no client may restart recording, the emitter must stop before the hook's
    // trampoline goes away, no detour may touch the recorder after it stops, and nothing of ours
    // may run after the DLL is unloaded.
    pipe_.Stop();
    SetPoseFilter(nullptr);
    if (filter_) filter_->Stop();
    if (hooksActive_) {
        RemoveHooks();
        hooksActive_ = false;
    }
    if (recorder_.IsRecording()) {
        recorder_.PushText(rec::RecordType::Note, "recording stopped: driver cleanup");
        recorder_.Stop();
    }
    filter_.reset(); // no detour can reach it any more (RemoveHooks waited for in-flight calls)
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

std::string ServerProvider::HandleCommand(std::string_view line) {
    line = Trim(line);
    const size_t sp = line.find(' ');
    const std::string_view cmd = line.substr(0, sp);
    const std::string_view arg = sp == std::string_view::npos ? std::string_view() : Trim(line.substr(sp + 1));

    if (cmd == "ping") return "{\"ok\":true}";
    if (cmd == "status") return StatusJson();
    if (cmd == "start") return StartRecording(arg);
    if (cmd == "stop") {
        if (!recorder_.IsRecording()) return Error("not recording");
        recorder_.PushText(rec::RecordType::Note, "recording stopped: client request");
        const std::string path = recorder_.GetStats().path;
        recorder_.Stop();
        DriverLog("recording stopped: %s", path.c_str());
        return "{\"ok\":true,\"path\":" + JsonEscape(path) + "}";
    }
    if (cmd == "mark") {
        if (!recorder_.IsRecording()) return Error("not recording");
        if (arg.empty()) return Error("mark needs a label");
        recorder_.PushText(rec::RecordType::Marker, arg);
        return "{\"ok\":true}";
    }
    if (cmd == "log") {
        const uint64_t since = arg.empty() ? 0 : std::strtoull(std::string(arg).c_str(), nullptr, 10);
        return GlobalLog().JsonSince(since, 500);
    }
    if (cmd == "filter" || cmd == "devices" || cmd == "config" || cmd == "set" || cmd == "trace" || cmd == "model") {
        if (!filter_) return Error("filter not running (pose hook not installed)");
        if (cmd == "devices") return "{\"ok\":true,\"devices\":" + filter_->DevicesJson() + "}";
        if (cmd == "trace") return filter_->TraceJson(arg.empty() ? 0.0 : std::strtod(std::string(arg).c_str(), nullptr));
        if (cmd == "model") return filter_->ModelJson();
        if (cmd == "config") return "{\"ok\":true,\"config\":" + filter_->SettingsJson() + "}";
        if (cmd == "filter") {
            if (arg == "on" || arg == "off") {
                filter_->SetEnabled(arg == "on");
                DriverLog("filter %s by client", arg == "on" ? "enabled" : "disabled");
            } else if (!arg.empty()) {
                return Error("usage: filter [on|off]");
            }
            return std::string("{\"ok\":true,\"enabled\":") + (filter_->Enabled() ? "true" : "false") + "}";
        }
        // set <key> <value>
        const size_t sp2 = arg.find(' ');
        if (sp2 == std::string_view::npos) return Error("usage: set <key> <value>");
        const std::string key(Trim(arg.substr(0, sp2))), value(Trim(arg.substr(sp2 + 1)));
        std::string error;
        if (!filter_->Set(key, value, &error)) return Error(key + ": " + error);
        recorder_.PushText(rec::RecordType::Note, "{\"event\":\"filter_config_changed\",\"filterConfig\":" + filter_->SettingsJson() + "}");
        DriverLog("set %s=%s by client", key.c_str(), value.c_str());
        return "{\"ok\":true,\"config\":" + filter_->SettingsJson() + "}";
    }
    return Error("unknown command (ping|status|start [label]|stop|mark <label>|filter [on|off]|devices|config|set <key> <value>|log <seq>|trace <since>|model)");
}

std::string ServerProvider::StartRecording(std::string_view label) {
    std::string pathOrError;
    if (!recorder_.Start(recordDir_, label, &pathOrError)) return Error(pathOrError);

    const HookStatus hs = GetHookStatus();
    std::ostringstream note;
    note << "{\"event\":\"recording_started\",\"label\":" << JsonEscape(label) << ",\"hooks\":{";
    for (int i = 0; i < HookStatus::kPoseSlots; ++i)
        note << "\"pose" << i << "\":{\"interface\":" << JsonEscape(hs.pose[i].interfaceName)
             << ",\"installed\":" << (hs.pose[i].installed ? "true" : "false")
             << ",\"shared\":" << (hs.pose[i].sharedWithPrevious ? "true" : "false") << "},";
    note << "\"log\":" << (hs.logInstalled ? "true" : "false") << "}";
    if (filter_)
        note << ",\"filterEnabled\":" << (filter_->Enabled() ? "true" : "false") << ",\"filterConfig\":" << filter_->SettingsJson()
             << ",\"filterDevices\":" << filter_->DevicesJson();
    note << "}";
    recorder_.PushText(rec::RecordType::Note, note.str());

    // Devices that never send a pose (e.g. base stations) are still described.
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) WriteDeviceInfo(i);

    DriverLog("recording started: %s", pathOrError.c_str());
    return "{\"ok\":true,\"path\":" + JsonEscape(pathOrError) + "}";
}

std::string ServerProvider::StatusJson() {
    const Recorder::Stats s = recorder_.GetStats();
    const HookStatus hs = GetHookStatus();
    std::ostringstream o;
    o << "{\"ok\":true,\"recording\":" << (s.recording ? "true" : "false")
      << ",\"path\":" << JsonEscape(s.path)
      << ",\"seconds\":" << s.seconds
      << ",\"poses\":" << s.poses << ",\"logs\":" << s.logs << ",\"markers\":" << s.markers
      << ",\"dropped\":" << s.dropped << ",\"bytes\":" << s.bytesWritten
      << ",\"hooks\":{";
    for (int i = 0; i < HookStatus::kPoseSlots; ++i)
        o << "\"pose" << i << "\":" << JsonEscape(hs.pose[i].installed ? hs.pose[i].interfaceName
                                                   : hs.pose[i].sharedWithPrevious ? "shared" : "off") << ",";
    o << "\"log\":" << (hs.logInstalled ? "true" : "false") << "}";
    o << ",\"filter\":" << (!filter_ ? "\"not running\"" : filter_->Enabled() ? "\"on\"" : "\"off\"") << "}";
    return o.str();
}

void ServerProvider::WriteDeviceInfo(uint32_t deviceIndex) {
    vr::CVRPropertyHelpers* props = vr::VRProperties();
    const vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(deviceIndex);
    if (c == vr::k_ulInvalidPropertyContainer) return;

    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    const int32_t deviceClass = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &err);
    if (err != vr::TrackedProp_Success || deviceClass == vr::TrackedDeviceClass_Invalid) return;

    std::ostringstream o;
    o << "{\"index\":" << deviceIndex
      << ",\"class\":" << deviceClass
      << ",\"serial\":" << JsonEscape(props->GetStringProperty(c, vr::Prop_SerialNumber_String))
      << ",\"model\":" << JsonEscape(props->GetStringProperty(c, vr::Prop_ModelNumber_String))
      << ",\"manufacturer\":" << JsonEscape(props->GetStringProperty(c, vr::Prop_ManufacturerName_String))
      << ",\"trackingSystem\":" << JsonEscape(props->GetStringProperty(c, vr::Prop_TrackingSystemName_String))
      << ",\"role\":" << props->GetInt32Property(c, vr::Prop_ControllerRoleHint_Int32)
      << "}";
    recorder_.PushText(rec::RecordType::DeviceInfo, o.str());
}

void ServerProvider::WriteRawPoseSnapshot() {
    vr::IVRServerDriverHost* host = vr::VRServerDriverHost();
    if (!host) return;
    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount] = {};
    host->GetRawTrackedDevicePoses(0.0f, poses, vr::k_unMaxTrackedDeviceCount);
    recorder_.PushRawPoses(poses, vr::k_unMaxTrackedDeviceCount, sizeof(vr::TrackedDevicePose_t));
}

} // namespace ots
