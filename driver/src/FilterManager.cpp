#include "FilterManager.h"

#include "DriverLog.h"

#include "JsonUtil.h"
#include "LogRing.h"
#include "Recorder.h"

#include <windows.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace ots {
namespace {

bool ParseBool(const std::string& v, bool* out) {
    std::string s = v;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    if (s == "1" || s == "true" || s == "on") { *out = true; return true; }
    if (s == "0" || s == "false" || s == "off") { *out = false; return true; }
    return false;
}

bool ParseDouble(const std::string& v, double* out) {
    char* end = nullptr;
    const double d = std::strtod(v.c_str(), &end);
    if (end == v.c_str() || *end != '\0' || !std::isfinite(d) || d < 0) return false;
    *out = d;
    return true;
}

constexpr double kTraceInterval = 1.0 / 30; // seconds between trace samples per device
constexpr double kTraceKeep = 25;           // seconds of trace kept per device
constexpr double kTraceLinger = 5;          // keep sampling this long after the last trace request

bool IsGood(const vr::DriverPose_t& p) { return p.poseIsValid && p.result == vr::TrackingResult_Running_OK; }

// Same world space as TrackingFilter::ToWorld.
void WorldPos(const vr::DriverPose_t& p, float out[3]) {
    const vr::HmdQuaternion_t& q = p.qWorldFromDriverRotation;
    const double* v = p.vecPosition;
    const double cx = q.y * v[2] - q.z * v[1], cy = q.z * v[0] - q.x * v[2], cz = q.x * v[1] - q.y * v[0];
    const double ccx = q.y * cz - q.z * cy, ccy = q.z * cx - q.x * cz, ccz = q.x * cy - q.y * cx;
    out[0] = float(v[0] + 2 * (q.w * cx + ccx) + p.vecWorldFromDriverTranslation[0]);
    out[1] = float(v[1] + 2 * (q.w * cy + ccy) + p.vecWorldFromDriverTranslation[1]);
    out[2] = float(v[2] + 2 * (q.w * cz + ccz) + p.vecWorldFromDriverTranslation[2]);
}

char StateChar(filter::State s) {
    switch (s) {
    case filter::State::Hold: return 'H';
    case filter::State::Probation: return 'P';
    case filter::State::Blend: return 'B';
    default: return 'T';
    }
}

std::vector<std::string> ParseSerials(const std::string& csv) {
    std::vector<std::string> out;
    std::stringstream ss(csv == "-" ? std::string() : csv);
    for (std::string item; std::getline(ss, item, ',');) {
        item.erase(0, item.find_first_not_of(" \t"));
        item.erase(item.find_last_not_of(" \t") + 1);
        std::transform(item.begin(), item.end(), item.begin(), [](unsigned char c) { return char(std::toupper(c)); });
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

std::string JoinSerials(const std::vector<std::string>& v) {
    std::string s;
    for (const auto& x : v) s += (s.empty() ? "" : ",") + x;
    return s;
}

const char* ModeName(int mode) {
    switch (mode) {
    case 1: return "pending";
    case 2: return "filtered";
    case 3: return "bypass";
    default: return "unseen";
    }
}

} // namespace

FilterManager::FilterManager(Recorder* recorder, ForwardFn forward, const Settings& settings)
    : recorder_(recorder), forward_(forward), settings_(settings) {
    enabled_ = settings.enabled;
    imuEnabled_ = settings.imuEnabled;
    for (auto& s : slots_) {
        auto cfg = settings.filter;
        cfg.body.enabled = cfg.hand.enabled = false;
        s.f.SetConfig(cfg);
    }
}

FilterManager::~FilterManager() { Stop(); }

double FilterManager::Now() {
    static const double freq = [] {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        return double(f.QuadPart);
    }();
    return double(QpcNow()) / freq;
}

void FilterManager::Start() {
    if (emitter_.joinable()) return;
    stop_ = false;
    emitter_ = std::thread(&FilterManager::EmitterLoop, this);
}

void FilterManager::Stop() {
    stop_ = true;
    if (emitter_.joinable()) emitter_.join();
    for (auto& s : slots_) CloseImu(s);
}

bool FilterManager::Process(void* host, uint32_t device, const vr::DriverPose_t& in, vr::DriverPose_t* out) {
    if (device >= slots_.size()) return false;
    Slot& s = slots_[device];
    int mode = s.mode.load(std::memory_order_acquire);
    if (mode == kUnseen) {
        s.mode.compare_exchange_strong(mode, kPending); // classified by the emitter thread
        return false;
    }
    if (s.bodyReference.load(std::memory_order_acquire)) {
        std::lock_guard lock(bodyMutex_);
        bodyHead_ = in;
        bodyHeadT_ = Now();
    }
    const int handSide = s.referenceHandSide.load(std::memory_order_acquire);
    if (handSide != 0) {
        const int index = handSide < 0 ? 0 : 1;
        std::lock_guard lock(bodyMutex_);
        rawHands_[index] = in;
        rawHandTimes_[index] = Now();
    }
    const bool enabled = enabled_.load(std::memory_order_acquire);
    if (!enabled || mode != kFiltered) {
        // Passed through untouched; the trace still shows it (as lost when tracking is bad).
        if (mode == kFiltered || mode == kBypass) RecordTrace(s, Now(), &in, in, IsGood(in) ? 'T' : 'L');
        return false;
    }

    double t;
    char state;
    {
        std::lock_guard lock(s.m);
        s.host = host;
        BodyReference(s.f);
        t = Now(); // after the reference: the models refuse one from the future
        if (imuEnabled_.load(std::memory_order_relaxed)) {
            s.imu.Observe(in, t);
            if (filter::math::Quat q; s.imu.Rotation(t, &q)) s.f.SetImuRotation(q, t);
        }
        *out = s.f.Process(in, t);
        ++s.inputs;
        s.lastOut = *out;
        state = StateChar(s.f.GetState());
        FlushTransitions(device, s);
    }
    RecordTrace(s, t, &in, *out, state);
    if (recorder_) recorder_->PushFilteredPose(device, 0, out, sizeof(*out));
    return true;
}

void FilterManager::RecordTrace(Slot& s, double t, const vr::DriverPose_t* raw, const vr::DriverPose_t& out, char state) {
    if (t > traceWantedUntil_.load(std::memory_order_relaxed)) return;
    if (t - s.lastTraceT.load(std::memory_order_relaxed) < kTraceInterval) return;
    s.lastTraceT.store(t, std::memory_order_relaxed);
    TraceSample smp{};
    smp.t = t;
    smp.rawValid = raw && raw->poseIsValid && (raw->vecPosition[0] != 0 || raw->vecPosition[1] != 0 || raw->vecPosition[2] != 0);
    if (smp.rawValid) WorldPos(*raw, smp.raw);
    WorldPos(out, smp.out);
    if (!smp.rawValid) std::copy(smp.out, smp.out + 3, smp.raw);
    smp.state = state;
    std::lock_guard lock(s.traceM);
    s.trace.push_back(smp);
    while (!s.trace.empty() && s.trace.front().t < t - kTraceKeep) s.trace.pop_front();
}

void FilterManager::FlushTransitions(uint32_t device, Slot& s) {
    const int handStatus = !s.f.GetConfig().hand.enabled ? 0 : s.f.HandActive() ? 3 : s.f.HandReady() ? 2 : 1;
    if (handStatus != s.handStatus) {
        s.handStatus = handStatus;
        const std::string event = "{\"event\":\"hand_model\",\"device\":" + std::to_string(device) +
            ",\"side\":" + std::to_string(s.f.GetConfig().hand.side) + ",\"status\":\"" +
            (handStatus == 0 ? "off" : handStatus == 1 ? "waiting" : handStatus == 2 ? "hold_or_ready" : "estimating") + "\"}";
        if (recorder_) recorder_->PushText(rec::RecordType::Note,event);
        GlobalLog().Add("filter",event);
    }
    const int bodyStatus = !s.f.GetConfig().body.enabled ? 0 : s.f.BodyActive() ? 3 : s.f.BodyReady() ? 2 : 1;
    if (bodyStatus != s.bodyStatus) {
        s.bodyStatus = bodyStatus;
        const std::string event = "{\"event\":\"body_model\",\"device\":" + std::to_string(device) +
            ",\"status\":\"" + (bodyStatus == 0 ? "off" : bodyStatus == 1 ? "learning" : bodyStatus == 2 ? "hold_or_ready" : "estimating") +
            "\",\"torsoLength\":" + std::to_string(s.f.TorsoLength()) + "}";
        if (recorder_) recorder_->PushText(rec::RecordType::Note, event);
        GlobalLog().Add("filter", event);
    }
    for (const auto& tr : s.f.TakeTransitions()) {
        if (tr.to == filter::State::Hold && tr.from != filter::State::Probation) ++s.holds;
        {
            char line[256];
            const int n = std::snprintf(line, sizeof(line), "%s %s -> %s (%s)", s.serial.c_str(), filter::ToString(tr.from),
                                        filter::ToString(tr.to), tr.reason);
            GlobalLog().Add("filter", std::string_view(line, std::min(size_t(std::max(0, n)), sizeof(line) - 1)));
        }
        if (recorder_ && recorder_->IsRecording()) {
            char buf[256];
            const int n = std::snprintf(buf, sizeof(buf), "{\"device\":%u,\"from\":\"%s\",\"to\":\"%s\",\"reason\":\"%s\"}",
                                        device, filter::ToString(tr.from), filter::ToString(tr.to), tr.reason);
            recorder_->PushText(rec::RecordType::FilterEvent, std::string_view(buf, std::min(size_t(std::max(0, n)), sizeof(buf) - 1)));
        }
    }
}

void FilterManager::Classify(uint32_t device) {
    vr::CVRPropertyHelpers* props = vr::VRProperties();
    const vr::PropertyContainerHandle_t c = props->TrackedDeviceToPropertyContainer(device);
    if (c == vr::k_ulInvalidPropertyContainer) return; // try again next tick
    vr::ETrackedPropertyError err = vr::TrackedProp_Success;
    const int cls = props->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &err);
    if (err != vr::TrackedProp_Success) return;
    std::string serial = props->GetStringProperty(c, vr::Prop_SerialNumber_String);
    const bool lighthouse = props->GetStringProperty(c, vr::Prop_TrackingSystemName_String) == "lighthouse";
    std::string upper = serial;
    std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char ch) { return char(std::toupper(ch)); });

    bool filtered;
    const int role = props->GetInt32Property(c,vr::Prop_ControllerRoleHint_Int32);
    const int handSide = cls == vr::TrackedDeviceClass_Controller ?
        (role == vr::TrackedControllerRole_LeftHand ? -1 : role == vr::TrackedControllerRole_RightHand ? 1 : 0) : 0;
    filter::Config cfg;
    bool reference = false;
    {
        std::lock_guard lock(settingsMutex_);
        cfg = settings_.filter;
        cfg.hand.enabled = cfg.hand.enabled && handSide != 0;
        cfg.hand.side = handSide;
        cfg.body.enabled = cfg.body.enabled && cls == vr::TrackedDeviceClass_GenericTracker &&
                           !settings_.bodyWaistSerial.empty() && upper == settings_.bodyWaistSerial;
        // The reference HMD: the one chosen, or with none chosen the only HMD there is (a second HMD makes
        // the choice ambiguous, and then nothing is used until one is chosen).
        if (cls == vr::TrackedDeviceClass_HMD) hmdSerials_.insert(upper);
        reference = cls == vr::TrackedDeviceClass_HMD &&
                    (settings_.bodyHmdSerial.empty() ? hmdSerials_.size() == 1 : upper == settings_.bodyHmdSerial);
        auto has = [&](const std::vector<std::string>& v) { return std::find(v.begin(), v.end(), upper) != v.end(); };
        const bool included = has(settings_.includeSerials), excluded = has(settings_.excludeSerials);
        if (cls == vr::TrackedDeviceClass_HMD) {
            filtered = settings_.hmd && included && !excluded; // opt-in per HMD: a held head pose is unpleasant
        } else {
            const bool byClass = lighthouse && ((cls == vr::TrackedDeviceClass_Controller && settings_.controllers) ||
                                                (cls == vr::TrackedDeviceClass_GenericTracker && settings_.trackers));
            filtered = (byClass || included) && !excluded;
        }
    }

    Slot& s = slots_[device];
    s.bodyReference.store(reference,std::memory_order_release);
    if (s.referenceHandSide.exchange(handSide,std::memory_order_acq_rel) != handSide) {
        std::lock_guard lock(bodyMutex_);
        rawHandTimes_[0] = rawHandTimes_[1] = -1;
    }
    const bool wasFiltered = s.mode.load(std::memory_order_acquire) == kFiltered;
    vr::DriverPose_t released;
    void* host = nullptr;
    {
        std::lock_guard lock(s.m);
        const bool identityChanged = s.serial != serial || s.deviceClass != cls;
        s.serial = serial;
        s.deviceClass = cls;
        s.lighthouse = lighthouse;
        s.handSide = handSide;
        if (wasFiltered && !filtered && s.host && s.f.Release(Now(), "correction turned off for this device", &released)) host = s.host;
        if (filtered && (!wasFiltered || identityChanged)) {
            s.f = filter::TrackingFilter(cfg); // fresh state
        }
        s.f.SetConfig(cfg);
        FlushTransitions(device, s);
    }
    s.mode.store(filtered ? kFiltered : kBypass, std::memory_order_release);
    if (host) forward_(host, device, &released); // outside our lock: vrserver takes its own locks
}

void FilterManager::EmitterLoop() {
    HANDLE timer = CreateWaitableTimerExW(nullptr, nullptr, CREATE_WAITABLE_TIMER_HIGH_RESOLUTION, TIMER_ALL_ACCESS);
    double lastRoleCheck = 0;
    while (!stop_) {
        double hz;
        {
            std::lock_guard lock(settingsMutex_);
            hz = std::clamp(settings_.emitHz, 10.0, 1000.0);
        }
        if (timer) {
            LARGE_INTEGER due;
            due.QuadPart = -LONGLONG(1e7 / hz); // relative, 100 ns units
            SetWaitableTimer(timer, &due, 0, nullptr, nullptr, FALSE);
            WaitForSingleObject(timer, 100);
        } else {
            Sleep(DWORD(std::max(1.0, 1000.0 / hz)));
        }

        const bool checkRoles = Now()-lastRoleCheck >= 1;
        if (checkRoles) lastRoleCheck = Now();
        for (uint32_t d = 0; d < slots_.size(); ++d) {
            const int mode = slots_[d].mode.load(std::memory_order_acquire);
            if (mode == kPending || slots_[d].recheck.exchange(false) || (checkRoles && mode != kUnseen)) Classify(d);
        }
        if (!enabled_.load(std::memory_order_acquire)) continue;

        const bool imu = imuEnabled_.load(std::memory_order_relaxed);
        for (uint32_t d = 0; d < slots_.size(); ++d) {
            Slot& s = slots_[d];
            if (s.mode.load(std::memory_order_acquire) != kFiltered) {
                CloseImu(s);
                continue;
            }
            if (imu) PollImu(d, s, Now());
            else CloseImu(s);
            vr::DriverPose_t pose;
            void* host = nullptr;
            uint64_t inputs;
            double t;
            {
                std::lock_guard lock(s.m);
                inputs = s.inputs;
                BodyReference(s.f);
                t = Now(); // after the reference: the models refuse one from the future
                if (filter::math::Quat q; imu && s.imu.Rotation(t, &q)) s.f.SetImuRotation(q, t);
                if (!s.host || !s.f.Emit(t, &pose)) {
                    FlushTransitions(d, s);
                    continue;
                }
                host = s.host;
                ++s.emitted;
                FlushTransitions(d, s);
            }
            // Outside our lock: vrserver takes its own locks in there.
            forward_(host, d, &pose);
            // A pose from the hook (lighthouse resuming, or the device disconnecting) may have been
            // forwarded while we were building ours, and ours would then be the one SteamVR keeps:
            // send the newest real output again so it is last.
            vr::DriverPose_t newer;
            bool resend = false;
            {
                std::lock_guard lock(s.m);
                if (s.inputs != inputs) {
                    newer = s.lastOut;
                    resend = true;
                }
            }
            if (resend) forward_(host, d, &newer);
            RecordTrace(s, t, nullptr, pose, 'H'); // no input: lighthouse stopped sending
            if (recorder_) recorder_->PushFilteredPose(d, rec::kEmittedHookSlot, &pose, sizeof(pose));
        }
    }
    if (timer) CloseHandle(timer);
}

// Lighthouse publishes each device's raw IMU as an IOBuffer (driver_lighthouse.dll: "/devices/lighthouse/",
// "/imu", ImuSample_t). Opened as a reader; samples go to the device's ImuOrientation.
void FilterManager::PollImu(uint32_t device, Slot& s, double t) {
    vr::IVRIOBuffer* io = vr::VRIOBuffer();
    if (!io) return;
    if (s.imuBuffer == vr::k_ulInvalidIOBufferHandle) {
        if (t < s.imuRetryT) return;
        s.imuRetryT = t + 5;
        std::string serial;
        {
            std::lock_guard lock(s.m);
            serial = s.serial;
        }
        if (serial.empty()) return;
        const std::string path = "/devices/lighthouse/" + serial + "/imu";
        vr::IOBufferHandle_t h = vr::k_ulInvalidIOBufferHandle;
        const int err = io->Open(path.c_str(), vr::IOBufferMode_Read, sizeof(vr::ImuSample_t), 0, &h);
        if (err != s.imuError || path != s.imuPath) {
            GlobalLog().Add("ots", "IMU " + path + (err == vr::IOBuffer_Success ? ": opened" : ": open failed, error " + std::to_string(err)));
            DriverLog("IMU %s: %s (%d)", path.c_str(), err == vr::IOBuffer_Success ? "opened" : "open failed", err);
        }
        s.imuError = err;
        s.imuPath = path;
        if (err != vr::IOBuffer_Success) return;
        s.imuBuffer = h;
        s.imuAnnounced = false;
    }
    vr::ImuSample_t buf[64];
    for (int i = 0; i < 16; ++i) { // at most 1024 samples a tick
        uint32_t read = 0;
        if (io->Read(s.imuBuffer, buf, sizeof(buf), &read) != vr::IOBuffer_Success || read < sizeof(vr::ImuSample_t)) break;
        const uint32_t n = read / sizeof(vr::ImuSample_t);
        if (recorder_) recorder_->PushImu(device, buf, n, sizeof(vr::ImuSample_t));
        std::lock_guard lock(s.m);
        for (uint32_t k = 0; k < n; ++k) {
            filter::ImuSample smp;
            smp.sampleTime = buf[k].fSampleTime;
            smp.gyro = {buf[k].vGyro.v[0], buf[k].vGyro.v[1], buf[k].vGyro.v[2]};
            smp.accel = {buf[k].vAccel.v[0], buf[k].vAccel.v[1], buf[k].vAccel.v[2]};
            s.imu.Add(smp, t);
        }
        if (n < sizeof(buf) / sizeof(buf[0])) break;
    }
    std::lock_guard lock(s.m);
    if (!s.imuAnnounced && s.imu.Aligned()) {
        s.imuAnnounced = true;
        char line[200];
        std::snprintf(line, sizeof(line), "IMU %s: mounting learned (fit error %.3f, angular velocity in %s axes, %.0f samples/s)",
                      s.serial.c_str(), s.imu.FitError(), s.imu.AxesHypothesis() == 0 ? "world" : "device", s.imu.SampleRate());
        GlobalLog().Add("ots", line);
        if (recorder_) recorder_->PushText(rec::RecordType::Note, line);
        (void)device;
    }
}

void FilterManager::CloseImu(Slot& s) {
    if (s.imuBuffer == vr::k_ulInvalidIOBufferHandle) return;
    if (vr::IVRIOBuffer* io = vr::VRIOBuffer()) io->Close(s.imuBuffer);
    s.imuBuffer = vr::k_ulInvalidIOBufferHandle;
    s.imuRetryT = 0;
    std::lock_guard lock(s.m);
    s.imu.Reset();
}

void FilterManager::SetEnabled(bool enabled) {
    if (!enabled) {
        enabled_ = false;
        // Next time it is enabled, start from a clean state. A device held right now would keep its
        // last "valid" held pose while lighthouse is silent: tell SteamVR it is not tracked.
        for (uint32_t d = 0; d < slots_.size(); ++d) {
            Slot& s = slots_[d];
            vr::DriverPose_t released;
            void* host = nullptr;
            {
                std::lock_guard lock(s.m);
                if (s.mode.load(std::memory_order_acquire) == kFiltered && s.host &&
                    s.f.Release(Now(), "correction turned off", &released))
                    host = s.host;
                FlushTransitions(d, s);
                s.f = filter::TrackingFilter(s.f.GetConfig());
            }
            if (host) forward_(host, d, &released);
        }
    } else {
        enabled_ = true;
    }
}

void FilterManager::BodyReference(filter::TrackingFilter& f) {
    if (!f.GetConfig().body.enabled && !f.GetConfig().hand.enabled) return;
    std::lock_guard lock(bodyMutex_);
    f.SetBodyReference(bodyHead_,bodyHeadT_);
    if (f.GetConfig().hand.enabled) {
        const int other = f.GetConfig().hand.side < 0 ? 1 : 0;
        f.SetHandReference(bodyHead_,bodyHeadT_,rawHands_[other],rawHandTimes_[other]);
    }
}

void FilterManager::ApplyFilterConfig() {
    filter::Config cfg;
    std::string waist;
    {
        std::lock_guard lock(settingsMutex_);
        cfg = settings_.filter;
        waist = settings_.bodyWaistSerial;
    }
    for (auto& s : slots_) {
        std::lock_guard lock(s.m);
        auto deviceCfg = cfg;
        std::string upper = s.serial;
        std::transform(upper.begin(),upper.end(),upper.begin(),[](unsigned char ch) { return char(std::toupper(ch)); });
        deviceCfg.body.enabled = cfg.body.enabled && s.deviceClass == vr::TrackedDeviceClass_GenericTracker && !waist.empty() && upper == waist;
        deviceCfg.hand.enabled = cfg.hand.enabled && s.deviceClass == vr::TrackedDeviceClass_Controller && s.handSide != 0;
        deviceCfg.hand.side = s.handSide;
        s.f.SetConfig(deviceCfg);
    }
}

void FilterManager::Reclassify() {
    // Devices keep their current mode until the emitter thread has classified them again, so a filtered
    // device that stays filtered never passes a raw pose through (or loses its hold) in between.
    for (auto& s : slots_) {
        const int mode = s.mode.load();
        if (mode == kFiltered || mode == kBypass) s.recheck = true;
    }
}

bool FilterManager::Set(const std::string& key, const std::string& value, std::string* error) {
    bool b = false;
    double d = 0;
    auto needBool = [&] {
        if (!ParseBool(value, &b)) { *error = "expected true/false"; return false; }
        return true;
    };
    auto needNum = [&] {
        if (!ParseDouble(value, &d)) { *error = "expected a non-negative number"; return false; }
        return true;
    };

    if (key == "filterEnabled") {
        if (!needBool()) return false;
        SetEnabled(b);
        return true;
    }
    if (key == "imuEnabled") {
        if (!needBool()) return false;
        {
            std::lock_guard lock(settingsMutex_);
            settings_.imuEnabled = b;
        }
        imuEnabled_ = b;
        return true;
    }
    if (key == "bodyEnabled" || key == "bodyWaistSerial" || key == "bodyHmdSerial") {
        if (key == "bodyEnabled" && !needBool()) return false;
        const auto serials = ParseSerials(value);
        if (key != "bodyEnabled" && serials.size() > 1) { *error = "expected one serial or -"; return false; }
        {
            std::lock_guard lock(settingsMutex_);
            if (key == "bodyEnabled") settings_.filter.body.enabled = b;
            else (key == "bodyWaistSerial" ? settings_.bodyWaistSerial : settings_.bodyHmdSerial) = serials.empty() ? "" : serials.front();
        }
        if (key != "bodyEnabled") {
            // A shared HMD change invalidates both models. A waist assignment does
            // not interrupt hand compensation or clear its reference.
            if (key == "bodyHmdSerial") {
                std::lock_guard lock(bodyMutex_); bodyHead_ = {}; bodyHeadT_ = -1;
            }
            for (auto& slot : slots_) {
                if (key == "bodyHmdSerial") slot.bodyReference = false;
                std::lock_guard lock(slot.m);
                auto c = slot.f.GetConfig();
                c.body.enabled = false;
                if (key == "bodyHmdSerial") c.hand.enabled = false;
                slot.f.SetConfig(c);
            }
        }
        ApplyFilterConfig();
        Reclassify();
        return true;
    }
    if (key == "handEnabled") {
        if (!needBool()) return false;
        { std::lock_guard lock(settingsMutex_); settings_.filter.hand.enabled = b; }
        ApplyFilterConfig();
        return true;
    }
    if (key == "filterControllers" || key == "filterTrackers" || key == "filterHmd") {
        if (!needBool()) return false;
        {
            std::lock_guard lock(settingsMutex_);
            (key == "filterControllers" ? settings_.controllers : key == "filterTrackers" ? settings_.trackers : settings_.hmd) = b;
        }
        Reclassify();
        return true;
    }
    if (key == "filterIncludeSerials" || key == "filterExcludeSerials") { // comma separated, "-" = none
        {
            std::lock_guard lock(settingsMutex_);
            (key == "filterIncludeSerials" ? settings_.includeSerials : settings_.excludeSerials) = ParseSerials(value);
        }
        Reclassify();
        return true;
    }
    if (key == "smoothing") { // percent
        if (!needNum()) return false;
        {
            std::lock_guard lock(settingsMutex_);
            settings_.filter.smoothing = std::clamp(d, 0.0, 100.0) / 100.0;
        }
        ApplyFilterConfig();
        return true;
    }
    {
        std::lock_guard lock(settingsMutex_);
        filter::Config& c = settings_.filter;
        if (key == "suspectEnabled" || key == "holdReportsValid") {
            if (!needBool()) return false;
            (key == "suspectEnabled" ? c.suspectEnabled : c.holdReportsValid) = b;
        } else if (key == "emitHz") {
            if (!needNum()) return false;
            settings_.emitHz = d;
            return true;
        } else {
            struct Num { const char* key; double* field; double scale; };
            const Num nums[] = {{"suspectWindowMs", &c.suspectWindow, 1e-3}, {"streamTimeoutMs", &c.streamTimeout, 1e-3},
                                {"handShoulderWidthCm", &c.hand.shoulderWidth, .01}, {"handArmReachCm", &c.hand.armReach, .01},
                                {"handMaxPredictionMs", &c.hand.maxPrediction, .001},
                                {"handArmFollowPct", &c.hand.armFollow, .01},
                                {"handFollowDeadzoneCm", &c.hand.followDeadzone, .01},
                                {"probationMs", &c.probation, 1e-3},       {"shortProbationMs", &c.shortProbation, 1e-3},
                                {"blendMs", &c.blend, 1e-3},
                                {"bodyNeckDownCm", &c.body.neckDown, 1e-2}, {"bodyNeckBackCm", &c.body.neckBack, 1e-2},
                                {"stepLimitCm", &c.stepLimit, 1e-2},       {"stepBlendMs", &c.stepBlend, 1e-3},
                                {"maxHoldSec", &c.maxHold, 1.0}};
            const Num* n = std::find_if(std::begin(nums), std::end(nums), [&](const Num& x) { return key == x.key; });
            if (n == std::end(nums)) { *error = "unknown key"; return false; }
            if (!needNum()) return false;
            if ((key == "bodyNeckDownCm" || key == "bodyNeckBackCm") && d > 30) { *error = "expected 0..30 cm"; return false; }
            if ((key == "handShoulderWidthCm" && (d < 20 || d > 60)) ||
                (key == "handArmReachCm" && (d < 30 || d > 120)) ||
                (key == "handMaxPredictionMs" && (d < 100 || d > 3000)) ||
                (key == "handArmFollowPct" && (d < 0 || d > 100)) ||
                (key == "handFollowDeadzoneCm" && (d < 0 || d > 50))) {
                *error = "hand dimensions or duration outside supported range"; return false;
            }
            *n->field = d * n->scale;
        }
    }
    ApplyFilterConfig();
    return true;
}

std::string FilterManager::SettingsJson() {
    std::lock_guard lock(settingsMutex_);
    const Settings& s = settings_;
    const filter::Config& c = s.filter;
    std::ostringstream o;
    o << "{\"filterEnabled\":" << (enabled_ ? "true" : "false") << ",\"filterControllers\":" << (s.controllers ? "true" : "false")
      << ",\"filterTrackers\":" << (s.trackers ? "true" : "false") << ",\"filterHmd\":" << (s.hmd ? "true" : "false")
      << ",\"suspectEnabled\":" << (c.suspectEnabled ? "true" : "false") << ",\"suspectWindowMs\":" << c.suspectWindow * 1e3
      << ",\"streamTimeoutMs\":" << c.streamTimeout * 1e3 << ",\"probationMs\":" << c.probation * 1e3
      << ",\"shortProbationMs\":" << c.shortProbation * 1e3
      << ",\"blendMs\":" << c.blend * 1e3 << ",\"stepLimitCm\":" << c.stepLimit * 1e2 << ",\"stepBlendMs\":" << c.stepBlend * 1e3
      << ",\"holdReportsValid\":" << (c.holdReportsValid ? "true" : "false") << ",\"maxHoldSec\":" << c.maxHold
      << ",\"emitHz\":" << s.emitHz << ",\"smoothing\":" << c.smoothing * 100
      << ",\"imuEnabled\":" << (s.imuEnabled ? "true" : "false")
      << ",\"bodyEnabled\":" << (c.body.enabled ? "true" : "false")
      << ",\"handEnabled\":" << (c.hand.enabled ? "true" : "false")
      << ",\"handShoulderWidthCm\":" << c.hand.shoulderWidth*100 << ",\"handArmReachCm\":" << c.hand.armReach*100
      << ",\"handMaxPredictionMs\":" << c.hand.maxPrediction*1000 << ",\"handArmFollowPct\":" << c.hand.armFollow*100
      << ",\"handFollowDeadzoneCm\":" << c.hand.followDeadzone*100
      << ",\"bodyWaistSerial\":" << JsonEscape(s.bodyWaistSerial) << ",\"bodyHmdSerial\":" << JsonEscape(s.bodyHmdSerial)
      << ",\"bodyHmdAuto\":" << JsonEscape(s.bodyHmdSerial.empty() && hmdSerials_.size() == 1 ? *hmdSerials_.begin() : std::string())
      << ",\"bodyNeckDownCm\":" << c.body.neckDown*100 << ",\"bodyNeckBackCm\":" << c.body.neckBack*100
      << ",\"filterIncludeSerials\":" << JsonEscape(JoinSerials(s.includeSerials))
      << ",\"filterExcludeSerials\":" << JsonEscape(JoinSerials(s.excludeSerials)) << "}";
    return o.str();
}

std::string FilterManager::TraceJson(double since) {
    const double now = Now();
    traceWantedUntil_.store(now + kTraceLinger, std::memory_order_relaxed);
    std::string o;
    o.reserve(64 * 1024);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "{\"ok\":true,\"now\":%.3f,\"devices\":[", now);
    o += buf;
    bool firstDevice = true;
    for (uint32_t d = 0; d < slots_.size(); ++d) {
        Slot& s = slots_[d];
        const int mode = s.mode.load();
        if (mode != kFiltered && mode != kBypass) continue;
        std::string serial;
        {
            std::lock_guard lock(s.m);
            serial = s.serial;
        }
        std::lock_guard lock(s.traceM);
        auto it = std::upper_bound(s.trace.begin(), s.trace.end(), since, [](double v, const TraceSample& x) { return v < x.t; });
        if (it == s.trace.end()) continue;
        o += firstDevice ? "" : ",";
        o += "{\"index\":" + std::to_string(d) + ",\"serial\":" + JsonEscape(serial) + ",\"s\":[";
        firstDevice = false;
        // Positions in millimeters keep the reply small.
        for (bool first = true; it != s.trace.end(); ++it, first = false) {
            const TraceSample& x = *it;
            std::snprintf(buf, sizeof(buf), "%s[%.3f,%d,%d,%d,%d,%d,%d,%d,\"%c\"]", first ? "" : ",", x.t, int(std::lround(x.raw[0] * 1000)),
                          int(std::lround(x.raw[1] * 1000)), int(std::lround(x.raw[2] * 1000)), int(std::lround(x.out[0] * 1000)),
                          int(std::lround(x.out[1] * 1000)), int(std::lround(x.out[2] * 1000)), x.rawValid ? 1 : 0, x.state);
            o += buf;
        }
        o += "]}";
    }
    o += "]}";
    return o;
}

std::string FilterManager::ModelJson() {
    const double now = Now();
    std::ostringstream o;
    o << "{\"ok\":true,\"devices\":[";
    bool first = true;
    auto mm = [](const filter::math::Vec& v) {
        return "[" + std::to_string(std::lround(v.x * 1000)) + "," + std::to_string(std::lround(v.y * 1000)) + "," +
               std::to_string(std::lround(v.z * 1000)) + "]";
    };
    for (uint32_t d = 0; d < slots_.size(); ++d) {
        Slot& s = slots_[d];
        if (s.mode.load() != kFiltered) continue;
        std::lock_guard lock(s.m);
        const auto& c = s.f.GetConfig();
        const filter::ModelDebug* m = c.hand.enabled ? &s.f.HandDebug() : c.body.enabled ? &s.f.BodyDebug() : nullptr;
        if (!m || m->t < 0 || now - m->t > .25) continue; // stale: the model is not working on this device now
        o << (first ? "" : ",") << "{\"index\":" << d << ",\"serial\":" << JsonEscape(s.serial) << ",\"kind\":\""
          << (c.hand.enabled ? "hand" : "waist") << "\",\"estimating\":" << (m->estimating ? "true" : "false")
          << ",\"neck\":" << mm(m->neck) << ",\"end\":" << mm(m->end);
        if (m->haveShoulder) o << ",\"shoulder\":" << mm(m->shoulder);
        if (m->haveElbow) o << ",\"elbow\":" << mm(m->elbow);
        o << "}";
        first = false;
    }
    o << "]}";
    return o.str();
}

std::string FilterManager::DevicesJson() {
    const double now = Now();
    std::ostringstream o;
    o << "[";
    bool first = true;
    for (uint32_t d = 0; d < slots_.size(); ++d) {
        Slot& s = slots_[d];
        const int mode = s.mode.load();
        if (mode == kUnseen) continue;
        std::lock_guard lock(s.m);
        o << (first ? "" : ",") << "{\"index\":" << d << ",\"serial\":" << JsonEscape(s.serial) << ",\"class\":" << s.deviceClass
          << ",\"mode\":\"" << ModeName(mode) << "\"" << ",\"handSide\":" << s.handSide
          << ",\"lighthouse\":" << (s.lighthouse ? "true" : "false");
        if (mode == kFiltered)
            o << ",\"state\":\"" << filter::ToString(s.f.GetState()) << "\",\"stateSeconds\":" << (now - s.f.StateSince())
              << ",\"bodyReady\":" << (s.f.BodyReady() ? "true" : "false") << ",\"bodyActive\":" << (s.f.BodyActive() ? "true" : "false")
              << ",\"handReady\":" << (s.f.HandReady() ? "true" : "false") << ",\"handActive\":" << (s.f.HandActive() ? "true" : "false")
              << ",\"torsoLengthCm\":" << s.f.TorsoLength()*100
              << ",\"imuOpen\":" << (s.imuBuffer != vr::k_ulInvalidIOBufferHandle ? "true" : "false") << ",\"imuError\":" << s.imuError
              << ",\"imuRate\":" << s.imu.SampleRate() << ",\"imuAligned\":" << (s.imu.Aligned() ? "true" : "false")
              << ",\"imuFitError\":" << s.imu.FitError() << ",\"imuActive\":" << (s.f.ImuActive() ? "true" : "false")
              << ",\"holds\":" << s.holds << ",\"emitted\":" << s.emitted << ",\"stepsLimited\":" << s.f.StepsLimited();
        o << "}";
        first = false;
    }
    o << "]";
    return o.str();
}

} // namespace ots
