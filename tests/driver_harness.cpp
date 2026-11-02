// Loads driver_trackingstabilizer.dll into a fake vrserver and checks the Phase 1 path end to end:
// hook install on IVRServerDriverHost_006, device classification, pose filtering, the emitter
// while the input stream is stopped, recovery, the control pipe, and a clean shutdown.
//
// Usage: driver_harness <path to driver_trackingstabilizer.dll>
// The pipe test is skipped while SteamVR is running (the real driver owns the pipe name).
#include <openvr_driver.h>

#include <windows.h>
#include <tlhelp32.h>
#include <timeapi.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

int g_failures = 0;
#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if (!(cond)) {                                                      \
            ++g_failures;                                                   \
            std::printf("  FAIL %s:%d: %s  ", __FILE__, __LINE__, #cond);   \
            std::printf(__VA_ARGS__);                                       \
            std::printf("\n");                                              \
        }                                                                   \
    } while (0)

using Clock = std::chrono::steady_clock;

struct Received {
    double t;
    uint32_t device;
    vr::DriverPose_t pose;
};

// ---- fake vrserver interfaces ----

class FakeHost : public vr::IVRServerDriverHost {
public:
    std::mutex m;
    std::vector<Received> received;
    Clock::time_point start = Clock::now();

    bool TrackedDeviceAdded(const char*, vr::ETrackedDeviceClass, vr::ITrackedDeviceServerDriver*) override { return false; }
    void TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t) override {
        const double t = std::chrono::duration<double>(Clock::now() - start).count();
        std::lock_guard lock(m);
        received.push_back({t, unWhichDevice, newPose});
    }
    void VsyncEvent(double) override {}
    void VendorSpecificEvent(uint32_t, vr::EVREventType, const vr::VREvent_Data_t&, double) override {}
    bool IsExiting() override { return false; }
    bool PollNextEvent(vr::VREvent_t*, uint32_t) override { return false; }
    void GetRawTrackedDevicePoses(float, vr::TrackedDevicePose_t* poses, uint32_t count) override {
        std::memset(poses, 0, sizeof(*poses) * count);
    }
    void RequestRestart(const char*, const char*, const char*, const char*) override {}
    uint32_t GetFrameTimings(vr::Compositor_FrameTiming*, uint32_t) override { return 0; }
    void SetDisplayEyeToHead(uint32_t, const vr::HmdMatrix34_t&, const vr::HmdMatrix34_t&) override {}
    void SetDisplayProjectionRaw(uint32_t, const vr::HmdRect2_t&, const vr::HmdRect2_t&) override {}
    void SetRecommendedRenderTargetSize(uint32_t, uint32_t, uint32_t) override {}
};

class FakeLog : public vr::IVRDriverLog {
public:
    void Log(const char* msg) override { std::printf("    [driver] %s\n", msg); }
};

class FakeSettings : public vr::IVRSettings {
public:
    const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError) override { return "unset"; }
    void SetBool(const char*, const char*, bool, vr::EVRSettingsError*) override {}
    void SetInt32(const char*, const char*, int32_t, vr::EVRSettingsError*) override {}
    void SetFloat(const char*, const char*, float, vr::EVRSettingsError*) override {}
    void SetString(const char*, const char*, const char*, vr::EVRSettingsError*) override {}
    // Everything unset: the driver falls back to its compiled-in defaults.
    bool GetBool(const char*, const char*, vr::EVRSettingsError* e) override { return Unset(e), false; }
    int32_t GetInt32(const char*, const char*, vr::EVRSettingsError* e) override { return Unset(e), 0; }
    float GetFloat(const char*, const char*, vr::EVRSettingsError* e) override { return Unset(e), 0.0f; }
    void GetString(const char*, const char*, char* v, uint32_t n, vr::EVRSettingsError* e) override {
        if (v && n) v[0] = 0;
        Unset(e);
    }
    void RemoveSection(const char*, vr::EVRSettingsError*) override {}
    void RemoveKeyInSection(const char*, const char*, vr::EVRSettingsError*) override {}

private:
    static void Unset(vr::EVRSettingsError* e) {
        if (e) *e = vr::VRSettingsError_UnsetSettingHasNoDefault;
    }
};

// Device 1 = HMD, 3 = tracker, 4 / 5 = controllers, 6 = a controller of another tracking system.
// Container handle = index + 1.
class FakeProperties : public vr::IVRProperties {
public:
    vr::ETrackedPropertyError ReadPropertyBatch(vr::PropertyContainerHandle_t c, vr::PropertyRead_t* b, uint32_t n) override {
        const uint32_t dev = uint32_t(c - 1);
        for (uint32_t i = 0; i < n; ++i) {
            vr::PropertyRead_t& r = b[i];
            if (r.prop == vr::Prop_DeviceClass_Int32) {
                const int32_t cls = dev == 1 ? vr::TrackedDeviceClass_HMD
                                    : dev == 3 ? vr::TrackedDeviceClass_GenericTracker
                                    : (dev == 4 || dev == 5 || dev == 6) ? vr::TrackedDeviceClass_Controller
                                               : vr::TrackedDeviceClass_Invalid;
                r.unTag = vr::k_unInt32PropertyTag;
                r.unRequiredBufferSize = sizeof(cls);
                if (r.pvBuffer && r.unBufferSize >= sizeof(cls)) std::memcpy(r.pvBuffer, &cls, sizeof(cls));
                r.eError = vr::TrackedProp_Success;
            } else if (r.prop == vr::Prop_ControllerRoleHint_Int32) {
                const int32_t role = dev == 4 ? vr::TrackedControllerRole_RightHand : dev == 5 ? vr::TrackedControllerRole_LeftHand : vr::TrackedControllerRole_Invalid;
                r.unTag = vr::k_unInt32PropertyTag; r.unRequiredBufferSize = sizeof(role);
                if (r.pvBuffer && r.unBufferSize >= sizeof(role)) std::memcpy(r.pvBuffer,&role,sizeof(role));
                r.eError = vr::TrackedProp_Success;
            } else if (r.prop == vr::Prop_SerialNumber_String || r.prop == vr::Prop_TrackingSystemName_String) {
                const std::string s = r.prop == vr::Prop_SerialNumber_String ? "TEST-" + std::to_string(dev)
                                      : dev == 6 ? "oculus" : "lighthouse";
                r.unTag = vr::k_unStringPropertyTag;
                r.unRequiredBufferSize = uint32_t(s.size() + 1);
                if (r.pvBuffer && r.unBufferSize >= s.size() + 1) {
                    std::memcpy(r.pvBuffer, s.c_str(), s.size() + 1);
                    r.eError = vr::TrackedProp_Success;
                } else {
                    r.eError = vr::TrackedProp_BufferTooSmall;
                }
            } else {
                r.unTag = vr::k_unInvalidPropertyTag;
                r.eError = vr::TrackedProp_UnknownProperty;
            }
        }
        return vr::TrackedProp_Success;
    }
    vr::ETrackedPropertyError WritePropertyBatch(vr::PropertyContainerHandle_t, vr::PropertyWrite_t*, uint32_t) override {
        return vr::TrackedProp_Success;
    }
    const char* GetPropErrorNameFromEnum(vr::ETrackedPropertyError) override { return "?"; }
    vr::PropertyContainerHandle_t TrackedDeviceToPropertyContainer(vr::TrackedDeviceIndex_t d) override { return d + 1; }
};

class FakeContext : public vr::IVRDriverContext {
public:
    FakeHost host;
    FakeLog log;
    FakeSettings settings;
    FakeProperties props;
    int dummy = 0; // stands in for interfaces the driver never calls (manager, resources)

    void* GetGenericInterface(const char* name, vr::EVRInitError* err) override {
        if (err) *err = vr::VRInitError_None;
        if (!std::strcmp(name, vr::IVRServerDriverHost_Version)) return static_cast<vr::IVRServerDriverHost*>(&host);
        if (!std::strcmp(name, vr::IVRDriverLog_Version)) return static_cast<vr::IVRDriverLog*>(&log);
        if (!std::strcmp(name, vr::IVRSettings_Version)) return static_cast<vr::IVRSettings*>(&settings);
        if (!std::strcmp(name, vr::IVRProperties_Version)) return static_cast<vr::IVRProperties*>(&props);
        if (!std::strcmp(name, vr::IVRDriverManager_Version) || !std::strcmp(name, vr::IVRResources_Version)) return &dummy;
        if (err) *err = vr::VRInitError_Init_InterfaceNotFound;
        return nullptr;
    }
    vr::DriverHandle_t GetDriverHandle() override { return 1; }
};

vr::DriverPose_t MakePose(double x, double vx, vr::ETrackingResult result, bool valid) {
    vr::DriverPose_t p{};
    p.qWorldFromDriverRotation = {1, 0, 0, 0};
    p.qDriverFromHeadRotation = {1, 0, 0, 0};
    p.qRotation = {1, 0, 0, 0};
    p.vecPosition[0] = x;
    p.vecPosition[1] = 1.0;
    p.vecVelocity[0] = vx;
    p.result = result;
    p.poseIsValid = valid;
    p.deviceIsConnected = true;
    return p;
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

std::string PipeRequest(const std::string& line) {
    HANDLE p = CreateFileW(L"\\\\.\\pipe\\OpenVR-TrackingStabilizer", GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (p == INVALID_HANDLE_VALUE) return {};
    const std::string req = line + "\n";
    DWORD n = 0;
    WriteFile(p, req.data(), DWORD(req.size()), &n, nullptr);
    std::string resp;
    char buf[4096];
    while (resp.find('\n') == std::string::npos && ReadFile(p, buf, sizeof(buf), &n, nullptr) && n) resp.append(buf, n);
    CloseHandle(p);
    return resp;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: driver_harness <driver_trackingstabilizer.dll>\n");
        return 2;
    }
    timeBeginPeriod(1);

    HMODULE dll = LoadLibraryA(argv[1]);
    if (!dll) { std::printf("cannot load %s (%lu)\n", argv[1], GetLastError()); return 1; }
    using Factory = void* (*)(const char*, int*);
    auto factory = reinterpret_cast<Factory>(GetProcAddress(dll, "HmdDriverFactory"));
    int rc = 0;
    auto* provider = static_cast<vr::IServerTrackedDeviceProvider*>(factory(vr::IServerTrackedDeviceProvider_Version, &rc));
    CHECK(provider != nullptr, "no provider");

    auto* ctx = new FakeContext; // leaked on purpose: the driver may log during unload
    std::printf("Init\n");
    CHECK(provider->Init(ctx) == vr::VRInitError_None, "Init failed");

    // ---- feed poses like lighthouse: 4 ms IMU cadence, + optical update 0.5 ms later ----
    vr::IVRServerDriverHost* host = &ctx->host; // virtual call -> patched function -> our detour
    struct Sent { double t; uint32_t device; vr::DriverPose_t pose; };
    std::vector<Sent> sent;
    const auto t0 = ctx->host.start;
    auto now = [&] { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    auto send = [&](uint32_t d, const vr::DriverPose_t& p) {
        sent.push_back({now(), d, p});
        host->TrackedDevicePoseUpdated(d, p, sizeof(p));
    };

    // Tracker (3): healthy 1 s -> loss (OK 0.25 s, then out of range, drifting 0.2 m/s) -> reset at
    // 2.0 s after loss (one invalid pose, then nothing for 1 s) -> resumes 5 cm away.
    const double lossAt = 1.0, resetAt = lossAt + 2.0, resumeAt = resetAt + 1.0, endAt = resumeAt + 1.5;
    const bool pipeTest = !SteamVrRunning(); // our pipe name would collide with the real driver's
    if (pipeTest) PipeRequest("trace 0");   // the driver only samples the trace while someone asks for it
    std::printf("feeding poses for %.1f s\n", endAt);
    bool sentInvalid = false;
    for (double next = 0; next < endAt; next += 0.004) {
        std::this_thread::sleep_until(t0 + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(next)));
        const double t = now();
        send(1, MakePose(0.0, 0, vr::TrackingResult_Running_OK, true));
        send(4, MakePose(-0.5, 0, vr::TrackingResult_Running_OK, true));
        if (t < lossAt) {
            send(3, MakePose(0.5, 0, vr::TrackingResult_Running_OK, true));
        } else if (t < resetAt) {
            const double l = t - lossAt;
            send(3, MakePose(0.5 + 0.2 * l, 0.2, l < 0.25 ? vr::TrackingResult_Running_OK : vr::TrackingResult_Calibrating_OutOfRange, true));
        } else if (t < resumeAt) {
            if (!sentInvalid) send(3, MakePose(0.0, 0, vr::TrackingResult_Calibrating_OutOfRange, false));
            sentInvalid = true;
        } else {
            send(3, MakePose(0.55, 0, vr::TrackingResult_Running_OK, true));
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        if (t < lossAt || t >= resumeAt) send(3, MakePose(t < lossAt ? 0.5 : 0.55, 0, vr::TrackingResult_Running_OK, true));
        send(1, MakePose(0.0, 0, vr::TrackingResult_Running_OK, true));
        send(4, MakePose(-0.5, 0, vr::TrackingResult_Running_OK, true));
    }

    // ---- pipe ----
    if (!pipeTest) {
        std::printf("pipe test skipped (SteamVR is running)\n");
    } else {
        // Trace: the tracker's hold shows up as H samples, the reset as samples without raw input.
        const std::string trace = PipeRequest("trace 0");
        CHECK(trace.find("\"serial\":\"TEST-3\"") != std::string::npos && trace.find(",\"H\"]") != std::string::npos, "trace lacks the hold");
        CHECK(trace.find(",0,\"H\"]") != std::string::npos, "trace lacks samples without raw input");
        CHECK(trace.find("\"serial\":\"TEST-1\"") != std::string::npos, "trace lacks the bypassed HMD");
        const std::string devices = PipeRequest("devices");
        std::printf("devices: %s", devices.c_str());
        CHECK(devices.find("\"TEST-3\"") != std::string::npos && devices.find("\"filtered\"") != std::string::npos, "devices reply");
        CHECK(devices.find("\"TEST-1\",\"class\":1,\"mode\":\"bypass\"") != std::string::npos, "HMD not bypassed");
        const std::string set = PipeRequest("set probationMs 500");
        CHECK(set.find("\"probationMs\":500") != std::string::npos, "set reply: %s", set.c_str());
        const std::string log = PipeRequest("log 0");
        CHECK(log.find("\"kind\":\"filter\"") != std::string::npos && log.find("TEST-3 TRACKING -> HOLD") != std::string::npos,
              "log reply lacks the filter transition");
        CHECK(log.find("\"kind\":\"ots\"") != std::string::npos, "log reply lacks driver lines");
        CHECK(PipeRequest("filter off").find("\"enabled\":false") != std::string::npos, "filter off");
        CHECK(PipeRequest("filter on").find("\"enabled\":true") != std::string::npos, "filter on");

        // Per-device switches: exclude a tracker, opt the HMD in (needs filterHmd as well).
        PipeRequest("set filterExcludeSerials test-3");
        PipeRequest("set filterHmd true");
        PipeRequest("set filterIncludeSerials TEST-1");
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // the emitter thread reclassifies
        const std::string after = PipeRequest("devices");
        CHECK(after.find("\"TEST-3\",\"class\":3,\"mode\":\"bypass\"") != std::string::npos, "excluded tracker still filtered: %s", after.c_str());
        CHECK(after.find("\"TEST-1\",\"class\":1,\"mode\":\"filtered\"") != std::string::npos, "included HMD not filtered: %s", after.c_str());
        CHECK(after.find("\"TEST-4\",\"class\":2,\"mode\":\"filtered\"") != std::string::npos, "other device changed: %s", after.c_str());
        const std::string cfg = PipeRequest("set smoothing 40");
        CHECK(cfg.find("\"smoothing\":40") != std::string::npos && cfg.find("\"filterExcludeSerials\":\"TEST-3\"") != std::string::npos,
              "config reply: %s", cfg.c_str());
    }

    // ---- checks ----
    std::vector<Received> got;
    {
        std::lock_guard lock(ctx->host.m);
        got = ctx->host.received;
    }
    auto ofDevice = [&](uint32_t d) {
        std::vector<Received> v;
        for (auto& r : got) if (r.device == d) v.push_back(r);
        return v;
    };
    size_t sentCount[8] = {};
    for (auto& s : sent) ++sentCount[s.device];

    // HMD and controller: healthy all along -> forwarded unchanged.
    for (uint32_t d : {1u, 4u}) {
        auto r = ofDevice(d);
        int changed = 0;
        for (auto& x : r) changed += x.pose.vecPosition[0] != (d == 1 ? 0.0 : -0.5) || !x.pose.poseIsValid;
        CHECK(r.size() == sentCount[d], "device %u: sent %zu received %zu", d, sentCount[d], r.size());
        CHECK(changed == 0, "device %u: %d poses changed", d, changed);
    }

    auto tr = ofDevice(3);
    double held = -1, worst = 0, maxVel = 0;
    size_t duringStop = 0, invalid = 0;
    for (auto& x : tr) {
        if (x.t < lossAt + 0.3 || x.t > resumeAt) continue;
        if (held < 0) held = x.pose.vecPosition[0];
        worst = std::max(worst, std::fabs(x.pose.vecPosition[0] - held));
        maxVel = std::max(maxVel, std::fabs(x.pose.vecVelocity[0]));
        invalid += !x.pose.poseIsValid;
        if (x.t > resetAt + 0.05 && x.t < resumeAt - 0.01) ++duringStop;
    }
    std::printf("tracker: held x=%.4f (lost at 0.5000), worst move while held %.6f, emitted during stop %zu\n", held, worst, duringStop);
    CHECK(held > 0.5 - 1e-9 && held < 0.5 + 0.2 * 0.3, "held at %.4f", held);
    CHECK(worst < 1e-9, "held position moved by %.6f", worst);
    CHECK(maxVel == 0, "velocity while held %.3f", maxVel);
    CHECK(invalid == 0, "%zu invalid poses forwarded while held", invalid);
    CHECK(duringStop > 0.95 * 250 * 0.8, "only %zu emitted poses during a ~0.94 s stop", duringStop);
    CHECK(!tr.empty() && std::fabs(tr.back().pose.vecPosition[0] - 0.55) < 1e-9, "did not recover to the live position");

    double maxStep = 0;
    for (size_t i = 1; i < tr.size(); ++i)
        if (tr[i].t > resumeAt) maxStep = std::max(maxStep, std::fabs(tr[i].pose.vecPosition[0] - tr[i - 1].pose.vecPosition[0]));
    CHECK(maxStep < 0.01, "recovery step %.4f m", maxStep);

    if (pipeTest) {
        std::printf("body model: explicit mapping, raw bypassed HMD, toggle, stale waist rotation\n");
        PipeRequest("filter off");
        PipeRequest("set smoothing 0");
        PipeRequest("set filterHmd false");
        PipeRequest("set filterIncludeSerials -");
        PipeRequest("set filterExcludeSerials -");
        CHECK(PipeRequest("set bodyWaistSerial test-3").find("\"bodyWaistSerial\":\"TEST-3\"") != std::string::npos,"waist mapping");
        PipeRequest("set bodyHmdSerial TEST-1");
        CHECK(PipeRequest("set bodyEnabled true").find("\"bodyEnabled\":true") != std::string::npos,"body toggle on");
        CHECK(PipeRequest("set bodyNeckDownCm NaN").find("\"ok\":false") != std::string::npos,"NaN accepted");
        CHECK(PipeRequest("set bodyNeckDownCm 12junk").find("\"ok\":false") != std::string::npos,"junk accepted");
        PipeRequest("filter on");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto headPose = MakePose(1,0,vr::TrackingResult_Running_OK,true); headPose.vecPosition[1] = 1.62; headPose.vecPosition[2] = -.08;
        auto waistPose = MakePose(1,0,vr::TrackingResult_Running_OK,true);
        for (int i = 0; i < 210; ++i) {
            send(1,headPose); send(3,waistPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        CHECK(PipeRequest("devices").find("\"bodyReady\":true") != std::string::npos,"model did not calibrate");
        waistPose.result = vr::TrackingResult_Calibrating_OutOfRange;
        for (int i = 0; i < 75; ++i) {
            headPose.vecPosition[0] = 1+i*.001;
            send(1,headPose); send(3,waistPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        auto lastTracker = [&] {
            std::lock_guard lock(ctx->host.m);
            for (auto it = ctx->host.received.rbegin(); it != ctx->host.received.rend(); ++it)
                if (it->device == 3) return it->pose;
            return vr::DriverPose_t{};
        };
        CHECK(lastTracker().vecPosition[0] > 1.06,"body did not follow bypassed HMD");
        CHECK(PipeRequest("devices").find("\"bodyActive\":true") != std::string::npos,"model not active");
        for (int i = 0; i < 60; ++i) {
            headPose.vecPosition[0] += .001; send(1,headPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        const double frozen = lastTracker().vecPosition[0];
        for (int i = 0; i < 30; ++i) {
            headPose.vecPosition[0] += .001; send(1,headPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        CHECK(std::abs(lastTracker().vecPosition[0]-frozen) < 1e-9,"stale waist rotation still followed HMD");
        CHECK(PipeRequest("set bodyEnabled false").find("\"bodyEnabled\":false") != std::string::npos,"body toggle off");
        for (int i = 0; i < 30; ++i) {
            headPose.vecPosition[0] += .001; send(1,headPose); send(3,waistPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        CHECK(std::abs(lastTracker().vecPosition[0]-frozen) < 1e-9,"toggle off changed the held position");

        std::printf("hand model: controller roles, 3-point input, bypassed opposite hand, lost HMD, toggle\n");
        PipeRequest("filter off");
        CHECK(PipeRequest("set handEnabled true").find("\"handEnabled\":true") != std::string::npos,"hand toggle on");
        CHECK(PipeRequest("set handArmReachCm 0").find("\"ok\":false") != std::string::npos,"invalid arm reach accepted");
        PipeRequest("set handFollowDeadzoneCm 0"); // the shoulder follow below moves the body 7.5 cm
        PipeRequest("filter on");
        auto right = MakePose(1.35,0,vr::TrackingResult_Running_OK,true); right.vecPosition[1] = 1.1; right.vecPosition[2] = -.3;
        auto left = right; left.vecPosition[0] = .65;
        headPose.vecPosition[0] = 1;
        for (int i = 0; i < 100; ++i) {
            send(1,headPose); send(4,right); send(5,left); send(6,right);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        const auto handDevices = PipeRequest("devices");
        CHECK(handDevices.find("\"handSide\":1") != std::string::npos && handDevices.find("\"handSide\":-1") != std::string::npos,"left/right roles missing");
        CHECK(handDevices.find("\"handReady\":true") != std::string::npos,"hands not ready");
        // The other hand's raw pose must remain available when it bypasses correction.
        PipeRequest("set filterExcludeSerials TEST-5");
        right.result = vr::TrackingResult_Calibrating_OutOfRange;
        for (int i = 0; i < 75; ++i) {
            headPose.vecPosition[0] = 1+i*.001; left.vecPosition[0] = .65+i*.001;
            send(1,headPose); send(5,left); send(4,right);
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        auto lastRight = [&] {
            std::lock_guard lock(ctx->host.m);
            for (auto it = ctx->host.received.rbegin(); it != ctx->host.received.rend(); ++it)
                if (it->device == 4) return it->pose;
            return vr::DriverPose_t{};
        };
        CHECK(lastRight().vecPosition[0] > 1.41,"occluded hand did not follow shoulder");
        CHECK(PipeRequest("devices").find("\"handActive\":true") != std::string::npos,"no active hand model");
        CHECK(PipeRequest("devices").find("\"TEST-6\",\"class\":2,\"mode\":\"bypass\",\"handSide\":0,\"lighthouse\":false") != std::string::npos,
              "a controller of another tracking system was corrected");
        {
            const std::string model = PipeRequest("model");
            CHECK(model.find("\"kind\":\"hand\",\"estimating\":true") != std::string::npos && model.find("\"elbow\":") != std::string::npos,
                  "model reply lacks the estimated arm: %s", model.c_str());
        }
        PipeRequest("set bodyWaistSerial -");
        CHECK(PipeRequest("devices").find("\"handReady\":true") != std::string::npos,"waist assignment reset hand compensation");
        const double heldRight = lastRight().vecPosition[0];
        headPose.poseIsValid = false;
        for (int i = 0; i < 30; ++i) {
            send(1,headPose); send(4,right); std::this_thread::sleep_for(std::chrono::milliseconds(4));
        }
        CHECK(std::abs(lastRight().vecPosition[0]-heldRight) < 1e-9,"hand followed a lost HMD");
        CHECK(PipeRequest("set handEnabled false").find("\"handEnabled\":false") != std::string::npos,"hand toggle off");
        headPose.poseIsValid = true; headPose.vecPosition[0] += .1;
        send(1,headPose); send(4,right);
        CHECK(std::abs(lastRight().vecPosition[0]-heldRight) < 1e-9,"hand toggle off changed held position");
    }
    std::printf("Cleanup\n"); provider->Cleanup(); FreeLibrary(dll); std::printf("unloaded\n");
    timeEndPeriod(1);
    std::printf(g_failures ? "%d FAILURE(S)\n" : "driver harness passed\n", g_failures);
    return g_failures ? 1 : 0;
}
