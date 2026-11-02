#pragma once
// Everything the UI shows, filled by VrSource (OpenVR) and DriverClient (pipe).
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace ots {

struct Vec3 {
    float x = 0, y = 0, z = 0;
};

struct DeviceInfo {
    uint32_t index = 0;
    int deviceClass = 0;          // vr::ETrackedDeviceClass
    std::string serial, model;
    int role = 0;                 // vr::ETrackedControllerRole
    bool connected = false;
    bool poseValid = false;
    int trackingResult = 0;       // vr::ETrackingResult
    Vec3 position;                // standing universe, meters
    Vec3 forward;                 // -Z axis of the device (unit)
    Vec3 right, up;               // +X / +Y axes of the device (unit)
    bool hasBattery = false;
    float battery = 0;            // 0..1
    bool charging = false;
    std::string modeLabel;        // base stations
    float fovH = 150, fovV = 110; // base stations, degrees
};

// From the driver's `devices` reply.
struct FilterDeviceInfo {
    bool handReady = false, handActive = false;
    int handSide = 0;
    bool lighthouse = true;
    bool bodyReady = false, bodyActive = false;
    float torsoLengthCm = 0;
    bool imuOpen = false, imuAligned = false, imuActive = false;
    int imuError = -1; // IOBuffer Open() result, -1 = not tried
    float imuRate = 0; // samples per second
    std::string serial;
    std::string mode;  // filtered / bypass / pending
    std::string state; // TRACKING / HOLD / ...
    double stateSeconds = 0;
    int holds = 0;
    int64_t emitted = 0;
    int64_t stepsLimited = 0;
};

// From the driver's `trace` reply: 30 Hz samples of the raw and corrected position (driver world space).
struct TraceSample {
    double t = 0;          // seconds on the driver's clock
    Vec3 raw, out;         // meters
    bool rawValid = true;  // false while the tracking driver sent nothing
    char state = 'T';      // T tracking, H hold, P probation, B blend, L lost and not corrected
};

// From the driver's `model` reply: the joints a hand / waist model is working with (driver world space).
struct ModelPose {
    std::string serial;
    bool hand = true;        // hand model (else waist)
    bool estimating = false; // estimating a hidden device (else following a visible one)
    bool haveShoulder = false, haveElbow = false;
    Vec3 neck, shoulder, elbow, end; // metres; end = the hand or the waist
};

struct LogEntry {
    uint64_t seq = 0;
    int64_t unixMs = 0;
    std::string kind; // driver / ots / filter / ui
    std::string text;
};

struct FilterConfig {
    bool imuEnabled = false;
    bool bodyEnabled = false;
    bool handEnabled = false;
    float handShoulderWidthCm = 36, handArmReachCm = 70, handMaxPredictionMs = 3000;
    std::string bodyWaistSerial, bodyHmdSerial;
    std::string bodyHmdAuto; // the HMD used when none is chosen (the only one connected), else empty
    bool valid = false;
    bool enabled = false, controllers = true, trackers = true, hmd = false; // hmd: "also correct the HMD" allowed
    float smoothing = 0;                                                     // jitter smoothing, 0..100 %
    float probationMs = 400, shortProbationMs = 100, blendMs = 200, stepLimitCm = 4.5f, stepBlendMs = 150;
    std::vector<std::string> includeSerials, excludeSerials;                // per-device overrides (upper case)
};

struct StateData {
    // OpenVR
    bool vrConnected = false;
    std::vector<DeviceInfo> devices;
    std::vector<Vec3> playArea;      // 4 corners, standing universe
    float rawToStanding[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}}; // driver world space -> standing

    // Driver (pipe)
    bool driverConnected = false;
    std::string driverStatusRaw;
    bool recording = false;
    std::string recordingPath;
    std::map<uint32_t, FilterDeviceInfo> filterDevices;
    FilterConfig filter;
    bool traceSupported = false;
    double traceNow = 0;                                    // driver clock at the last trace reply
    int64_t traceNowLocalMs = 0;                            // steady clock (ms) when it arrived
    std::map<std::string, std::vector<TraceSample>> trace;  // by serial, oldest first, ~25 s
    std::vector<ModelPose> models;                          // body models at work now (empty: none)
    std::vector<LogEntry> log;       // newest last, capped
    uint64_t logNext = 0;
};

// Shared between the render thread and the worker threads; the UI draws from a copy taken under `m`.
struct AppState : StateData {
    std::mutex m;

    StateData Snapshot() {
        std::lock_guard lock(m);
        return static_cast<const StateData&>(*this);
    }
    void AddUiLog(const std::string& text);
};

} // namespace ots
