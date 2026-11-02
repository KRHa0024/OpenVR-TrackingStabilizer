#pragma once
// Owns one ots::filter::TrackingFilter per tracked device and the emitter thread (docs/phase1.md).
//
// Process() runs on the thread that called TrackedDevicePoseUpdated (lighthouse's). New devices
// are passed through until the emitter thread has looked up their class and serial: property
// queries are never made from inside the hook.
#include "ImuOrientation.h"
#include "TrackingFilter.h"

#include <openvr_driver.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace ots {

class Recorder;

class FilterManager {
public:
    // Which devices are filtered: Lighthouse controllers / trackers by class unless their serial is excluded,
    // others when their serial is included. The HMD only when `hmd` allows it AND its serial is included.
    struct Settings {
        bool enabled = true;
        bool controllers = true;
        bool trackers = true;
        bool hmd = false;
        std::vector<std::string> includeSerials; // upper-case
        std::vector<std::string> excludeSerials; // upper-case
        double emitHz = 250;
        filter::Config filter;
        std::string bodyWaistSerial, bodyHmdSerial; // explicit role mapping, upper-case
        bool imuEnabled = false; // rotation from the raw IMU while lighthouse has none (experimental)
    };

    // Forwards a pose to vrserver's original TrackedDevicePoseUpdated (bypassing our hook).
    using ForwardFn = void (*)(void* host, uint32_t device, const vr::DriverPose_t* pose);

    FilterManager(Recorder* recorder, ForwardFn forward, const Settings& settings);
    ~FilterManager();

    void Start();
    void Stop();

    // Hot path. Returns true and fills `out` when the device is filtered; false = forward `in` as is.
    bool Process(void* host, uint32_t device, const vr::DriverPose_t& in, vr::DriverPose_t* out);

    void SetEnabled(bool enabled);
    bool Enabled() const { return enabled_.load(std::memory_order_acquire); }

    // Runtime change of one setting (same key names as steamvr.vrsettings). Not persisted.
    bool Set(const std::string& key, const std::string& value, std::string* error);

    std::string DevicesJson();
    std::string SettingsJson();
    // Raw and filtered positions (30 Hz, last 25 s) newer than `since` (seconds, driver clock), for the
    // overlay's graph. Sampling only runs while someone asked for a trace in the last few seconds.
    std::string TraceJson(double since);
    // The joints each hand / waist model is working with now (driver world space, mm), for visualisation.
    std::string ModelJson();

private:
    enum Mode : int { kUnseen = 0, kPending = 1, kFiltered = 2, kBypass = 3 };

    struct TraceSample {
        double t;
        float raw[3], out[3]; // world space, meters
        bool rawValid;        // false while the input stream has stopped (lighthouse reset tracking)
        char state;           // T tracking, H hold, P probation, B blend, L lost (not filtered)
    };

    struct Slot {
        std::mutex m; // guards everything below except `mode`, `recheck` and the trace
        std::atomic<int> mode{kUnseen};
        std::atomic<bool> recheck{false}; // settings changed: classify again without passing through as pending
        std::atomic<bool> bodyReference{false};
        filter::TrackingFilter f;
        void* host = nullptr;
        std::string serial;
        int deviceClass = -1;
        bool lighthouse = false;
        int handSide = 0;
        std::atomic<int> referenceHandSide{0};
        uint64_t emitted = 0;
        uint64_t holds = 0;
        uint64_t inputs = 0;          // poses processed from the hook (the emitter checks it changed)
        vr::DriverPose_t lastOut{};   // the latest of them, as forwarded
        int bodyStatus = -1;
        int handStatus = -1;
        filter::ImuOrientation imu; // guarded by m

        // Raw IMU stream, emitter thread only.
        vr::IOBufferHandle_t imuBuffer = vr::k_ulInvalidIOBufferHandle;
        double imuRetryT = 0;
        int imuError = -1;        // last Open() result (-1 = not tried), shown in `devices`
        std::string imuPath;
        bool imuAnnounced = false; // "aligned" logged for this stream

        std::mutex traceM;
        std::atomic<double> lastTraceT{-1};
        std::deque<TraceSample> trace;
    };

    void EmitterLoop();
    void Classify(uint32_t device);
    void FlushTransitions(uint32_t device, Slot& s); // caller holds s.m
    void ApplyFilterConfig();
    void Reclassify();
    void RecordTrace(Slot& s, double t, const vr::DriverPose_t* raw, const vr::DriverPose_t& out, char state);
    void BodyReference(filter::TrackingFilter& f); // caller holds slot mutex
    void PollImu(uint32_t device, Slot& s, double t);  // emitter thread
    void CloseImu(Slot& s);                            // emitter thread (or after it stopped)
    static double Now();

    Recorder* recorder_;
    ForwardFn forward_;
    std::atomic<bool> enabled_{true};
    std::atomic<bool> imuEnabled_{false};

    std::mutex settingsMutex_;
    Settings settings_;
    std::set<std::string> hmdSerials_; // HMDs seen (upper-case), for the automatic reference; settingsMutex_

    std::array<Slot, vr::k_unMaxTrackedDeviceCount> slots_;
    std::mutex bodyMutex_; // raw reference only; never acquire a slot mutex while holding this
    vr::DriverPose_t bodyHead_{};
    double bodyHeadT_ = -1;
    vr::DriverPose_t rawHands_[2]{};
    double rawHandTimes_[2] = {-1,-1};
    std::thread emitter_;
    std::atomic<bool> stop_{false};
    std::atomic<double> traceWantedUntil_{0};
};

} // namespace ots
