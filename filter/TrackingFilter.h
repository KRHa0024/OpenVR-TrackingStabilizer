#pragma once
// Per-device tracking-loss filter (docs/phase1.md).
//
// Sits between the lighthouse driver and vrserver. While tracking is healthy the input pose is
// returned unchanged (bit for bit) unless jitter smoothing is on. When optical tracking is lost it holds the last trustworthy
// position with zero velocity, keeps holding through a probation period after tracking resumes
// (bad single-base bootstraps fall over within ~0.3 s), then blends back to the live position.
//
// Not thread-safe: the owner serializes Process() and Emit() for one device.
// Uses only the vr::DriverPose_t type from openvr_driver.h, no OpenVR runtime calls, so the same
// code runs inside vrserver and in tools/ots-replay.
#include <openvr_driver.h>
#include "WaistModel.h"
#include "HandModel.h"

#include <cstdint>
#include <deque>
#include <vector>

namespace ots::filter {

struct Config {
    // Early detection from the pose update cadence: with optical data ~45% of update intervals
    // are < 1 ms (optical updates interleave with IMU updates); IMU-only output is a steady ~4 ms.
    // Off by default: on the Phase 0 replays it changed 0.2-2% of healthy poses (false alarms
    // freezing moving limbs) and did not improve the loss cases (docs/phase1.md, replay results).
    bool suspectEnabled = false;
    double suspectWindow = 0.050;     // no short interval for this long -> SUSPECT
    double shortInterval = 0.001;     // what counts as a "short" update interval
    int suspectArmCount = 50;         // short intervals needed in the last second to trust the cadence
    double suspectMax = 0.300;        // result still OK after this long -> it was a false alarm

    double streamTimeout = 0.020;     // no input for this long while holding -> Emit() takes over
    double resetGap = 0.100;          // a pause at least this long means lighthouse reset tracking (it
                                      // stops sending ~2 s into an optical loss; those pauses last
                                      // 0.12 s or more). Shorter pauses are USB / radio stalls.
    double probation = 0.400;         // Running_OK must last this long before trusting it again ...
    double shortProbation = 0.100;    // ... unless lighthouse never reset tracking during the loss (no
                                      // invalid pose, no stream stop): bad bootstraps only follow resets.
                                      // 50 ms let a 10.8 cm correction through in the E4 replay; 100 ms did not.
    double blend = 0.200;             // hold -> live interpolation time

    double stepLimit = 0.045;         // unexplained step (m) while tracking that gets smoothed
    double stepBlend = 0.150;         // time to absorb such a step
    double stepMaxDt = 0.050;         // only compare consecutive poses closer than this

    bool holdReportsValid = true;     // report held poses as valid / Running_OK
    double maxHold = 0.0;             // give up holding after this long (0 = never)

    // Jitter smoothing, 0 (off: output untouched) .. 1 (strongest). A One Euro filter on position,
    // rotation and the reported velocities: slow movement is smoothed hard, fast movement follows with
    // little lag.
    double smoothing = 0.0;
    BodyConfig body;
    HandConfig hand;
};

enum class State : uint8_t { Tracking, Suspect, Hold, Probation, Blend };
const char* ToString(State s);

struct Transition {
    double t;
    State from;
    State to;
    const char* reason; // static string
};

class TrackingFilter {
public:
    explicit TrackingFilter(const Config& config = {}) : cfg_(Sanitized(config)) {
        body_.Configure(config.body);
        hand_.Configure(config.hand,config.body);
    }

    // Times and distances clamped to values the state machine can work with (settings files and
    // the pipe accept anything; a zero step limit, for example, would turn every pose into a step).
    static Config Sanitized(Config c);

    void SetConfig(const Config& config) {
        const bool wasModelEnabled = ModelEnabled();
        cfg_ = Sanitized(config);
        body_.Configure(config.body);
        hand_.Configure(config.hand,config.body);
        if (!wasModelEnabled && ModelEnabled() && state_ != State::Tracking) {
            holdRotation_ = blendRotation_ = lastRotation_;
            rotationT_ = lastInputT_;
        }
        if (!config.body.enabled) bodyActive_ = false;
        if (!config.hand.enabled) handActive_ = false;
    }
    const Config& GetConfig() const { return cfg_; }

    // `t` is the arrival time of the pose in seconds (any monotonic origin).
    vr::DriverPose_t Process(const vr::DriverPose_t& in, double t);

    // While holding and the input has stopped (lighthouse sends nothing after it resets tracking),
    // fills `out` with the held pose and returns true. Call periodically.
    bool Emit(double t, vr::DriverPose_t* out);
    // Stops correcting (the filter is turned off for this device, or a hold timed out). If the
    // output was a held pose, fills `out` with a pose that tells SteamVR the device is not tracked
    // and returns true: otherwise, while lighthouse is silent, the last held "valid" pose would stay.
    bool Release(double t, const char* reason, vr::DriverPose_t* out);
    void SetBodyReference(const vr::DriverPose_t& pose, double t) { body_.Reference(pose, t); }
    bool BodyReady() const { return body_.Ready(); }
    bool BodyActive() const { return bodyActive_; }
    double TorsoLength() const { return body_.TorsoLength(); }
    void SetHandReference(const vr::DriverPose_t& head, double headTime,
                          const vr::DriverPose_t& other, double otherTime) { hand_.Reference(head,headTime,other,otherTime); }
    bool HandReady() const { return hand_.Ready(); }
    bool HandActive() const { return handActive_; }
    // The joints the hand / waist model last worked with, for visualisation.
    const ModelDebug& HandDebug() const { return hand_.Debug(); }
    const ModelDebug& BodyDebug() const { return body_.Debug(); }
    // The device's rotation from its own IMU (ImuOrientation), at time `t`. Used while lighthouse
    // reports no rotation (it stops ~2 s into an optical loss); call before Process / Emit.
    void SetImuRotation(const math::Quat& world, double t) { imuRotation_ = world; imuT_ = t; }
    bool ImuActive() const { return imuActive_; } // the last output's rotation came from the IMU

    State GetState() const { return state_; }
    double StateSince() const { return stateSince_; }
    uint64_t StepsLimited() const { return stepsLimited_; }

    // Transitions since the last call, oldest first.
    std::vector<Transition> TakeTransitions();

private:
    struct Vec3 {
        double x = 0, y = 0, z = 0;
    };
    static Vec3 ToWorld(const vr::DriverPose_t& p);
    static void SetFromWorld(vr::DriverPose_t& p, const Vec3& world);

    void SetState(State s, double t, const char* reason);
    void Reset(double t, const char* reason);
    bool ShortIntervalsSince(double since, int atLeast) const;
    bool Armed(double t) const;
    void EnterHold(double t, const char* reason, double lostSince = -1);
    vr::DriverPose_t HoldPose(const vr::DriverPose_t& in, double t);
    void AcceptRotation(const vr::DriverPose_t& in, double t);
    bool ImuFresh(double t) const { return imuT_ >= 0 && t - imuT_ <= .05 && imuT_ - t <= .05; }
    void UseImuRotation(double t); // model path: take the IMU's rotation while lighthouse has none
    void UpdateBodyHold(double t);
    void UpdateHandHold(double t);
    bool ModelEnabled() const { return cfg_.body.enabled || cfg_.hand.enabled; }
    // The output rotation is the filter's own (held, then blended back) rather than lighthouse's
    // pass-through: with a model, or once the IMU took over in this loss episode.
    bool ManagedRotation() const { return ModelEnabled() || imuEpisode_; }
    vr::DriverPose_t BlendPose(const vr::DriverPose_t& in, double t);
    vr::DriverPose_t TrackingPose(const vr::DriverPose_t& in, double t);
    Vec3 StepOffset(double t);
    void Smooth(vr::DriverPose_t& out, double t);

    Config cfg_;
    State state_ = State::Tracking;
    double stateSince_ = 0;
    std::vector<Transition> transitions_;

    bool haveInput_ = false;
    double lastInputT_ = 0;
    vr::DriverPose_t prevIn_{};
    bool haveValid_ = false;
    vr::DriverPose_t lastValid_{};

    std::deque<double> shortTimes_; // arrival times of short intervals, last second
    bool trusted_ = false;          // has produced a good TRACKING output since the last reset

    // All positions below are world space (see ToWorld).
    Vec3 hold_;
    double holdStart_ = 0;
    bool resetSeen_ = false;        // lighthouse reset tracking during the current loss
    double probationStart_ = 0;
    Vec3 blendFrom_;
    double blendStart_ = 0;
    Vec3 lastOut_;
    WaistModel body_;
    HandModel hand_;
    bool handActive_ = false;
    bool bodyActive_ = false;
    bool bodyAnchored_ = false, bodyRejected_ = false;
    math::Vec bodyBias_{}; // small world-space calibration residual at loss onset
    math::Quat holdRotation_{1,0,0,0}, lastRotation_{1,0,0,0}, blendRotation_{1,0,0,0};
    double rotationT_ = -1, bodyUpdateT_ = -1;
    double lighthouseRotationT_ = -1; // last rotation taken from lighthouse (rotationT_: from any source)
    math::Quat imuRotation_{1,0,0,0};
    double imuT_ = -1;
    bool imuActive_ = false;
    bool imuEpisode_ = false; // the IMU's rotation was used during the current loss episode

    bool offsetActive_ = false;
    Vec3 offsetStart_;
    double offsetT0_ = 0;
    uint64_t stepsLimited_ = 0;

    // Jitter smoothing state: pose in world space, velocities as reported (driver space).
    bool smoothInit_ = false;
    double smoothT_ = 0;
    Vec3 smoothPos_;
    double smoothRot_[4] = {1, 0, 0, 0}; // w, x, y, z
    double smoothVel_[3] = {}, smoothAngVel_[3] = {};
};

} // namespace ots::filter
