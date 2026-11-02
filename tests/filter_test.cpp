// Unit tests for ots::filter::TrackingFilter on synthetic lighthouse-like pose streams.
#include "TrackingFilter.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

using ots::filter::Config;
using ots::filter::State;
using ots::filter::TrackingFilter;

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

struct Phase {
    double duration;
    bool optical;                        // interleave optical updates (short intervals)
    int result;                          // vr::ETrackingResult
    bool valid = true;
    bool stream = true;                  // false = lighthouse sends nothing
    std::function<double(double)> x = [](double) { return 0.0; }; // reported world x over local time
    double vx = 0;                       // reported world x velocity
    bool rotatedFrame = false;           // driver space rotated 90 deg about +y and shifted (re-bootstrap from another base)
};

struct Sample {
    double t;
    vr::DriverPose_t in;
    vr::DriverPose_t out;
    State state;
};

// World position of the device is (x, 1, 0). In the rotated frame, world = R * p + t with
// R = 90 deg about +y, i.e. R(a, b, c) = (c, b, -a), and t = (2, 0, -1).
vr::DriverPose_t MakePose(double x, double vx, int result, bool valid, bool rotatedFrame = false) {
    vr::DriverPose_t p{};
    p.qWorldFromDriverRotation = {1, 0, 0, 0};
    p.qDriverFromHeadRotation = {1, 0, 0, 0};
    p.qRotation = {1, 0, 0, 0};
    p.vecPosition[0] = x;
    p.vecPosition[1] = 1.0;
    p.vecVelocity[0] = vx;
    if (rotatedFrame) {
        const double s = std::sqrt(0.5);
        p.qWorldFromDriverRotation = {s, 0, s, 0};
        p.vecWorldFromDriverTranslation[0] = 2.0;
        p.vecWorldFromDriverTranslation[2] = -1.0;
        // p = R^-1 (w - t), R^-1(a, b, c) = (-c, b, a)
        const double w[3] = {x - 2.0, 1.0, 0.0 + 1.0};
        p.vecPosition[0] = -w[2];
        p.vecPosition[1] = w[1];
        p.vecPosition[2] = w[0];
        p.vecVelocity[0] = 0;
        p.vecVelocity[2] = vx;
    }
    p.result = vr::ETrackingResult(result);
    p.poseIsValid = valid;
    p.deviceIsConnected = true;
    return p;
}

double WorldX(const vr::DriverPose_t& p) {
    if (p.qWorldFromDriverRotation.w == 1.0) return p.vecPosition[0] + p.vecWorldFromDriverTranslation[0];
    return p.vecPosition[2] + p.vecWorldFromDriverTranslation[0]; // R(a, b, c).x = c
}

// IMU updates every 4 ms; with optical data an extra update 0.5 ms later (~half the intervals short).
std::vector<Sample> Run(TrackingFilter& f, const std::vector<Phase>& phases, std::vector<Sample>* emitted = nullptr) {
    std::vector<Sample> out;
    double t0 = 0;
    for (const Phase& ph : phases) {
        for (double local = 0; local < ph.duration - 1e-9; local += 0.004) {
            const double t = t0 + local;
            if (!ph.stream) {
                vr::DriverPose_t e;
                if (emitted && f.Emit(t, &e)) emitted->push_back({t, e, e, f.GetState()});
                continue;
            }
            auto feed = [&](double tt, double l) {
                vr::DriverPose_t in = MakePose(ph.x(l), ph.vx, ph.result, ph.valid, ph.rotatedFrame);
                vr::DriverPose_t o = f.Process(in, tt);
                out.push_back({tt, in, o, f.GetState()});
            };
            feed(t, local);
            if (ph.optical) feed(t + 0.0005, local + 0.0005);
        }
        t0 += ph.duration;
    }
    return out;
}

bool Same(const vr::DriverPose_t& a, const vr::DriverPose_t& b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

void TestPassThrough() {
    std::printf("pass-through while tracking\n");
    TrackingFilter f;
    auto s = Run(f, {{3.0, true, vr::TrackingResult_Running_OK, true, true, [](double l) { return 0.1 * l; }, 0.1}});
    int changed = 0;
    for (auto& x : s) changed += !Same(x.in, x.out);
    CHECK(changed == 0, "%d poses changed", changed);
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
}

// Optical lost: IMU-only cadence, result flips at 250 ms, reset (invalid + stream stop) at 2 s.
std::vector<Phase> LossPhases(double drift) {
    auto driftX = [drift](double l) { return 0.5 + drift * l; };
    return {
        {2.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.5; }},
        {0.25, false, vr::TrackingResult_Running_OK, true, true, driftX, drift},
        {1.75, false, vr::TrackingResult_Calibrating_OutOfRange, true, true, [=](double l) { return driftX(0.25 + l); }, drift},
        {0.004, false, vr::TrackingResult_Calibrating_OutOfRange, false, true, [](double) { return 0.0; }},
        {2.0, false, vr::TrackingResult_Calibrating_OutOfRange, false, false},
    };
}

void TestLossHold(bool suspectEnabled) {
    std::printf("loss -> hold (suspect %s)\n", suspectEnabled ? "on" : "off");
    Config c;
    c.suspectEnabled = suspectEnabled;
    TrackingFilter f(c);
    std::vector<Sample> emitted;
    auto s = Run(f, LossPhases(0.1), &emitted);

    double firstHold = -1, worst = 0, maxVel = 0, heldX = 0;
    for (auto& x : s) {
        if (x.t < 2.0) continue;
        if (firstHold < 0 && x.state != State::Tracking) {
            firstHold = x.t - 2.0;
            heldX = x.out.vecPosition[0];
        }
        if (x.state != State::Tracking)
            CHECK(std::fabs(x.out.vecPosition[0] - heldX) < 1e-9, "held output moved at %.3f", x.t);
        if (x.state != State::Tracking) {
            worst = std::max(worst, std::fabs(x.out.vecPosition[0] - 0.5));
            maxVel = std::max(maxVel, std::fabs(x.out.vecVelocity[0]));
            CHECK(x.out.poseIsValid && x.out.result == vr::TrackingResult_Running_OK, "held pose reported invalid at %.3f", x.t);
        }
    }
    const double expectDetect = suspectEnabled ? 0.07 : 0.26;
    CHECK(firstHold >= 0 && firstHold <= expectDetect, "detected after %.3f s", firstHold);
    // Tracking until detection lets some drift through (up to 0.1 m/s * detect time), then the
    // output stays exactly where it was.
    CHECK(worst <= 0.1 * expectDetect + 1e-6, "worst offset %.4f m", worst);
    CHECK(maxVel == 0, "held velocity %.3f", maxVel);
    CHECK(f.GetState() == State::Hold, "state %s", ToString(f.GetState()));

    CHECK(!emitted.empty(), "nothing emitted while the stream stopped");
    for (auto& e : emitted) {
        CHECK(std::fabs(e.out.vecPosition[0] - heldX) < 1e-9, "emitted x %.4f, held %.4f", e.out.vecPosition[0], heldX);
        CHECK(e.out.poseIsValid && e.out.vecVelocity[0] == 0 && e.out.poseTimeOffset == 0, "emitted pose fields");
    }
}

void TestBadBootstrapThenRecovery() {
    std::printf("bad bootstrap rejected, good one blended\n");
    TrackingFilter f;
    auto phases = LossPhases(0.0);
    // Bad bootstrap: OK for 250 ms while drifting 0.45 m/s, then out of range and drifting on.
    phases.push_back({0.25, true, vr::TrackingResult_Running_OK, true, true, [](double l) { return 0.55 + 0.45 * l; }, 0.45});
    phases.push_back({1.0, false, vr::TrackingResult_Calibrating_OutOfRange, true, true, [](double l) { return 0.66 + 0.45 * l; }, 0.45});
    // Good bootstrap 5 cm away from the held position.
    phases.push_back({2.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.55; }});
    auto s = Run(f, phases);

    const double badStart = 6.004, goodStart = badStart + 1.25;
    double worstBad = 0;
    for (auto& x : s)
        if (x.t >= badStart && x.t < goodStart) worstBad = std::max(worstBad, std::fabs(x.out.vecPosition[0] - 0.5));
    CHECK(worstBad < 1e-6, "bad bootstrap moved the output by %.3f m", worstBad);

    double maxStep = 0, prev = 0.5;
    bool sawBlend = false;
    for (auto& x : s) {
        if (x.t < goodStart) continue;
        maxStep = std::max(maxStep, std::fabs(x.out.vecPosition[0] - prev));
        prev = x.out.vecPosition[0];
        sawBlend |= x.state == State::Blend;
        if (x.t < goodStart + 0.39) CHECK(std::fabs(x.out.vecPosition[0] - 0.5) < 1e-6, "moved during probation at %.3f", x.t);
    }
    CHECK(sawBlend, "no blend");
    CHECK(maxStep < 0.005, "blend step %.4f m per update", maxStep);
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
    CHECK(Same(s.back().in, s.back().out), "not pass-through after recovery");
}

void TestShortDropoutUsesShortProbation() {
    std::printf("short dropout without reset recovers after the short probation\n");
    TrackingFilter f;
    // Out of range for 0.5 s (no invalid pose, no stream stop), then OK again 3 cm away.
    auto s = Run(f, {
                        {2.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.5; }},
                        {0.5, false, vr::TrackingResult_Calibrating_OutOfRange, true, true, [](double) { return 0.51; }},
                        {1.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.53; }},
                    });
    const double resume = 2.5;
    double blendAt = -1, trackingAt = -1;
    for (auto& x : s) {
        if (x.t < resume) continue;
        if (blendAt < 0 && x.state == State::Blend) blendAt = x.t - resume;
        if (trackingAt < 0 && x.state == State::Tracking) trackingAt = x.t - resume;
    }
    const Config c;
    CHECK(blendAt >= c.shortProbation - 1e-6 && blendAt < c.shortProbation + 0.01, "blend started %.3f s after resume", blendAt);
    CHECK(trackingAt > 0 && trackingAt < c.shortProbation + c.blend + 0.01, "tracking again %.3f s after resume", trackingAt);
    CHECK(Same(s.back().in, s.back().out), "not pass-through after recovery");
}

void TestResetKeepsLongProbation() {
    std::printf("loss with a lighthouse reset keeps the long probation\n");
    TrackingFilter f;
    auto phases = LossPhases(0.0);
    phases.push_back({2.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.52; }});
    auto s = Run(f, phases);
    const double resume = 6.004;
    double blendAt = -1;
    for (auto& x : s)
        if (x.t >= resume && blendAt < 0 && x.state == State::Blend) blendAt = x.t - resume;
    const Config c;
    CHECK(blendAt >= c.probation - 1e-6 && blendAt < c.probation + 0.01, "blend started %.3f s after resume", blendAt);
}

void TestFrameChangeOnRebootstrap() {
    std::printf("held position survives a driver-space change\n");
    TrackingFilter f;
    auto phases = LossPhases(0.0);
    // Tracking resumes in a different driver frame (other base station), same world position.
    Phase resumed{2.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.5; }};
    resumed.rotatedFrame = true;
    phases.push_back(resumed);
    std::vector<Sample> emitted;
    auto s = Run(f, phases, &emitted);
    double worst = 0;
    for (auto& x : s)
        if (x.out.poseIsValid) worst = std::max(worst, std::fabs(WorldX(x.out) - 0.5));
    for (auto& e : emitted) worst = std::max(worst, std::fabs(WorldX(e.out) - 0.5));
    CHECK(worst < 1e-9, "world x moved by %.6f m", worst);
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
    CHECK(Same(s.back().in, s.back().out), "not pass-through after recovery");
}

void TestStepLimiter() {
    std::printf("step limiter\n");
    TrackingFilter f;
    auto s = Run(f, {
                        {1.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.0; }},
                        {1.0, true, vr::TrackingResult_Running_OK, true, true, [](double) { return 0.20; }},
                    });
    double maxStep = 0, prev = 0;
    for (auto& x : s) {
        maxStep = std::max(maxStep, std::fabs(x.out.vecPosition[0] - prev));
        prev = x.out.vecPosition[0];
    }
    CHECK(maxStep < 0.01, "output step %.4f m", maxStep);
    CHECK(f.StepsLimited() == 1, "steps limited %llu", (unsigned long long)f.StepsLimited());
    CHECK(Same(s.back().in, s.back().out), "offset did not decay");
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
}

void TestSuspectFalseAlarm() {
    std::printf("suspect false alarm\n");
    Config c;
    c.suspectEnabled = true;
    TrackingFilter f(c);
    auto s = Run(f, {
                        {2.0, true, vr::TrackingResult_Running_OK, true, true},
                        {0.12, false, vr::TrackingResult_Running_OK, true, true},
                        {1.0, true, vr::TrackingResult_Running_OK, true, true},
                    });
    bool sawSuspect = false, sawHold = false;
    for (auto& x : s) {
        sawSuspect |= x.state == State::Suspect;
        sawHold |= x.state == State::Hold;
    }
    CHECK(sawSuspect, "no suspect");
    CHECK(!sawHold, "escalated to hold");
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
}

// 2 mm of 40 Hz shake on a slow drift: smoothing should cut the shake and still follow a fast move.
void TestJitterSmoothing() {
    std::printf("jitter smoothing\n");
    Config c;
    c.smoothing = 0.6;
    TrackingFilter f(c);
    auto shake = [](double l) { return 0.002 * std::sin(2 * 3.14159265 * 40 * l) + 0.01 * l; };
    auto s = Run(f, {
                        {2.0, true, vr::TrackingResult_Running_OK, true, true, shake, 0.01},
                        {0.5, true, vr::TrackingResult_Running_OK, true, true, [](double l) { return 0.02 + 1.0 * l; }, 1.0},
                    });
    double inRough = 0, outRough = 0;
    for (size_t i = 2; i < s.size() && s[i].t < 2.0; ++i)
        if (s[i].t > 1.0) {
            inRough += std::fabs(s[i].in.vecPosition[0] - 2 * s[i - 1].in.vecPosition[0] + s[i - 2].in.vecPosition[0]);
            outRough += std::fabs(s[i].out.vecPosition[0] - 2 * s[i - 1].out.vecPosition[0] + s[i - 2].out.vecPosition[0]);
        }
    CHECK(outRough < inRough * 0.3, "shake kept: %.5f of %.5f", outRough, inRough);
    const double lag = s.back().in.vecPosition[0] - s.back().out.vecPosition[0];
    CHECK(lag >= 0 && lag < 0.05, "lag behind a 1 m/s move: %.4f m", lag);
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
}

// The reported velocities are smoothed too (SteamVR extrapolates with them), and a resting device whose
// reported velocity only jitters is smoothed hard: rotation jitter used to raise the cutoff by itself.
void TestJitterSmoothingVelocities() {
    std::printf("jitter smoothing: velocities and rotation\n");
    Config c;
    c.smoothing = 0.6;
    TrackingFilter f(c);
    double inVel = 0, outVel = 0, inRot = 0, outRot = 0;
    for (int i = 0; i < 1000; ++i) {
        const double t = i * 0.002, noise = (i % 2 ? 1 : -1);
        vr::DriverPose_t in = MakePose(0.5 + 0.0005 * noise, 0.03 * noise, vr::TrackingResult_Running_OK, true);
        in.vecAngularVelocity[1] = 0.2 * noise;
        const double a = 0.004 * noise; // ~0.23 deg about y
        in.qRotation = {std::cos(a / 2), 0, std::sin(a / 2), 0};
        const vr::DriverPose_t o = f.Process(in, t);
        if (t < 1.0) continue;
        inVel += std::fabs(in.vecVelocity[0]) + std::fabs(in.vecAngularVelocity[1]);
        outVel += std::fabs(o.vecVelocity[0]) + std::fabs(o.vecAngularVelocity[1]);
        inRot += std::fabs(in.qRotation.y);
        outRot += std::fabs(o.qRotation.y);
    }
    CHECK(outVel < inVel * 0.1, "velocity jitter kept: %.4f of %.4f", outVel, inVel);
    CHECK(outRot < inRot * 0.1, "rotation jitter kept: %.5f of %.5f", outRot, inRot);
}

// A pose at exactly (0, 0, 0) in driver space that claims to be valid and OK is lighthouse's
// placeholder, not a position: hold instead of following it there.
void TestOriginPlaceholderIsHeld() {
    std::printf("origin placeholder is held, not followed\n");
    TrackingFilter f;
    double t = 0;
    for (; t < 1.0; t += 0.004) f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
    vr::DriverPose_t origin = MakePose(0, 0, vr::TrackingResult_Running_OK, true);
    origin.vecPosition[1] = 0; // all three zero
    double worst = 0;
    for (; t < 1.2; t += 0.004) {
        const vr::DriverPose_t o = f.Process(origin, t);
        worst = std::max(worst, std::fabs(o.vecPosition[0] - 0.5) + std::fabs(o.vecPosition[1] - 1.0));
        CHECK(o.poseIsValid, "held pose reported invalid at %.3f", t);
    }
    CHECK(worst < 1e-9, "output moved toward the origin by %.4f m", worst);
    CHECK(f.GetState() == State::Hold, "state %s", ToString(f.GetState()));

    // Before anything was trusted it is reported invalid rather than at the origin.
    TrackingFilter g;
    const vr::DriverPose_t o = g.Process(origin, 0.0);
    CHECK(!o.poseIsValid, "untrusted origin placeholder passed as valid");
}

void TestDisconnectResets() {
    std::printf("disconnect resets\n");
    TrackingFilter f;
    Run(f, LossPhases(0.0));
    CHECK(f.GetState() == State::Hold, "precondition: state %s", ToString(f.GetState()));
    vr::DriverPose_t p = MakePose(0, 0, vr::TrackingResult_Uninitialized, false);
    p.deviceIsConnected = false;
    vr::DriverPose_t o = f.Process(p, 10.0);
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
    CHECK(Same(p, o), "disconnected pose modified");
    vr::DriverPose_t e;
    CHECK(!f.Emit(10.1, &e), "emits after disconnect");
}

} // namespace

// --- Real loss shape (Phase 0 recordings, controller and Tundra alike): optical updates stop, ~0.2 s
// of IMU-only Running_OK, ~1.75 s of Calibrating_OutOfRange with valid IMU-propagated rotation, then
// "Resetting tracking": one invalid pose and silence until optics return.

vr::HmdQuaternion_t AboutY(double a) { return {std::cos(a / 2), 0, std::sin(a / 2), 0}; }

double AngleTo(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b) {
    const double d = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
    return 2 * std::acos(std::fmin(1.0, d));
}

Config ModelConfig() {
    Config c;
    c.hand.enabled = true; // the model path; the hand never becomes ready without references
    c.hand.side = 1;
    return c;
}

// The model path used to accept a loss rotation only within 100 ms of the previous accepted one, so a
// single pause froze the rotation for the rest of the episode.
void TestModelRotationSurvivesGaps() {
    std::printf("model path: rotation follows the whole out-of-range stream, across a pause\n");
    TrackingFilter f(ModelConfig());
    double t = 0;
    for (; t < 1.0; t += 0.004) {
        f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
        f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t + 0.0005);
    }
    const double lossStart = t;
    double worstAfterGap = 0;
    double angle = 0;
    for (; t < lossStart + 1.75; t += 0.004) {
        if (t > lossStart + 0.6 && t < lossStart + 0.75) continue; // 150 ms with no updates
        angle = 1.0 * (t - lossStart);                                  // turning at 1 rad/s
        vr::DriverPose_t in = MakePose(0.5, 0, vr::TrackingResult_Calibrating_OutOfRange, true);
        in.qRotation = AboutY(angle);
        const vr::DriverPose_t o = f.Process(in, t);
        if (t > lossStart + 0.8) worstAfterGap = std::fmax(worstAfterGap, AngleTo(o.qRotation, in.qRotation));
    }
    CHECK(worstAfterGap < 0.02, "rotation stuck after the pause: %.3f rad behind", worstAfterGap);
    vr::DriverPose_t reset = MakePose(0.5, 0, vr::TrackingResult_Calibrating_OutOfRange, false);
    f.Process(reset, t);
    vr::DriverPose_t e;
    bool emitted = false;
    for (double s = t + 0.004; s < t + 1.0; s += 0.004)
        if (f.Emit(s, &e)) emitted = true;
    CHECK(emitted, "nothing emitted after the reset");
    CHECK(AngleTo(e.qRotation, AboutY(angle)) < 0.02, "held rotation is not the last one: %.3f rad off", AngleTo(e.qRotation, AboutY(angle)));
    CHECK(e.poseIsValid, "held pose reported invalid");
}

// With a model on, a pause in the stream used to start a hold after 20 ms and force the long probation.
void TestShortStallIsNotALoss() {
    std::printf("model path: a 30 ms stall is not a loss\n");
    TrackingFilter f(ModelConfig());
    double t = 0;
    auto feed = [&](double until) {
        for (; t < until; t += 0.004) {
            f.Process(MakePose(0.5 + t, 1.0, vr::TrackingResult_Running_OK, true), t);
            f.Process(MakePose(0.5 + t, 1.0, vr::TrackingResult_Running_OK, true), t + 0.0005);
        }
    };
    feed(1.0);
    vr::DriverPose_t e;
    bool emitted = false;
    for (double s = t; s < t + 0.030; s += 0.004)
        if (f.Emit(s, &e)) emitted = true;
    t += 0.030;
    feed(t + 0.5);
    CHECK(!emitted, "held poses emitted during a 30 ms stall");
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
    for (const auto& tr : f.TakeTransitions()) CHECK(false, "unexpected transition to %s (%s)", ToString(tr.to), tr.reason);
}

void TestHeldPoseHasNoVelocityWithSmoothing() {
    std::printf("jitter smoothing: a held pose reports no velocity\n");
    Config c;
    c.smoothing = 1.0;
    TrackingFilter f(c);
    double t = 0;
    for (; t < 1.0; t += 0.004) {
        f.Process(MakePose(0.5 + t, 1.0, vr::TrackingResult_Running_OK, true), t);
        f.Process(MakePose(0.5 + t, 1.0, vr::TrackingResult_Running_OK, true), t + 0.0005);
    }
    double worst = 0;
    for (double s = t; s < t + 0.3; s += 0.004) {
        const vr::DriverPose_t o = f.Process(MakePose(0.5 + t, 1.0, vr::TrackingResult_Calibrating_OutOfRange, true), s);
        worst = std::fmax(worst, std::fabs(o.vecVelocity[0]) + std::fabs(o.vecVelocity[1]) + std::fabs(o.vecVelocity[2]));
    }
    CHECK(f.GetState() == State::Hold, "state %s", ToString(f.GetState()));
    CHECK(worst == 0, "held pose reports %.3f m/s", worst);
}

void TestNonFiniteInputIsRejected() {
    std::printf("a non-finite pose never reaches the output\n");
    for (double smoothing : {0.0, 0.5}) {
        Config c;
        c.smoothing = smoothing;
        TrackingFilter f(c);
        double t = 0;
        for (; t < 1.0; t += 0.004) f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
        vr::DriverPose_t bad = MakePose(0.5, 0, vr::TrackingResult_Running_OK, true);
        bad.vecPosition[0] = std::nan("");
        bad.vecVelocity[0] = std::nan("");
        f.Process(bad, t);
        bool finite = true;
        for (t += 0.004; t < 2.0; t += 0.004) {
            const vr::DriverPose_t o = f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
            for (int i = 0; i < 3; ++i) finite = finite && std::isfinite(o.vecPosition[i]) && std::isfinite(o.vecVelocity[i]);
        }
        CHECK(finite, "non-finite output after one bad pose (smoothing %.1f)", smoothing);
    }
}

void TestNonFiniteInputBeforeTrusted() {
    std::printf("a non-finite pose before the first good one does not stay in the smoothing\n");
    Config c;
    c.smoothing = 0.5;
    TrackingFilter f(c);
    double t = 0;
    for (int i = 0; i < 2; ++i, t += 0.004) {
        vr::DriverPose_t bad = MakePose(0.5, 0, vr::TrackingResult_Calibrating_InProgress, true);
        bad.vecVelocity[0] = std::nan("");
        f.Process(bad, t);
    }
    bool finite = true;
    for (; t < 1.0; t += 0.004) {
        const vr::DriverPose_t o = f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
        finite = finite && std::isfinite(o.vecPosition[0]) && std::isfinite(o.vecVelocity[0]);
    }
    CHECK(finite, "non-finite output after a bad pose while untrusted");
}

void TestProbationNeedsUnbrokenInput() {
    std::printf("a pause during probation restarts it\n");
    TrackingFilter f{Config{}};
    double t = 0;
    for (; t < 1.0; t += 0.004) f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
    f.Process(MakePose(0.5, 0, vr::TrackingResult_Calibrating_OutOfRange, true), t);
    t += 0.004;
    f.Process(MakePose(0.9, 0, vr::TrackingResult_Running_OK, true), t); // one good pose, then silence
    CHECK(f.GetState() == State::Probation, "state %s", ots::filter::ToString(f.GetState()));
    t += 1.0;
    f.Process(MakePose(0.9, 0, vr::TrackingResult_Running_OK, true), t);
    CHECK(f.GetState() == State::Probation, "two poses a second apart passed probation: %s", ots::filter::ToString(f.GetState()));
}

void TestReleaseTellsSteamVR() {
    std::printf("giving up a hold sends one untracked pose\n");
    Config c;
    c.maxHold = 0.5;
    TrackingFilter f(c);
    double t = 0;
    for (; t < 1.0; t += 0.004) f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
    f.Process(MakePose(0.5, 0, vr::TrackingResult_Calibrating_OutOfRange, false), t);
    int valid = 0, invalid = 0;
    vr::DriverPose_t e;
    for (double s = t + 0.004; s < t + 1.5; s += 0.004)
        if (f.Emit(s, &e)) (e.poseIsValid ? valid : invalid)++;
    CHECK(valid > 0, "nothing held before the time limit");
    CHECK(invalid == 1, "%d untracked poses sent (want 1)", invalid);
    CHECK(f.GetState() == State::Tracking, "state %s", ToString(f.GetState()));
}

// Once lighthouse goes silent the held pose turns with the device's own IMU (when the driver has one).
void TestImuRotationWhileSilent() {
    std::printf("held pose turns with the IMU once lighthouse is silent\n");
    for (bool model : {false, true}) {
        TrackingFilter f(model ? ModelConfig() : Config{});
        double t = 0;
        for (; t < 1.0; t += 0.004) f.Process(MakePose(0.5, 0, vr::TrackingResult_Running_OK, true), t);
        f.Process(MakePose(0.5, 0, vr::TrackingResult_Calibrating_OutOfRange, false), t); // the reset pose
        vr::DriverPose_t e;
        double worst = 0;
        int used = 0;
        for (double s = t + 0.004; s < t + 2.0; s += 0.004) {
            const auto imu = AboutY(0.8 * (s - t));
            f.SetImuRotation(imu, s);
            if (!f.Emit(s, &e)) continue;
            if (s > t + 0.2) worst = std::fmax(worst, AngleTo(e.qRotation, imu)), used += f.ImuActive();
        }
        CHECK(used > 0 && worst < 1e-6, "%s path: IMU rotation not used (%d, %.3f rad off)", model ? "model" : "plain", used, worst);
        // Stale IMU data is ignored: the last rotation is kept.
        const double later = t + 3.0;
        f.Emit(later, &e);
        CHECK(!f.ImuActive(), "%s path: stale IMU rotation used", model ? "model" : "plain");
    }
}

int main() {
    TestPassThrough();
    TestLossHold(true);
    TestLossHold(false);
    TestBadBootstrapThenRecovery();
    TestShortDropoutUsesShortProbation();
    TestJitterSmoothing();
    TestJitterSmoothingVelocities();
    TestOriginPlaceholderIsHeld();
    TestResetKeepsLongProbation();
    TestFrameChangeOnRebootstrap();
    TestStepLimiter();
    TestSuspectFalseAlarm();
    TestDisconnectResets();
    TestModelRotationSurvivesGaps();
    TestShortStallIsNotALoss();
    TestHeldPoseHasNoVelocityWithSmoothing();
    TestNonFiniteInputIsRejected();
    TestNonFiniteInputBeforeTrusted();
    TestProbationNeedsUnbrokenInput();
    TestReleaseTellsSteamVR();
    TestImuRotationWhileSilent();
    std::printf(g_failures ? "%d FAILURE(S)\n" : "all filter tests passed\n", g_failures);
    return g_failures ? 1 : 0;
}
