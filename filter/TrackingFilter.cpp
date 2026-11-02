#include "TrackingFilter.h"

#include <algorithm>
#include <cmath>

namespace ots::filter {
namespace {

// A position of exactly (0, 0, 0) in driver space is lighthouse's "no pose" placeholder (driver space
// is anchored at a base station, so a real device is never there). Treat it as invalid even if the
// pose claims otherwise, so it is never output or remembered as the last good position.
bool AtDriverOrigin(const vr::DriverPose_t& p) {
    return p.vecPosition[0] == 0 && p.vecPosition[1] == 0 && p.vecPosition[2] == 0;
}

// Valid, not the origin placeholder, and finite throughout (math::Usable checks position and rotation;
// one non-finite velocity would otherwise poison the step limiter and the smoothing for good).
bool IsUsable(const vr::DriverPose_t& p) {
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(p.vecVelocity[i]) || !std::isfinite(p.vecAngularVelocity[i])) return false;
    return math::Usable(p) && !AtDriverOrigin(p);
}

bool IsGood(const vr::DriverPose_t& p) {
    return IsUsable(p) && p.result == vr::TrackingResult_Running_OK;
}

double Smoothstep(double u) {
    u = std::clamp(u, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

// Rotates v by unit quaternion q (w, x, y, z); conjugate=true rotates by q^-1.
void Rotate(const vr::HmdQuaternion_t& q, const double v[3], double out[3], bool conjugate = false) {
    const double s = conjugate ? -1.0 : 1.0;
    const double qx = q.x * s, qy = q.y * s, qz = q.z * s, qw = q.w;
    const double cx = qy * v[2] - qz * v[1], cy = qz * v[0] - qx * v[2], cz = qx * v[1] - qy * v[0];
    const double ccx = qy * cz - qz * cy, ccy = qz * cx - qx * cz, ccz = qx * cy - qy * cx;
    out[0] = v[0] + 2 * (qw * cx + ccx);
    out[1] = v[1] + 2 * (qw * cy + ccy);
    out[2] = v[2] + 2 * (qw * cz + ccz);
}

} // namespace

// Lighthouse's driver space is anchored to the base station it bootstrapped from, so it changes
// when tracking restarts from another base. Everything the filter keeps across updates (held
// position, history, blend origin, step offset) is therefore stored in world space and mapped back
// into the driver space of the pose being output.
TrackingFilter::Vec3 TrackingFilter::ToWorld(const vr::DriverPose_t& p) {
    double r[3];
    Rotate(p.qWorldFromDriverRotation, p.vecPosition, r);
    return {r[0] + p.vecWorldFromDriverTranslation[0], r[1] + p.vecWorldFromDriverTranslation[1],
            r[2] + p.vecWorldFromDriverTranslation[2]};
}

void TrackingFilter::SetFromWorld(vr::DriverPose_t& p, const Vec3& w) {
    const double d[3] = {w.x - p.vecWorldFromDriverTranslation[0], w.y - p.vecWorldFromDriverTranslation[1],
                         w.z - p.vecWorldFromDriverTranslation[2]};
    Rotate(p.qWorldFromDriverRotation, d, p.vecPosition, /*conjugate=*/true);
}

Config TrackingFilter::Sanitized(Config c) {
    auto atLeast = [](double& v, double min) { if (!std::isfinite(v) || v < min) v = min; };
    atLeast(c.suspectWindow, 0.005);
    atLeast(c.suspectMax, 0.005);
    atLeast(c.streamTimeout, 0.005);
    atLeast(c.resetGap, c.streamTimeout);
    atLeast(c.probation, 0);
    atLeast(c.shortProbation, 0);
    atLeast(c.blend, 0);
    atLeast(c.stepLimit, 0.005);
    atLeast(c.stepBlend, 0);
    atLeast(c.stepMaxDt, 0.001);
    atLeast(c.maxHold, 0);
    if (!std::isfinite(c.smoothing)) c.smoothing = 0;
    return c;
}

const char* ToString(State s) {
    switch (s) {
    case State::Tracking: return "TRACKING";
    case State::Suspect: return "SUSPECT";
    case State::Hold: return "HOLD";
    case State::Probation: return "PROBATION";
    case State::Blend: return "BLEND";
    }
    return "?";
}

std::vector<Transition> TrackingFilter::TakeTransitions() {
    std::vector<Transition> out;
    out.swap(transitions_);
    return out;
}

void TrackingFilter::SetState(State s, double t, const char* reason) {
    if (s == state_) return;
    transitions_.push_back({t, state_, s, reason});
    state_ = s;
    stateSince_ = t;
}

void TrackingFilter::Reset(double t, const char* reason) {
    SetState(State::Tracking, t, reason);
    trusted_ = false;
    offsetActive_ = false;
    body_.Reset();
    hand_.Reset();
    bodyActive_ = false;
    handActive_ = false;
}

bool TrackingFilter::ShortIntervalsSince(double since, int atLeast) const {
    int n = 0;
    for (auto it = shortTimes_.rbegin(); it != shortTimes_.rend() && *it > since; ++it)
        if (++n >= atLeast) return true;
    return false;
}

bool TrackingFilter::Armed(double) const {
    return cfg_.suspectEnabled && int(shortTimes_.size()) >= cfg_.suspectArmCount;
}

void TrackingFilter::EnterHold(double t, const char* reason, double lostSince) {
    if (state_ == State::Tracking || state_ == State::Blend) {
        // Hold exactly where the output is now. Rewinding to the estimated loss time was tried and
        // dropped: for a moving limb it jumps back by speed x rewind (up to ~37 cm in E4).
        hold_ = lastOut_;
        holdStart_ = t;
        resetSeen_ = false; // new loss episode
        holdRotation_ = lastRotation_;
        rotationT_ = lighthouseRotationT_ = lastInputT_;
        imuEpisode_ = false;
        bodyUpdateT_ = t;
        bodyAnchored_ = bodyRejected_ = false;
        hand_.BeginLoss({hold_.x,hold_.y,hold_.z},lastRotation_,t,lostSince >= 0 ? lostSince : t);
    }
    // From SUSPECT / PROBATION the hold position is already set.
    SetState(State::Hold, t, reason);
}

void TrackingFilter::UpdateBodyHold(double t) {
    bodyActive_ = false;
    if (!cfg_.body.enabled || bodyRejected_) return;
    math::Vec estimate;
    if (body_.Predict(t, holdRotation_, rotationT_, &estimate)) {
        // Limit movement both at model entry and when a reference returns. A rejected
        // estimate freezes the current output instead of jumping back to the loss origin.
        const math::Vec current{hold_.x,hold_.y,hold_.z};
        if (!bodyAnchored_) {
            bodyBias_ = current-estimate;
            // A poor model fit at loss onset is evidence against using this episode.
            if (math::Length(bodyBias_) > .10) { bodyRejected_ = true; return; }
            bodyAnchored_ = true;
        }
        estimate = estimate+bodyBias_;
        const auto delta = estimate-current;
        const double distance = math::Length(delta);
        if (distance <= .35) {
            const double step = std::min(.05, 1.5*std::max(0.0,t-bodyUpdateT_));
            const auto next = current + delta*(distance > step && distance > 0 ? step/distance : 1.0);
            hold_ = {next.x,next.y,next.z};
            bodyActive_ = true;
        }
    }
    bodyUpdateT_ = t;
}

void TrackingFilter::UpdateHandHold(double t) {
    handActive_ = false;
    math::Vec estimate, shoulder;
    if (cfg_.hand.enabled && hand_.Predict(t,holdRotation_,rotationT_,&estimate,&shoulder)) {
        const math::Vec current{hold_.x,hold_.y,hold_.z};
        const auto delta = estimate-current;
        const double distance = math::Length(delta);
        if (distance <= .35) {
            const double step = std::min(.05,1.5*std::max(0.0,t-bodyUpdateT_));
            auto next = current+delta*(distance > step && distance > 0 ? step/distance : 1.0);
            const auto arm = next-shoulder;
            const double reach = math::Length(arm);
            if (reach > cfg_.hand.armReach) next = shoulder+arm*(cfg_.hand.armReach/reach);
            // If reach and continuity cannot both be met, retain position hold.
            if (math::Length(next-current) <= step+1e-9) {
                hold_ = {next.x,next.y,next.z};
                handActive_ = true;
            }
        }
    }
    bodyUpdateT_ = t;
}

// The rotation output while the position is held (model path). Every usable input carries a real
// rotation: IMU-propagated ones while lighthouse reports the device out of range (~1.75 s after an
// optical loss), and good ones in SUSPECT and PROBATION. A turn faster than the allowance since the
// last accepted rotation is not taken:
// - out of range (IMU only): a glitch. After a pause, or 100 ms of such turns, the device's own report
//   is taken again, so one gap never freezes the rotation for the rest of the episode;
// - good (optical) input: a correction made by a fresh bootstrap. It stays withheld and BLEND turns
//   to it smoothly.
void TrackingFilter::AcceptRotation(const vr::DriverPose_t& in, double t) {
    if (!IsUsable(in)) return;
    const auto q = math::WorldRotation(in);
    const double dt = t - rotationT_;
    if (dt < 0) return;
    const bool reseed = !IsGood(in) && t - lighthouseRotationT_ > .1;
    if (reseed || math::Angle(holdRotation_, q) <= .15 + 8 * std::min(dt, .1)) {
        holdRotation_ = q;
        rotationT_ = lighthouseRotationT_ = t;
    }
}

// Lighthouse's rotation stopped coming (it sends one every ~4 ms while out of range, then nothing after
// its reset): carry on with the IMU's, which was anchored to that same last rotation.
void TrackingFilter::UseImuRotation(double t) {
    if (!ImuFresh(t) || t - lighthouseRotationT_ <= .02) return;
    holdRotation_ = imuRotation_;
    rotationT_ = t; // keeps the body model going
    imuActive_ = imuEpisode_ = true;
}

vr::DriverPose_t TrackingFilter::HoldPose(const vr::DriverPose_t& in, double t) {
    imuActive_ = false;
    AcceptRotation(in, t);
    UseImuRotation(t);
    if (ModelEnabled() && state_ != State::Suspect) {
        if (cfg_.body.enabled) UpdateBodyHold(t);
        else UpdateHandHold(t);
    }
    // An invalid input (lighthouse's reset pose) carries nothing usable: start from the last valid one.
    vr::DriverPose_t out = IsUsable(in) || !haveValid_ ? in : lastValid_;
    SetFromWorld(out, hold_);
    if (ManagedRotation()) {
        math::SetRotation(out,holdRotation_);
        for (int i = 0; i < 3; ++i) out.vecAngularVelocity[i] = out.vecAngularAcceleration[i] = 0;
    }
    for (int i = 0; i < 3; ++i) {
        out.vecVelocity[i] = 0;
        out.vecAcceleration[i] = 0;
    }
    if (!IsUsable(in)) {
        for (int i = 0; i < 3; ++i) {
            out.vecAngularVelocity[i] = 0;
            out.vecAngularAcceleration[i] = 0;
        }
    }
    if (cfg_.holdReportsValid && haveValid_) {
        out.poseIsValid = true;
        out.result = vr::TrackingResult_Running_OK;
    }
    out.deviceIsConnected = in.deviceIsConnected;
    return out;
}

vr::DriverPose_t TrackingFilter::BlendPose(const vr::DriverPose_t& in, double t) {
    // The live target still goes through the step limiter, so a jump in the input during the
    // blend is smoothed as well.
    const vr::DriverPose_t target = TrackingPose(in, t);
    const double u = cfg_.blend > 0 ? std::clamp((t - blendStart_) / cfg_.blend, 0.0, 1.0) : 1.0;
    const double s = Smoothstep(u);
    const Vec3 live = ToWorld(target);
    const Vec3 d{live.x - blendFrom_.x, live.y - blendFrom_.y, live.z - blendFrom_.z};
    vr::DriverPose_t out = target;
    if (ManagedRotation() && math::Unit(math::WorldRotation(target))) {
        math::SetRotation(out,math::Interpolate(blendRotation_,math::WorldRotation(target),s));
        // Suppress prediction across an optical rotation correction.
        for (int i = 0; i < 3; ++i) out.vecAngularVelocity[i] = out.vecAngularAcceleration[i] = 0;
    }
    SetFromWorld(out, {blendFrom_.x + d.x * s, blendFrom_.y + d.y * s, blendFrom_.z + d.z * s});

    // Report the velocity the output actually has (live velocity scaled by s plus the blend's
    // own motion), so SteamVR's prediction does not overshoot while the position catches up.
    const double ds = cfg_.blend > 0 ? 6.0 * u * (1.0 - u) / cfg_.blend : 0.0;
    double vWorld[3];
    Rotate(target.qWorldFromDriverRotation, target.vecVelocity, vWorld);
    const double vb[3] = {vWorld[0] * s + d.x * ds, vWorld[1] * s + d.y * ds, vWorld[2] * s + d.z * ds};
    Rotate(target.qWorldFromDriverRotation, vb, out.vecVelocity, /*conjugate=*/true);
    return out;
}

TrackingFilter::Vec3 TrackingFilter::StepOffset(double t) {
    if (!offsetActive_) return {};
    const double u = cfg_.stepBlend > 0 ? (t - offsetT0_) / cfg_.stepBlend : 1.0;
    if (u >= 1.0) {
        offsetActive_ = false;
        return {};
    }
    const double k = 1.0 - Smoothstep(u);
    return {offsetStart_.x * k, offsetStart_.y * k, offsetStart_.z * k};
}

vr::DriverPose_t TrackingFilter::TrackingPose(const vr::DriverPose_t& in, double t) {
    // Step limiter: a jump the reported velocity cannot explain is absorbed into an offset that
    // decays over stepBlend, so the output glides instead of teleporting.
    if (haveInput_ && IsGood(prevIn_) && t - lastInputT_ <= cfg_.stepMaxDt) {
        // Effective sample-time difference. Back-to-back optical and IMU updates can arrive 0.1 ms
        // apart with poseTimeOffsets that differ by more than that (E4: a 43 cm jump slipped
        // through when negative dt was skipped), so clamp instead of skipping.
        const double dt = std::max(0.0, (t + in.poseTimeOffset) - (lastInputT_ + prevIn_.poseTimeOffset));
        {
            const Vec3 prev = ToWorld(prevIn_), cur = ToWorld(in);
            double v[3];
            Rotate(prevIn_.qWorldFromDriverRotation, prevIn_.vecVelocity, v);
            const Vec3 r{cur.x - (prev.x + v[0] * dt), cur.y - (prev.y + v[1] * dt), cur.z - (prev.z + v[2] * dt)};
            if (std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z) > cfg_.stepLimit) {
                const Vec3 o = StepOffset(t);
                offsetStart_ = {o.x - r.x, o.y - r.y, o.z - r.z};
                offsetT0_ = t;
                offsetActive_ = true;
                ++stepsLimited_;
            }
        }
    }
    const Vec3 o = StepOffset(t);
    if (!offsetActive_) return in; // exact pass-through
    const Vec3 w = ToWorld(in);
    vr::DriverPose_t out = in;
    SetFromWorld(out, {w.x + o.x, w.y + o.y, w.z + o.z});
    return out;
}

// One Euro filter (Casiez et al. 2012) in world space. The cutoff rises with speed, so slow shakes are
// smoothed hard while fast movement keeps up.
// - Speed is the velocity the driver reports (IMU based, steady: ~2 cm/s and ~0.1 rad/s at rest), not a
//   difference of samples: at 400-1200 updates a second those are mostly the jitter itself, which kept
//   the cutoff high (rotation was hardly smoothed at all).
// - The reported velocities are smoothed with the same cutoff. SteamVR extrapolates every pose with them
//   (~40 ms ahead for rendering), so raw ones put the jitter straight back.
void TrackingFilter::Smooth(vr::DriverPose_t& out, double t) {
    constexpr double kPi = 3.14159265358979;
    const double s = std::clamp(cfg_.smoothing, 0.0, 1.0);
    const double minCutoff = 25.0 * std::pow(0.03, s); // Hz: 25 (barely) .. 0.75 (strong)
    constexpr double kBetaPos = 10.0, kBetaRot = 2.0;   // Hz per m/s, Hz per rad/s
    auto alpha = [&](double cutoff, double dt) { return 1.0 / (1.0 + 1.0 / (2 * kPi * cutoff * dt)); };
    auto len = [](const double* v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); };

    const Vec3 p = ToWorld(out);
    double q[4]; // world rotation = qWorldFromDriver * qRotation
    {
        const vr::HmdQuaternion_t& a = out.qWorldFromDriverRotation;
        const vr::HmdQuaternion_t& b = out.qRotation;
        q[0] = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
        q[1] = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
        q[2] = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
        q[3] = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    }
    const double dt = t - smoothT_;
    if (!smoothInit_ || dt <= 0 || dt > 0.2) {
        smoothInit_ = true;
        smoothT_ = t;
        smoothPos_ = p;
        std::copy(q, q + 4, smoothRot_);
        std::copy(out.vecVelocity, out.vecVelocity + 3, smoothVel_);
        std::copy(out.vecAngularVelocity, out.vecAngularVelocity + 3, smoothAngVel_);
        return;
    }
    smoothT_ = t;

    const double ap = alpha(minCutoff + kBetaPos * len(out.vecVelocity), dt);
    smoothPos_ = {smoothPos_.x + (p.x - smoothPos_.x) * ap, smoothPos_.y + (p.y - smoothPos_.y) * ap, smoothPos_.z + (p.z - smoothPos_.z) * ap};
    for (int i = 0; i < 3; ++i) out.vecVelocity[i] = smoothVel_[i] += (out.vecVelocity[i] - smoothVel_[i]) * ap;

    double d = smoothRot_[0] * q[0] + smoothRot_[1] * q[1] + smoothRot_[2] * q[2] + smoothRot_[3] * q[3];
    if (d < 0) { // same rotation, other hemisphere
        for (double& c : q) c = -c;
        d = -d;
    }
    const double ar = alpha(minCutoff + kBetaRot * len(out.vecAngularVelocity), dt);
    for (int i = 0; i < 3; ++i) out.vecAngularVelocity[i] = smoothAngVel_[i] += (out.vecAngularVelocity[i] - smoothAngVel_[i]) * ar;
    double n = 0;
    for (int i = 0; i < 4; ++i) {
        smoothRot_[i] += (q[i] - smoothRot_[i]) * ar; // nlerp
        n += smoothRot_[i] * smoothRot_[i];
    }
    n = std::sqrt(n);
    for (double& c : smoothRot_) c /= n;

    SetFromWorld(out, smoothPos_);
    // Back to driver space: qRotation = conj(qWorldFromDriver) * smoothed world rotation.
    const vr::HmdQuaternion_t& a = out.qWorldFromDriverRotation;
    const double cw = a.w, cx = -a.x, cy = -a.y, cz = -a.z;
    const double* r = smoothRot_;
    out.qRotation.w = cw * r[0] - cx * r[1] - cy * r[2] - cz * r[3];
    out.qRotation.x = cw * r[1] + cx * r[0] + cy * r[3] - cz * r[2];
    out.qRotation.y = cw * r[2] - cx * r[3] + cy * r[0] + cz * r[1];
    out.qRotation.z = cw * r[3] + cx * r[2] - cy * r[1] + cz * r[0];
}

vr::DriverPose_t TrackingFilter::Process(const vr::DriverPose_t& in, double t) {
    if (!in.deviceIsConnected) {
        Reset(t, "disconnected");
        haveInput_ = false;
        haveValid_ = false;
        smoothInit_ = false;
        shortTimes_.clear();
        return in;
    }

    if (!haveInput_ && state_ == State::Tracking) stateSince_ = t; // first input (or after a disconnect)

    // Cadence bookkeeping.
    if (haveInput_ && t - lastInputT_ < cfg_.shortInterval) shortTimes_.push_back(t);
    while (!shortTimes_.empty() && shortTimes_.front() < t - 1.0) shortTimes_.pop_front();

    const bool good = IsGood(in);
    bodyActive_ = false;
    handActive_ = false;
    imuActive_ = false;
    if (cfg_.body.enabled && state_ == State::Tracking && good) body_.Observe(in,t);
    if (cfg_.hand.enabled && state_ == State::Tracking && good) hand_.Observe(in,t);
    vr::DriverPose_t out = in;

    // A loss during which lighthouse reset tracking (it sends one invalid pose, then nothing)
    // ends with a fresh bootstrap, which may be bad: that is what the long probation is for.
    const bool inLoss = state_ == State::Suspect || state_ == State::Hold || state_ == State::Probation;
    if (inLoss && haveInput_ && t - lastInputT_ > cfg_.resetGap) resetSeen_ = true;

    switch (state_) {
    case State::Tracking:
        if (!good) {
            // Nothing trustworthy to hold yet (e.g. right after power-on): pass through, but never
            // as a valid pose at the placeholder origin.
            if (trusted_) {
                EnterHold(t, IsUsable(in) ? "result not OK" : "pose invalid");
                out = HoldPose(in, t);
            } else if (in.poseIsValid && AtDriverOrigin(in)) {
                out.poseIsValid = false;
            }
        } else if (Armed(t) && t - stateSince_ > cfg_.suspectWindow &&
                   !ShortIntervalsSince(t - cfg_.suspectWindow, 1)) {
            hold_ = lastOut_;
            holdStart_ = t;
            resetSeen_ = false;
            holdRotation_ = lastRotation_; rotationT_ = lighthouseRotationT_ = lastInputT_; bodyUpdateT_ = t;
            imuEpisode_ = false;
            bodyAnchored_ = bodyRejected_ = false;
            hand_.BeginLoss({hold_.x,hold_.y,hold_.z},lastRotation_,t,t);
            SetState(State::Suspect, t, "optical cadence stopped");
            out = HoldPose(in, t);
        } else {
            out = TrackingPose(in, t);
        }
        break;

    case State::Suspect:
        if (!good) {
            SetState(State::Hold, t, IsUsable(in) ? "result not OK" : "pose invalid");
            out = HoldPose(in, t);
        } else if (ShortIntervalsSince(t - cfg_.suspectWindow, 2) || t - stateSince_ > cfg_.suspectMax) {
            blendFrom_ = hold_;
            blendRotation_ = holdRotation_;
            blendStart_ = t;
            SetState(State::Blend, t, t - stateSince_ > cfg_.suspectMax ? "suspect timeout" : "optical cadence resumed");
            out = BlendPose(in, t);
        } else {
            out = HoldPose(in, t);
        }
        break;

    case State::Hold:
        if (cfg_.maxHold > 0 && t - holdStart_ > cfg_.maxHold) {
            Reset(t, "max hold exceeded");
            out = in;
        } else {
            if (good) {
                probationStart_ = t;
                SetState(State::Probation, t, "tracking resumed");
            }
            out = HoldPose(in, t);
        }
        break;

    case State::Probation:
        if (!good) {
            SetState(State::Hold, t, "lost during probation");
            out = HoldPose(in, t);
        } else if (haveInput_ && t - lastInputT_ > cfg_.resetGap) {
            probationStart_ = t; // probation asks for an unbroken run of good poses
            out = HoldPose(in, t);
        } else if (ModelEnabled() && haveInput_ && IsGood(prevIn_) &&
                   (math::Length(math::WorldPosition(in)-math::WorldPosition(prevIn_)) > .08 ||
                    !math::Unit(math::WorldRotation(in)) ||
                    math::Angle(math::WorldRotation(in),math::WorldRotation(prevIn_)) > .35)) {
            probationStart_ = t; // optical recovery must be internally consistent before blending
            out = HoldPose(in, t);
        } else if (t - probationStart_ >= (resetSeen_ ? cfg_.probation : cfg_.shortProbation)) {
            blendFrom_ = hold_;
            blendRotation_ = holdRotation_;
            blendStart_ = t;
            SetState(State::Blend, t, resetSeen_ ? "probation passed" : "short probation passed");
            out = BlendPose(in, t);
        } else {
            out = HoldPose(in, t);
        }
        break;

    case State::Blend:
        if (!good) {
            EnterHold(t, "lost during blend");
            out = HoldPose(in, t);
        } else if (t - blendStart_ >= cfg_.blend) {
            SetState(State::Tracking, t, "blend done");
            out = TrackingPose(in, t); // a step offset picked up during the blend keeps decaying
        } else {
            out = BlendPose(in, t);
        }
        break;
    }

    if (!IsUsable(in) && state_ != State::Tracking) resetSeen_ = true;
    if (cfg_.smoothing > 0 && IsUsable(out)) { // one non-finite value would stay in the filter's state
        // A held pose keeps the velocities HoldPose gave it (no linear motion): smoothed ones would
        // make SteamVR extrapolate the "held" device forward for a while.
        const bool held = state_ == State::Suspect || state_ == State::Hold || state_ == State::Probation;
        double v[3], w[3];
        std::copy(out.vecVelocity, out.vecVelocity + 3, v);
        std::copy(out.vecAngularVelocity, out.vecAngularVelocity + 3, w);
        Smooth(out, t);
        if (held) {
            std::copy(v, v + 3, out.vecVelocity);
            std::copy(w, w + 3, out.vecAngularVelocity);
            std::copy(v, v + 3, smoothVel_);
            std::copy(w, w + 3, smoothAngVel_);
        }
    } else {
        smoothInit_ = false;
    }
    if (IsUsable(out)) lastOut_ = ToWorld(out);
    if (math::Usable(out)) lastRotation_ = math::WorldRotation(out);
    if (state_ == State::Tracking && good) trusted_ = true;
    if (IsUsable(in)) {
        lastValid_ = in;
        haveValid_ = true;
    }
    prevIn_ = in;
    lastInputT_ = t;
    haveInput_ = true;
    return out;
}

bool TrackingFilter::Emit(double t, vr::DriverPose_t* out) {
    // A device whose updates stop without an invalid pose first: hold it (model path), but only after
    // a real pause; a USB / radio stall of a few tens of ms is not a loss.
    if (ModelEnabled() && (state_ == State::Tracking || state_ == State::Blend) && trusted_ &&
        haveInput_ && t-lastInputT_ > cfg_.resetGap) EnterHold(t,"pose stream stopped",lastInputT_); // lost since the last update
    if (state_ == State::Tracking || state_ == State::Blend || !haveValid_) return false;
    // Wait for a pause in the stream, except after lighthouse's reset pose: nothing follows it.
    if (haveInput_ && t - lastInputT_ <= cfg_.streamTimeout && IsUsable(prevIn_)) return false;
    if (cfg_.maxHold > 0 && t - holdStart_ > cfg_.maxHold) return Release(t, "max hold exceeded", out);
    if (!haveInput_ || t - lastInputT_ > cfg_.resetGap) resetSeen_ = true; // lighthouse stopped sending: it reset tracking
    imuActive_ = false;
    UseImuRotation(t);
    if (state_ != State::Suspect) {
        if (cfg_.hand.enabled) UpdateHandHold(t);
        else UpdateBodyHold(t);
    }
    *out = lastValid_;
    SetFromWorld(*out, hold_);
    // The held rotation (the model's / the IMU's), or the one last output (smoothed), not lastValid_'s
    // raw one: no step when the emitter takes over from Process.
    math::SetRotation(*out, ManagedRotation() ? holdRotation_ : lastRotation_);
    for (int i = 0; i < 3; ++i) {
        out->vecVelocity[i] = 0;
        out->vecAcceleration[i] = 0;
        out->vecAngularVelocity[i] = 0;
        out->vecAngularAcceleration[i] = 0;
    }
    out->poseTimeOffset = 0;
    lastOut_ = hold_;
    lastRotation_ = math::WorldRotation(*out);
    if (cfg_.holdReportsValid) {
        out->poseIsValid = true;
        out->result = vr::TrackingResult_Running_OK;
    }
    return true;
}

bool TrackingFilter::Release(double t, const char* reason, vr::DriverPose_t* out) {
    const bool held = state_ != State::Tracking && state_ != State::Blend && haveValid_;
    if (held) {
        *out = lastValid_;
        SetFromWorld(*out, hold_);
        math::SetRotation(*out, ManagedRotation() ? holdRotation_ : lastRotation_);
        for (int i = 0; i < 3; ++i)
            out->vecVelocity[i] = out->vecAcceleration[i] = out->vecAngularVelocity[i] = out->vecAngularAcceleration[i] = 0;
        out->poseTimeOffset = 0;
        out->poseIsValid = false;
        out->result = vr::TrackingResult_Running_OutOfRange;
    }
    Reset(t, reason);
    return held;
}

} // namespace ots::filter
