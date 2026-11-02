#pragma once
#include "PoseMath.h"

namespace ots::filter {
// The joints a model last worked with (world space, metres), for visualising it.
struct ModelDebug {
    double t = -1;           // when (the filter's clock); -1 = never
    bool estimating = false; // from a loss estimate; false = from a healthy observation
    bool haveShoulder = false, haveElbow = false;
    math::Vec neck, shoulder, elbow, end; // end: the hand (hand model) or the waist (waist model)
};

struct BodyConfig {
    bool enabled = false; // only the explicitly mapped waist receives this flag
    double neckDown = .12; // HMD origin -> neck pivot, meters in HMD coordinates
    double neckBack = .08;
};

// A head/neck segment and one rigid torso segment. Learns the torso length and the
// tracker mounting direction from healthy observations; never integrates acceleration.
class WaistModel {
public:
    void Configure(const BodyConfig& cfg);
    void Reset();
    void Reference(const vr::DriverPose_t& pose, double t);
    void Observe(const vr::DriverPose_t& waist, double t);
    bool Predict(double t, math::Quat waistRotation, double rotationTime, math::Vec* out) const;
    bool Ready() const { return ready_; }
    double TorsoLength() const { return math::Length(torso_); }
    const ModelDebug& Debug() const { return debug_; }
private:
    bool Neck(double t, math::Vec* out) const;
    BodyConfig cfg_;
    vr::DriverPose_t head_{};
    double headT_ = -1, sampleT_ = -1, stableSince_ = -1;
    unsigned samples_ = 0;
    math::Vec torso_{};
    bool ready_ = false;
    mutable ModelDebug debug_;
};
} // namespace ots::filter
