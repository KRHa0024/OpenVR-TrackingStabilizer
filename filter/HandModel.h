#pragma once
#include "WaistModel.h"

namespace ots::filter {
struct HandConfig {
    bool enabled = false;
    int side = 0; // -1 left, +1 right; assigned from the runtime's controller role
    double shoulderWidth = .36;
    double armReach = .70; // shoulder to controller origin, including the hand
    double maxPrediction = 3.0; // lighthouse sends rotation for ~2 s after a loss; the shoulder follow needs none
    // Arm model: while the hand's rotation keeps arriving (lighthouse's out-of-range rotation, or the IMU),
    // the hand turns about an elbow pivot by this share of its rotation since the loss (0 = off).
    double armFollow = .75;
    double forearm = .26;                // elbow to controller origin, metres
    double maxPredictionTurning = 30;    // seconds the estimate may run while rotation keeps arriving
    // Move the shoulder (and with it elbow and hand) with the HMD-derived neck. Off: the shoulder stays
    // where it was at the loss and only the arm model's turn moves the hand.
    bool followBody = true;
    // The first this-many metres of the neck's movement since the loss are not followed: looking around
    // and leaning move the head (and the estimated shoulder) while the hand usually stays put.
    // Replays (tools/analysis/compare_hand_models.py, 1 s / 3 s pseudo-occlusions): 10 cm kept a still
    // user's error at the plain hold's level while moving users stay at the arm model's.
    double followDeadzone = .10;
};

// Shoulder-relative position hold with a reach constraint. Head yaw is frozen at loss onset: looking
// sideways must not swing an occluded hand around the head. The other hand is only a consistency cue.
// With the hand's rotation (arm model) a turn of the controller also swings the hand about an elbow,
// placed `forearm` from the controller along the direction it learned the shoulder to be in, in the
// controller's own axes (how the user holds it). Movement without rotation (reaching) is not seen.
class HandModel {
public:
    void Configure(const HandConfig& cfg, const BodyConfig& neck);
    void Reset();
    void Reference(const vr::DriverPose_t& head, double headTime,
                   const vr::DriverPose_t& other, double otherTime);
    void Observe(const vr::DriverPose_t& hand, double t);
    // `lostSince`: when the hand was last seen (earlier than `t` when its updates simply stopped).
    // `rotation`: the hand's world rotation at the loss (the arm model's reference).
    void BeginLoss(math::Vec output, math::Quat rotation, double t, double lostSince);
    // `rotation` / `rotationTime`: the hand's latest known rotation (rotationTime < 0: none).
    bool Predict(double t, math::Quat rotation, double rotationTime, math::Vec* out, math::Vec* shoulder = nullptr);
    bool Ready() const { return ready_; }
    const ModelDebug& Debug() const { return debug_; }
private:
    bool Neck(double t, math::Vec* position, math::Quat* yaw) const;
    bool Other(double t, math::Vec* position) const;
    HandConfig cfg_;
    double neckDown_ = .12, neckBack_ = .08;
    vr::DriverPose_t head_{}, other_{};
    double headTime_ = -1, otherTime_ = -1, healthyTime_ = -1, lossTime_ = -1;
    double stableSince_ = -1;
    math::Vec shoulderOffset_{}, handOffset_{}, neckAtLoss_{}, otherAtLoss_{};
    math::Vec previousNeck_{};
    bool ready_ = false, lossReady_ = false, haveOtherAtLoss_ = false;
    // arm model
    math::Vec forearmDir_{0, 0, 1}; // controller axes, unit: towards the elbow
    bool haveForearm_ = false, armReady_ = false;
    math::Quat rotationAtLoss_{1, 0, 0, 0}, armTurn_{1, 0, 0, 0};
    math::Vec elbowOffset_{}, handFromElbow_{};
    ModelDebug debug_;
};
} // namespace ots::filter
