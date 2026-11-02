#include "HandModel.h"

namespace ots::filter {
using namespace math;

void HandModel::Configure(const HandConfig& c, const BodyConfig& neck) {
    if (c.enabled != cfg_.enabled || c.side != cfg_.side || c.shoulderWidth != cfg_.shoulderWidth ||
        c.armReach != cfg_.armReach || c.maxPrediction != cfg_.maxPrediction || c.armFollow != cfg_.armFollow ||
        c.forearm != cfg_.forearm || c.maxPredictionTurning != cfg_.maxPredictionTurning || c.followBody != cfg_.followBody || c.followDeadzone != cfg_.followDeadzone ||
        neck.neckDown != neckDown_ || neck.neckBack != neckBack_) Reset();
    cfg_ = c;
    neckDown_ = neck.neckDown;
    neckBack_ = neck.neckBack;
}

void HandModel::Reset() {
    ready_ = lossReady_ = haveOtherAtLoss_ = haveForearm_ = armReady_ = false;
    headTime_ = otherTime_ = healthyTime_ = stableSince_ = lossTime_ = -1;
}

void HandModel::Reference(const vr::DriverPose_t& head, double headTime,
                          const vr::DriverPose_t& other, double otherTime) {
    if (!head.deviceIsConnected) Reset();
    head_ = head; headTime_ = headTime;
    other_ = other; otherTime_ = otherTime;
}

bool HandModel::Neck(double t, Vec* position, Quat* yaw) const {
    if (!cfg_.enabled || (cfg_.side != -1 && cfg_.side != 1) || !std::isfinite(cfg_.armReach) ||
        !std::isfinite(cfg_.shoulderWidth) || !std::isfinite(cfg_.maxPrediction) || cfg_.maxPrediction <= 0 ||
        cfg_.armReach < .3 || cfg_.armReach > 1.2 || cfg_.shoulderWidth < .2 || cfg_.shoulderWidth > .6 || !Good(head_) ||
        !Unit(head_.qDriverFromHeadRotation) || t < headTime_ || t-headTime_ > .1) return false;
    const Quat deviceRotation = WorldRotation(head_);
    const Quat headRotation = Multiply(deviceRotation,head_.qDriverFromHeadRotation);
    const Vec headPosition = WorldPosition(head_) + Rotate(deviceRotation,
        {head_.vecDriverFromHeadTranslation[0],head_.vecDriverFromHeadTranslation[1],head_.vecDriverFromHeadTranslation[2]});
    *position = headPosition + Rotate(headRotation,{0,-neckDown_,neckBack_});
    const Vec forward = Rotate(headRotation,{0,0,-1});
    if (!Finite(*position) || std::hypot(forward.x,forward.z) < .3) return false;
    const double angle = std::atan2(-forward.x,-forward.z);
    *yaw = {std::cos(angle/2),0,std::sin(angle/2),0};
    return true;
}

bool HandModel::Other(double t, Vec* position) const {
    if (!Good(other_) || t < otherTime_ || t-otherTime_ > .1) return false;
    *position = WorldPosition(other_);
    return true;
}

void HandModel::Observe(const vr::DriverPose_t& hand, double t) {
    Vec neck; Quat yaw;
    if (!Good(hand) || !Neck(t,&neck,&yaw)) {
        ready_ = false; stableSince_ = -1; return;
    }
    const Vec shoulder = neck + Rotate(yaw,{cfg_.side*cfg_.shoulderWidth*.5,-.10,0});
    const double reach = Length(WorldPosition(hand)-shoulder);
    if (!std::isfinite(reach) || reach < .05 || reach > cfg_.armReach+.05) {
        ready_ = false; stableSince_ = -1; return;
    }
    if (stableSince_ < 0 || t < healthyTime_ || t-healthyTime_ > .1) stableSince_ = t;
    healthyTime_ = t;
    ready_ = t-stableSince_ >= .1;
    // Where the arm leaves the controller, in its own axes: the shoulder's direction, averaged.
    const Vec toShoulder = Rotate(Conjugate(WorldRotation(hand)), shoulder-WorldPosition(hand)) * (1/reach);
    if (Finite(toShoulder)) {
        const Vec mixed = haveForearm_ ? forearmDir_ + (toShoulder-forearmDir_)*.01 : toShoulder;
        const double n = Length(mixed);
        if (n > 1e-6) forearmDir_ = mixed*(1/n), haveForearm_ = true;
    }
    debug_ = {t, false, true, haveForearm_, neck, shoulder,
              WorldPosition(hand) + Rotate(WorldRotation(hand), forearmDir_*cfg_.forearm), WorldPosition(hand)};
}

void HandModel::BeginLoss(Vec output, Quat rotation, double t, double lostSince) {
    lossReady_ = armReady_ = false;
    Vec neck; Quat yaw;
    if (!ready_ || lostSince < healthyTime_ || lostSince-healthyTime_ > .1 || !Neck(t,&neck,&yaw)) return;
    shoulderOffset_ = Rotate(yaw,{cfg_.side*cfg_.shoulderWidth*.5,-.10,0});
    handOffset_ = output-(neck+shoulderOffset_);
    const double reach = Length(handOffset_);
    // Refuse a bad fit rather than pulling the hand inward on loss onset.
    if (!Finite(output) || reach < .05 || reach > cfg_.armReach) return;
    neckAtLoss_ = previousNeck_ = neck;
    haveOtherAtLoss_ = Other(t,&otherAtLoss_);
    lossTime_ = std::min(t,lostSince); // the time limit counts from the loss
    lossReady_ = true;
    if (haveForearm_ && cfg_.armFollow > 0 && Unit(rotation)) {
        const Vec elbow = output + Rotate(rotation, forearmDir_*cfg_.forearm);
        elbowOffset_ = elbow-(neck+shoulderOffset_);
        handFromElbow_ = output-elbow;
        rotationAtLoss_ = rotation;
        armTurn_ = {1,0,0,0};
        armReady_ = Finite(elbowOffset_);
    }
}

bool HandModel::Predict(double t, Quat rotation, double rotationTime, Vec* out, Vec* shoulderOut) {
    Vec neck; Quat ignoredYaw;
    const bool turning = armReady_ && rotationTime >= 0 && t >= rotationTime && t-rotationTime <= .1 && Unit(rotation);
    const double limit = turning ? std::max(cfg_.maxPrediction, cfg_.maxPredictionTurning) : cfg_.maxPrediction;
    if (!lossReady_ || t < lossTime_ || t-lossTime_ > limit || !Neck(t,&neck,&ignoredYaw)) return false;
    if (Length(neck-previousNeck_) > .15) { lossReady_ = false; return false; }
    previousNeck_ = neck;
    Vec other;
    // A large head displacement with a stationary visible opposite hand could be
    // head-only leaning. Stop following for this episode; never copy the other arm.
    if (haveOtherAtLoss_ && Other(t,&other) && Length(neck-neckAtLoss_) > .15 &&
        Length(other-otherAtLoss_) < .05) { lossReady_ = false; return false; }
    Vec followed = neckAtLoss_;
    if (cfg_.followBody) {
        const Vec moved = neck-neckAtLoss_;
        const double d = Length(moved);
        if (d > cfg_.followDeadzone) followed = neckAtLoss_ + moved*((d-cfg_.followDeadzone)/d);
    }
    const Vec shoulder = followed+shoulderOffset_;
    if (shoulderOut) *shoulderOut = shoulder;
    if (armReady_) {
        // The share of the controller's turn the forearm makes; kept when rotation stops arriving, so the
        // hand does not jump back to the plain shoulder follow.
        if (turning) armTurn_ = Interpolate({1,0,0,0}, Multiply(rotation, Conjugate(rotationAtLoss_)), cfg_.armFollow);
        *out = shoulder + elbowOffset_ + Rotate(armTurn_, handFromElbow_);
    } else {
        *out = shoulder+handOffset_;
    }
    if (!Finite(*out)) return false;
    debug_ = {t, true, true, armReady_, followed, shoulder, shoulder+elbowOffset_, *out};
    return true;
}
} // namespace ots::filter
