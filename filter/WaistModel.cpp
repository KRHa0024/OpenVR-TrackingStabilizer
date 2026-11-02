#include "WaistModel.h"

namespace ots::filter {
using namespace math;
void WaistModel::Configure(const BodyConfig& c) {
    if (c.enabled != cfg_.enabled || c.neckDown != cfg_.neckDown || c.neckBack != cfg_.neckBack) Reset();
    cfg_ = c;
}
void WaistModel::Reset() {
    ready_ = false; samples_ = 0; stableSince_ = sampleT_ = headT_ = -1; torso_ = {};
}
void WaistModel::Reference(const vr::DriverPose_t& p, double t) {
    // Never reuse a calibration across an HMD disconnect / reconnect.
    if (!p.deviceIsConnected) Reset();
    head_ = p; headT_ = t;
}
bool WaistModel::Neck(double t, Vec* out) const {
    if (!cfg_.enabled || !Good(head_) || !Unit(head_.qDriverFromHeadRotation) || t < headT_ || t-headT_ > .1) return false;
    const auto deviceRotation = WorldRotation(head_);
    const Vec headPosition = WorldPosition(head_) + Rotate(deviceRotation,
        {head_.vecDriverFromHeadTranslation[0],head_.vecDriverFromHeadTranslation[1],head_.vecDriverFromHeadTranslation[2]});
    *out = headPosition + Rotate(Multiply(deviceRotation,head_.qDriverFromHeadRotation), {0,-cfg_.neckDown,cfg_.neckBack});
    return Finite(*out);
}
void WaistModel::Observe(const vr::DriverPose_t& p, double t) {
    Vec neck;
    if (!Good(p) || !Neck(t,&neck)) { stableSince_ = -1; samples_ = 0; return; }
    debug_ = {t, false, false, false, neck, {}, {}, WorldPosition(p)};
    const Vec local = Rotate(Conjugate(WorldRotation(p)), neck-WorldPosition(p));
    const double len = Length(local);
    // Conservative anthropometric envelope; no assumed floor or tracker role.
    if (len < .25 || len > .85 || !Finite(local)) { ready_ = false; stableSince_ = -1; return; }
    const bool changed = Length(local-torso_) > .08;
    if (ready_ && !changed) return;
    ready_ = false;
    if (stableSince_ < 0 || changed || t-sampleT_ > .1 || t <= sampleT_) {
        torso_ = local; stableSince_ = t; samples_ = 1;
    } else {
        torso_ = torso_ + (local-torso_)*(1.0/double(++samples_));
    }
    sampleT_ = t;
    ready_ = samples_ >= 20 && t-stableSince_ >= .5;
}
bool WaistModel::Predict(double t, Quat q, double rotationTime, Vec* out) const {
    Vec neck;
    if (!ready_ || !Unit(q) || t < rotationTime || t-rotationTime > .1 || !Neck(t,&neck)) return false;
    *out = neck-Rotate(q,torso_);
    if (!Finite(*out)) return false;
    debug_ = {t, true, false, false, neck, {}, {}, *out};
    return true;
}
} // namespace ots::filter
