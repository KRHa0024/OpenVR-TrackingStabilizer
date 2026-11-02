#include "TrackingFilter.h"
#include <cstdio>
#include <cstring>
#include <limits>

using namespace ots::filter;
using namespace ots::filter::math;
namespace {
int failures = 0;
#define CHECK(x) do { if (!(x)) { ++failures; std::printf("FAIL line %d: %s\n",__LINE__,#x); } } while (0)
vr::DriverPose_t Pose(Vec v, Quat q = {1,0,0,0}) {
    vr::DriverPose_t p{};
    p.deviceIsConnected = p.poseIsValid = true; p.result = vr::TrackingResult_Running_OK;
    p.qRotation = q; p.qWorldFromDriverRotation = p.qDriverFromHeadRotation = {1,0,0,0};
    p.vecPosition[0] = v.x; p.vecPosition[1] = v.y; p.vecPosition[2] = v.z;
    return p;
}
Quat Tilt(double angle) { return {std::cos(angle/2),0,0,std::sin(angle/2)}; }
Config Settings() { Config c; c.body.enabled = true; return c; }
vr::DriverPose_t Head(double x = 1) { return Pose({x,1.62,-.08}); } // neck = (x,1.5,0)
vr::DriverPose_t Waist(double x = 1) { return Pose({x,1,0}); }
vr::DriverPose_t Lost(double x = 1, Quat q = {1,0,0,0}) {
    auto p = Pose({x,1,0},q); p.result = vr::TrackingResult_Calibrating_OutOfRange; return p;
}
void Train(TrackingFilter& f, double start = 0) {
    for (int i = 0; i <= 200; ++i) {
        const double t = start+i*.004; f.SetBodyReference(Head(),t);
        const auto p = Waist(); const auto out = f.Process(p,t);
        CHECK(std::memcmp(&p,&out,sizeof(p)) == 0);
    }
    CHECK(f.BodyReady()); CHECK(std::abs(f.TorsoLength()-.5) < 1e-9);
}
void KinematicsAndFallback() {
    TrackingFilter f(Settings()); Train(f);
    auto out = f.Process(Lost(),.804); CHECK(f.BodyActive());
    for (int i = 2; i < 102; ++i) {
        const double t = .8+i*.004, x = 1+(t-.804)*.2, angle = (t-.804)*.5;
        f.SetBodyReference(Head(x),t);
        out = f.Process(Lost(99,Tilt(angle)),t); // raw position drift must be ignored
        const Vec truth = Vec{x,1.5,0}-Rotate(Tilt(angle),{0,.5,0});
        CHECK(Length(WorldPosition(out)-truth) < 1e-8);
        CHECK(Angle(WorldRotation(out),Tilt(angle)) < 1e-7);
    }
    CHECK(WorldPosition(out).x > 1.15); // differs from simple HMD translation
    const Vec held = WorldPosition(out);
    auto badHead = Head(2); badHead.result = vr::TrackingResult_Running_OutOfRange;
    f.SetBodyReference(badHead,1.21); out = f.Process(Lost(),1.212);
    CHECK(!f.BodyActive()); CHECK(Length(WorldPosition(out)-held) < 1e-9);
    f.SetBodyReference(Head(3),1.22); out = f.Process(Lost(),1.224);
    CHECK(!f.BodyActive()); CHECK(Length(WorldPosition(out)-held) < 1e-9); // reference jump rejected
}
void StaleStreamsAndDisconnects() {
    TrackingFilter stopped(Settings()); Train(stopped);
    vr::DriverPose_t emitted{};
    CHECK(!stopped.Emit(.832,&emitted)); // a 32 ms stall is not a loss
    CHECK(stopped.Emit(.932,&emitted)); CHECK(stopped.GetState() == State::Hold);
    CHECK(Unit(WorldRotation(emitted)));
    TrackingFilter f(Settings()); Train(f);
    f.Process(Lost(),.804);
    f.SetBodyReference(Head(1.01),.83);
    vr::DriverPose_t out{}; CHECK(f.Emit(.832,&out)); CHECK(f.BodyActive());
    f.SetBodyReference(Head(1.1),.95); CHECK(f.Emit(.952,&out)); CHECK(!f.BodyActive());
    const Vec held = WorldPosition(out);
    f.SetBodyReference(Head(1.2),1.0); CHECK(f.Emit(1.004,&out));
    CHECK(Length(WorldPosition(out)-held) < 1e-9); // waist orientation expired
    auto disconnect = Lost(); disconnect.deviceIsConnected = false;
    f.Process(disconnect,1.01); CHECK(!f.Emit(1.1,&out)); CHECK(!f.BodyReady());
    // A reconnect starting invalid must not reuse the pre-disconnect trusted position.
    const auto invalid = Lost(5); out = f.Process(invalid,1.2);
    CHECK(f.GetState() == State::Tracking); CHECK(out.result == invalid.result);
    Train(f,2);
    auto head = Head(); head.deviceIsConnected = false; f.SetBodyReference(head,2.804);
    CHECK(!f.BodyReady()); f.SetBodyReference(Head(),2.808);
    out = f.Process(Lost(),2.808); CHECK(!f.BodyActive());
}
void StaleReferenceAndToggle() {
    TrackingFilter f(Settings()); Train(f); f.Process(Lost(),.804);
    auto out = f.Process(Lost(9),1.0); CHECK(!f.BodyActive());
    CHECK(std::abs(WorldPosition(out).x-1) < 1e-9);
    Config c = Settings(); c.body.enabled = false; f.SetConfig(c); CHECK(!f.BodyReady());
    f.SetBodyReference(Head(2),1.004); out = f.Process(Lost(9),1.004);
    CHECK(!f.BodyActive()); CHECK(std::abs(WorldPosition(out).x-1) < 1e-9);
    c.body.enabled = true; f.SetConfig(c); out = f.Process(Lost(9),1.008);
    CHECK(!f.BodyReady()); CHECK(!f.BodyActive());
}
void RecoveryRotationAndFrames() {
    TrackingFilter f(Settings()); Train(f); f.Process(Lost(),.804);
    const Quat recovered = Tilt(1.2);
    auto live = Waist(1.1); live.qRotation = recovered;
    auto out = f.Process(live,.808); CHECK(Angle(WorldRotation(out),{1,0,0,0}) < 1e-9);
    for (int i = 1; i < 25; ++i) {
        out = f.Process(live,.808+i*.004); CHECK(Angle(WorldRotation(out),{1,0,0,0}) < 1e-9);
    }
    // Same world pose in a different lighthouse driver frame.
    live.qWorldFromDriverRotation = Tilt(.7); live.qRotation = Multiply(Conjugate(live.qWorldFromDriverRotation),recovered);
    live.vecWorldFromDriverTranslation[0] = 3;
    const Vec local = Rotate(Conjugate(live.qWorldFromDriverRotation),Vec{1.1,1,0}-Vec{3,0,0});
    live.vecPosition[0] = local.x; live.vecPosition[1] = local.y;
    Quat prev = WorldRotation(out); double maxAngle = 0;
    for (int i = 25; i < 110; ++i) {
        out = f.Process(live,.808+i*.004);
        maxAngle = std::max(maxAngle,Angle(prev,WorldRotation(out))); prev = WorldRotation(out);
    }
    CHECK(maxAngle < .04); CHECK(Angle(WorldRotation(out),recovered) < 1e-7);
    CHECK(Length(WorldPosition(out)-Vec{1.1,1,0}) < 1e-8);
}
void RejectBadRotationAndUnstableRecovery() {
    TrackingFilter f(Settings()); Train(f); f.Process(Lost(),.804);
    auto out = f.Process(Lost(99,Tilt(2)),.808);
    CHECK(Angle(WorldRotation(out),{1,0,0,0}) < 1e-9);
    auto bad = Lost(); bad.qRotation.w = std::numeric_limits<double>::quiet_NaN();
    out = f.Process(bad,.812); CHECK(Unit(WorldRotation(out)));
    for (int i = 0; i < 200; ++i) {
        const auto p = Pose({1,1,0},Tilt(i%2 ? 0 : 1)); out = f.Process(p,.816+i*.004);
        CHECK(f.GetState() != State::Blend); CHECK(Unit(WorldRotation(out)));
    }
}
void LearningConfidenceAndHeadPivot() {
    TrackingFilter f(Settings());
    for (int i = 0; i < 300; ++i) {
        const double t = i*.004; f.SetBodyReference(Head(),t);
        f.Process(Waist(i%2 ? .7 : 1.3),t);
    }
    CHECK(!f.BodyReady()); f.Process(Lost(),1.2); CHECK(!f.BodyActive());
    TrackingFilter g(Settings());
    for (int i = 0; i < 300; ++i) {
        const double t = i*.004; auto h = Head(); h.vecPosition[1] = 3;
        g.SetBodyReference(h,t); g.Process(Waist(),t);
    }
    CHECK(!g.BodyReady());
    TrackingFilter h(Settings()); Train(h); h.Process(Lost(),.804);
    auto head = Head(); head.qDriverFromHeadRotation = Tilt(.4);
    // Change the HMD device origin / rotation offset without moving the physical head.
    head.vecDriverFromHeadTranslation[0] = .02;
    const Vec desiredNeck{1,1.5,0};
    const Vec p = desiredNeck-Rotate(head.qDriverFromHeadRotation,{0,-.12,.08})-Vec{.02,0,0};
    head.vecPosition[0] = p.x; head.vecPosition[1] = p.y; head.vecPosition[2] = p.z;
    h.SetBodyReference(head,.808); const auto out = h.Process(Lost(),.808);
    CHECK(Length(WorldPosition(out)-Vec{1,1,0}) < 1e-8);
    TrackingFilter residual(Settings()); Train(residual);
    // A small calibration residual must not move an otherwise stationary waist on loss.
    auto shiftedHead = Head(1.04); residual.SetBodyReference(shiftedHead,.804);
    auto anchored = residual.Process(Lost(),.804);
    residual.SetBodyReference(shiftedHead,.808); anchored = residual.Process(Lost(),.808);
    CHECK(Length(WorldPosition(anchored)-Vec{1,1,0}) < 1e-8);
    TrackingFilter poor(Settings()); Train(poor); poor.SetBodyReference(Head(1.2),.804);
    poor.Process(Lost(),.804); CHECK(!poor.BodyActive());
}
}
int main() {
    KinematicsAndFallback(); StaleStreamsAndDisconnects(); StaleReferenceAndToggle();
    RecoveryRotationAndFrames(); RejectBadRotationAndUnstableRecovery(); LearningConfidenceAndHeadPivot();
    std::printf("body model tests: %s (%d failures)\n",failures ? "FAILED" : "passed",failures);
    return failures ? 1 : 0;
}
