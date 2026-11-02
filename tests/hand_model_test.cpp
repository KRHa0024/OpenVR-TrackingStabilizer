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
Quat Yaw(double angle) { return {std::cos(angle/2),0,std::sin(angle/2),0}; }
// Plain shoulder follow (no arm model, no dead zone); ArmModelSwingsHand covers the arm model.
Config Settings(int side = -1) {
    Config c; c.hand.enabled = true; c.hand.side = side; c.hand.armFollow = 0; c.hand.followDeadzone = 0; return c;
}
vr::DriverPose_t Head(double x = 1) { return Pose({x,1.62,-.08}); }
vr::DriverPose_t Hand(int side = -1, double x = 1) { return Pose({x+side*.35,1.1,-.3}); }
vr::DriverPose_t Lost(int side = -1) {
    auto p = Hand(side); p.result = vr::TrackingResult_Calibrating_OutOfRange; p.vecPosition[0] = 99; return p;
}
void References(TrackingFilter& f, double t, double x = 1, bool otherVisible = true) {
    const int side = f.GetConfig().hand.side;
    auto other = Hand(-side,x); if (!otherVisible) other.poseIsValid = false;
    f.SetHandReference(Head(x),t,other,t);
}
void Train(TrackingFilter& f, double start = 0) {
    for (int i = 0; i <= 100; ++i) {
        const double t = start+i*.004; References(f,t);
        const auto p = Hand(f.GetConfig().hand.side); const auto out = f.Process(p,t);
        CHECK(std::memcmp(&p,&out,sizeof(p)) == 0);
    }
    CHECK(f.HandReady() == f.GetConfig().hand.enabled); CHECK(!f.HandActive()); CHECK(!f.BodyReady());
}
void FollowAndBothHands() {
    for (const int side : {-1,1}) {
        TrackingFilter f(Settings(side)); Train(f);
        References(f,.404); auto out = f.Process(Lost(side),.404);
        CHECK(f.HandActive()); CHECK(Length(WorldPosition(out)-WorldPosition(Hand(side))) < 1e-9);
        for (int i = 1; i <= 100; ++i) {
            const double t = .404+i*.004, x = 1+i*.001;
            References(f,t,x,false); // both hands can be lost: no corrected opposite-hand feedback
            auto lost = Lost(side); lost.qRotation = Yaw(i*.002);
            out = f.Process(lost,t);
            CHECK(f.HandActive()); CHECK(Length(WorldPosition(out)-WorldPosition(Hand(side,x))) < 1e-9);
            CHECK(Angle(WorldRotation(out),lost.qRotation) < 1e-7);
        }
    }
}
void TurnsAndOtherHand() {
    TrackingFilter f(Settings()); Train(f); References(f,.404); f.Process(Lost(),.404);
    // A head turning about the neck does not move the neck or swing the hidden hand.
    auto head = Head(); head.qRotation = Yaw(.8);
    const Vec pos = Vec{1,1.5,0}-Rotate(head.qRotation,{0,-.12,.08});
    head.vecPosition[0] = pos.x; head.vecPosition[1] = pos.y; head.vecPosition[2] = pos.z;
    auto other = Hand(1); other.vecPosition[0] += .5;
    f.SetHandReference(head,.408,other,.408);
    const auto out = f.Process(Lost(),.408);
    CHECK(f.HandActive()); CHECK(Length(WorldPosition(out)-WorldPosition(Hand())) < 1e-9);

    TrackingFilter conflict(Settings()); Train(conflict); References(conflict,.404); conflict.Process(Lost(),.404);
    vr::DriverPose_t held{};
    for (int i = 1; i < 50; ++i) {
        const double t = .404+i*.004;
        conflict.SetHandReference(Head(1+i*.004),t,Hand(1),t);
        held = conflict.Process(Lost(),t);
    }
    CHECK(!conflict.HandActive()); const Vec p = WorldPosition(held);
    References(conflict,.61,1.2); held = conflict.Process(Lost(),.61);
    CHECK(Length(WorldPosition(held)-p) < 1e-9); // conflict latches until the next loss episode
}
void StaleHeadJumpsAndTimeout() {
    TrackingFilter f(Settings()); Train(f); References(f,.404); f.Process(Lost(),.404);
    auto out = f.Process(Lost(),.51); CHECK(!f.HandActive());
    CHECK(Length(WorldPosition(out)-WorldPosition(Hand())) < 1e-9);
    References(f,.514,2); out = f.Process(Lost(),.514);
    CHECK(!f.HandActive()); CHECK(Length(WorldPosition(out)-WorldPosition(Hand())) < 1e-9);

    Config oneSecond = Settings(); oneSecond.hand.maxPrediction = 1.0;
    TrackingFilter deadline(oneSecond); Train(deadline); References(deadline,.404); deadline.Process(Lost(),.404);
    for (int i = 1; i <= 275; ++i) {
        const double t = .404+i*.004;
        References(deadline,t,1+i*.0005); out = deadline.Process(Lost(),t);
    }
    CHECK(!deadline.HandActive()); const Vec p = WorldPosition(out);
    References(deadline,1.508,1.2); out = deadline.Process(Lost(),1.508);
    CHECK(Length(WorldPosition(out)-p) < 1e-9);
}
void StoppedInputAndSwitches() {
    TrackingFilter f(Settings()); Train(f);
    References(f,.432,1.001); vr::DriverPose_t out{}; CHECK(!f.Emit(.432,&out)); // a 32 ms stall is not a loss
    References(f,.532,1.001); CHECK(f.Emit(.532,&out));
    CHECK(f.GetState() == State::Hold); CHECK(f.HandActive());
    for (int i = 1; i <= 50; ++i) { const double t = .532+i*.004; References(f,t,1.001+i*.001); CHECK(f.Emit(t,&out)); }
    CHECK(WorldPosition(out).x > .69); // translation can continue with frozen rotation, no raw IMU required
    CHECK(Angle(WorldRotation(out),{1,0,0,0}) < 1e-9);
    const Vec p = WorldPosition(out);
    auto cfg = f.GetConfig(); cfg.hand.enabled = false; f.SetConfig(cfg);
    References(f,.736,1.5); out = f.Process(Lost(),.736);
    CHECK(!f.HandActive()); CHECK(Length(WorldPosition(out)-p) < 1e-9);
    cfg.hand.enabled = true; f.SetConfig(cfg); References(f,.74); f.Process(Lost(),.74);
    CHECK(!f.HandActive()); CHECK(!f.HandReady());
    Config disabled = Settings(); disabled.hand.enabled = false;
    TrackingFilter midway(disabled); Train(midway); // same healthy passthrough, without preparing the hand model
    auto raw = Lost(); raw.qRotation = Yaw(.8);
    midway.Process(raw,.404);
    disabled.hand.enabled = true; midway.SetConfig(disabled);
    References(midway,.408); out = midway.Process(raw,.408);
    CHECK(Angle(WorldRotation(out),Yaw(.8)) < 1e-7);
    CHECK(!midway.HandActive());
}
void ReconnectDimensionsAndInvalid() {
    TrackingFilter f(Settings()); Train(f); References(f,.404);
    auto head = Head(); head.deviceIsConnected = false;
    f.SetHandReference(head,.408,Hand(1),.408); f.Process(Lost(),.408);
    CHECK(!f.HandActive()); CHECK(!f.HandReady());
    References(f,.412); f.Process(Lost(),.412); CHECK(!f.HandActive());
    auto off = Lost(); off.deviceIsConnected = false; f.Process(off,.416);
    References(f,.42); auto out = f.Process(Lost(),.42);
    CHECK(out.result != vr::TrackingResult_Running_OK); CHECK(!f.Emit(.5,&out));
    Train(f,1);
    auto cfg = f.GetConfig(); cfg.hand.armReach = .3; f.SetConfig(cfg);
    for (int i = 0; i < 100; ++i) { const double t = 1.404+i*.004; References(f,t); f.Process(Hand(),t); }
    CHECK(!f.HandReady()); f.Process(Lost(),1.804); CHECK(!f.HandActive());

    TrackingFilter rotation(Settings()); Train(rotation); References(rotation,.404); rotation.Process(Lost(),.404);
    auto bad = Lost(); bad.qRotation.w = std::numeric_limits<double>::quiet_NaN();
    References(rotation,.408); out = rotation.Process(bad,.408); CHECK(Unit(WorldRotation(out)));
}
void RecoveryAndWorldFrames() {
    TrackingFilter f(Settings()); Train(f); References(f,.404); f.Process(Lost(),.404);
    auto live = Hand(); live.qRotation = Yaw(1);
    auto out = f.Process(live,.408); CHECK(Angle(WorldRotation(out),{1,0,0,0}) < 1e-9);
    Quat prev = WorldRotation(out); double maximum = 0;
    for (int i = 1; i < 110; ++i) {
        if (i == 25) {
            const Vec world = WorldPosition(live); const Quat q = WorldRotation(live);
            live.qWorldFromDriverRotation = Yaw(.7); live.vecWorldFromDriverTranslation[0] = 2;
            const Vec local = Rotate(Conjugate(live.qWorldFromDriverRotation),world-Vec{2,0,0});
            live.vecPosition[0] = local.x; live.vecPosition[1] = local.y; live.vecPosition[2] = local.z;
            live.qRotation = Multiply(Conjugate(live.qWorldFromDriverRotation),q);
        }
        out = f.Process(live,.408+i*.004);
        maximum = std::max(maximum,Angle(prev,WorldRotation(out))); prev = WorldRotation(out);
    }
    CHECK(maximum < .04); CHECK(Angle(WorldRotation(out),Yaw(1)) < 1e-7);
    CHECK(Length(WorldPosition(out)-WorldPosition(Hand())) < 1e-8);
}
void ReachAndContinuity() {
    TrackingFilter f(Settings());
    auto fullReach = Hand(); fullReach.vecPosition[0] = .121; fullReach.vecPosition[1] = 1.4; fullReach.vecPosition[2] = 0;
    for (int i = 0; i <= 100; ++i) { const double t = i*.004; References(f,t); f.Process(fullReach,t); }
    CHECK(f.HandReady()); References(f,.404); f.Process(Lost(),.404);
    // Rapid shoulder translation: no reachable output exists within the allowed step.
    References(f,.408,1.1,false); const auto out = f.Process(Lost(),.408);
    CHECK(!f.HandActive()); CHECK(Length(WorldPosition(out)-WorldPosition(fullReach)) < 1e-9);
}
// Arm model: with the head still, turning the hidden controller swings it about an elbow `forearm`
// from it, towards where the shoulder was in the controller's axes; the turn is kept when rotation
// stops arriving, and the estimate outlasts the plain shoulder follow's time limit.
void ArmModelSwingsHand() {
    Config c = Settings(); c.hand.armFollow = 1; c.hand.maxPrediction = 1;
    TrackingFilter f(c); Train(f); References(f,.404); f.Process(Lost(),.404);
    const Vec shoulder{1-.18,1.40,0}, hand0 = WorldPosition(Hand());
    const Vec toShoulder = (shoulder-hand0)*(1/Length(shoulder-hand0));
    const Vec elbow = hand0+toShoulder*c.hand.forearm;
    vr::DriverPose_t out{};
    double t = .404;
    for (int i = 1; i <= 500; ++i) { // 2 s, turning to 0.5 rad over the first second
        t = .404+i*.004; References(f,t,1,false);
        auto lost = Lost(); lost.qRotation = Yaw(std::min(i,250)*.002); out = f.Process(lost,t);
    }
    CHECK(f.HandActive());
    const Vec expected = elbow+Rotate(Yaw(.5),hand0-elbow);
    CHECK(Length(WorldPosition(out)-expected) < 1e-3);
    CHECK(Length(WorldPosition(out)-hand0) > .05); // it did move
    // Rotation stops arriving (lighthouse silent, no IMU): the turn is kept, no jump back.
    const Vec before = WorldPosition(out);
    for (int i = 6; i <= 55; ++i) { References(f,t+i*.004,1,false); CHECK(f.Emit(t+i*.004,&out)); } // after the 20 ms pause
    CHECK(Length(WorldPosition(out)-before) < 1e-6);
}
}
int main() {
    FollowAndBothHands(); TurnsAndOtherHand(); StaleHeadJumpsAndTimeout();
    StoppedInputAndSwitches(); ReconnectDimensionsAndInvalid(); RecoveryAndWorldFrames(); ReachAndContinuity();
    ArmModelSwingsHand();
    std::printf("hand model tests: %s (%d failures)\n",failures ? "FAILED" : "passed",failures);
    return failures ? 1 : 0;
}
