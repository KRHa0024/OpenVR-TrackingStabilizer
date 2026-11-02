// Unit tests for ots::filter::ImuOrientation on a synthetic device: a known IMU mounting, gyro scale and
// bias, lighthouse poses while tracked, then poses stop and only IMU samples arrive.
#include "ImuOrientation.h"

#include <cmath>
#include <cstdio>

using namespace ots::filter;
using namespace ots::filter::math;

namespace {
int g_failures = 0;
#define CHECK(cond, ...)                                                  \
    do {                                                                  \
        if (!(cond)) {                                                    \
            ++g_failures;                                                 \
            std::printf("  FAIL line %d: %s  ", __LINE__, #cond);         \
            std::printf(__VA_ARGS__);                                     \
            std::printf("\n");                                            \
        }                                                                 \
    } while (0)

Quat ExpV(Vec v) {
    const double a = Length(v);
    if (a < 1e-12) return {1, 0, 0, 0};
    const double s = std::sin(a / 2) / a;
    return {std::cos(a / 2), v.x * s, v.y * s, v.z * s};
}
Quat Norm(Quat q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

struct Device {
    Quat mount = Norm({.3, -.5, .7, .2}); // device axes from IMU axes
    double scale = 1 / 57.2958;           // the IMU reports deg/s
    Vec bias{.8, -.4, .3};                // deg/s
    bool deviceAxes = false;              // pose angular velocity in device axes instead of world
    Quat q{1, 0, 0, 0};                   // world rotation
    double t = 0;

    Vec WorldOmega() const { // a wandering turn, 1-3 rad/s
        return {2 * std::sin(1.3 * t), 1.5 * std::cos(.7 * t + 1), 2.5 * std::sin(.9 * t + 2)};
    }
    void Step(double dt) {
        q = Norm(Multiply(ExpV(WorldOmega() * dt), q));
        t += dt;
    }
    ImuSample Sample() const {
        const Vec dev = Rotate(Conjugate(q), WorldOmega());
        ImuSample s;
        s.sampleTime = 1000 + t;
        s.gyro = Rotate(Conjugate(mount), dev) * (1 / scale) + bias;
        s.accel = Rotate(Conjugate(mount), Rotate(Conjugate(q), Vec{0, 9.81, 0}));
        return s;
    }
    vr::DriverPose_t Pose() const {
        vr::DriverPose_t p{};
        p.qWorldFromDriverRotation = {1, 0, 0, 0};
        p.qDriverFromHeadRotation = {1, 0, 0, 0};
        p.qRotation = q;
        p.vecPosition[0] = .5;
        p.vecPosition[1] = 1;
        const Vec w = deviceAxes ? Rotate(Conjugate(q), WorldOmega()) : WorldOmega();
        p.vecAngularVelocity[0] = w.x, p.vecAngularVelocity[1] = w.y, p.vecAngularVelocity[2] = w.z;
        p.result = vr::TrackingResult_Running_OK;
        p.poseIsValid = p.deviceIsConnected = true;
        return p;
    }
};

void Run(bool deviceAxes) {
    std::printf("learns the mounting (angular velocity in %s axes) and keeps the rotation without poses\n",
                deviceAxes ? "device" : "world");
    Device d;
    d.deviceAxes = deviceAxes;
    ImuOrientation imu;
    const double dt = .001;
    for (int i = 0; i < 6000; ++i) { // 6 s tracked
        d.Step(dt);
        imu.Add(d.Sample(), d.t);
        imu.Observe(d.Pose(), d.t);
    }
    CHECK(imu.Aligned(), "not aligned after 6 s of motion (fit error %.3f)", imu.FitError());
    CHECK(imu.AxesHypothesis() == (deviceAxes ? 1 : 0), "picked hypothesis %d", imu.AxesHypothesis());
    double worst = 0;
    Quat r;
    for (int i = 0; i < 3000; ++i) { // 3 s with IMU samples only
        d.Step(dt);
        imu.Add(d.Sample(), d.t);
        if (imu.Rotation(d.t, &r)) worst = std::fmax(worst, Angle(r, d.q));
        else worst = 1e9;
    }
    CHECK(worst < .035, "rotation off by %.2f deg after 3 s without poses", worst * 57.2958);
    CHECK(!imu.Rotation(d.t + .2, &r), "still reports a rotation without samples");
}

void NotAlignedWithoutMotion() {
    std::printf("no rotation before the mounting is known\n");
    Device d;
    ImuOrientation imu;
    Quat r;
    for (int i = 0; i < 100; ++i) {
        d.Step(.001);
        imu.Add(d.Sample(), d.t);
        imu.Observe(d.Pose(), d.t);
    }
    CHECK(!imu.Aligned(), "aligned from 0.1 s");
    CHECK(!imu.Rotation(d.t, &r), "rotation reported before alignment");
}
} // namespace

int main() {
    Run(false);
    Run(true);
    NotAlignedWithoutMotion();
    std::printf(g_failures ? "imu orientation tests: %d FAILURE(S)\n" : "imu orientation tests passed\n", g_failures);
    return g_failures ? 1 : 0;
}
