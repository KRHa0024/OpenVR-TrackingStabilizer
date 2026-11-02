#pragma once
// A device's rotation from its raw IMU (gyro + accelerometer, read by the driver from lighthouse's
// /devices/lighthouse/<serial>/imu IOBuffer), for when lighthouse itself stops reporting it: ~2 s into
// an optical loss lighthouse resets tracking and sends nothing until optics return.
//
// Everything about the IMU is learned from lighthouse's own poses while the device is tracked, so no
// calibration file or axis convention is assumed:
// - mounting: the rotation (and scale) taking gyro readings to the device's angular velocity, fitted
//   over moving samples. Both readings of the pose's vecAngularVelocity (world or device axes) are
//   fitted and the better one kept;
// - gyro bias, and the direction of gravity as the accelerometer sees it in the world.
// Every usable pose re-anchors the rotation; between poses (i.e. while lighthouse is silent) the gyro
// is integrated and the tilt corrected towards gravity. Yaw drifts slowly (no reference for it).
#include "PoseMath.h"

#include <cstdint>

namespace ots::filter {

struct ImuSample {
    double sampleTime = 0; // device clock, seconds (only differences are used)
    math::Vec gyro, accel; // raw IMU axes and units
};

class ImuOrientation {
public:
    void Reset();
    // A pose from lighthouse, `t` its arrival time. Usable poses re-anchor; good (optical) ones while
    // moving also teach the mounting, and slow ones the bias and gravity.
    void Observe(const vr::DriverPose_t& pose, double t);
    // Samples in arrival order; `t` their arrival time.
    void Add(const ImuSample& s, double t);
    // World rotation now, integrated from the last pose; false until the mounting is learned, without
    // recent samples, or after kMaxFree of integration without any pose.
    bool Rotation(double t, math::Quat* world) const;

    bool Aligned() const { return aligned_; }
    double FitError() const { return fitError_; } // relative residual of the mounting fit (0 = perfect)
    int AxesHypothesis() const { return hypothesis_; } // 0 = angular velocity in world axes, 1 = device axes
    uint64_t Samples() const { return samples_; }
    double SampleRate() const { return rate_; }

    static constexpr double kMaxFree = 120; // seconds of gyro-only rotation before giving up

private:
    struct Fit {
        double s[3][3] = {}; // sum of gyro * omega^T
        double aa = 0, bb = 0;
        int n = 0;
    };
    static void Accumulate(Fit& f, math::Vec a, math::Vec b);
    static bool Solve(const Fit& f, math::Quat* r, double* scale, double* error);
    math::Vec ToDevice(math::Vec gyro) const; // bias-corrected, mounted and scaled: rad/s in device axes

    // mounting
    Fit fit_[2];
    uint64_t pairs_ = 0;
    bool aligned_ = false;
    int hypothesis_ = 0;
    math::Quat mount_{1, 0, 0, 0};
    double scale_ = 1, fitError_ = 1;
    math::Vec bias_; // raw gyro units
    bool haveBias_ = false;
    math::Vec up_{0, 1, 0}; // world direction of the accelerometer's reading at rest
    double gNorm_ = 0;      // its magnitude, raw units

    // gyro samples since the last pose (paired with that pose's angular velocity)
    math::Vec gyroSum_, accelSum_;
    int gyroCount_ = 0;

    // integration
    bool anchored_ = false;
    math::Quat q_{1, 0, 0, 0};
    math::Vec omega_; // last angular velocity, device axes, rad/s
    double anchorT_ = -1, lastSampleT_ = -1, lastSampleClock_ = 0;
    bool haveSampleClock_ = false;
    uint64_t samples_ = 0;
    double rate_ = 0, rateT0_ = -1;
    uint64_t rateN0_ = 0;
};

} // namespace ots::filter
