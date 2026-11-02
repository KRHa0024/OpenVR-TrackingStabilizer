#include "ImuOrientation.h"

#include <algorithm>
#include <cmath>

namespace ots::filter {
namespace {
using math::Quat;
using math::Vec;

Vec Cross(Vec a, Vec b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
Vec Normalized(Vec v) {
    const double n = math::Length(v);
    return n > 1e-12 ? v * (1 / n) : v;
}
Vec Mix(Vec a, Vec b, double k) { return a + (b - a) * k; }

// Rotation by the rotation vector v (axis * angle).
Quat Exp(Vec v) {
    const double a = math::Length(v);
    if (a < 1e-12) return {1, v.x / 2, v.y / 2, v.z / 2};
    const double s = std::sin(a / 2) / a;
    return {std::cos(a / 2), v.x * s, v.y * s, v.z * s};
}
Quat Normalized(Quat q) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / n, q.x / n, q.y / n, q.z / n};
}

// Eigenvector of the largest eigenvalue of a symmetric 4x4 matrix (cyclic Jacobi).
void LargestEigen(double a[4][4], double vec[4], double* value) {
    double v[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        double off = 0;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q) off += a[p][q] * a[p][q];
        if (off < 1e-22) break;
        for (int p = 0; p < 4; ++p)
            for (int q = p + 1; q < 4; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (theta >= 0 ? 1 : -1) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
                const double c = 1 / std::sqrt(t * t + 1), s = t * c;
                for (int k = 0; k < 4; ++k) { // columns p, q
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 4; ++k) { // rows p, q
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 4; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
    }
    int best = 0;
    for (int i = 1; i < 4; ++i)
        if (a[i][i] > a[best][best]) best = i;
    for (int k = 0; k < 4; ++k) vec[k] = v[k][best];
    *value = a[best][best];
}

constexpr int kMinPairs = 300;      // moving samples before the mounting is trusted
constexpr double kMaxFitError = .1; // residual energy share allowed for the fit
constexpr double kMinTurn = .5;     // rad/s: slower samples are dominated by noise and bias

} // namespace

void ImuOrientation::Reset() { *this = ImuOrientation(); }

void ImuOrientation::Accumulate(Fit& f, Vec a, Vec b) {
    const double av[3] = {a.x, a.y, a.z}, bv[3] = {b.x, b.y, b.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) f.s[i][j] += av[i] * bv[j];
    f.aa += a.x * a.x + a.y * a.y + a.z * a.z;
    f.bb += b.x * b.x + b.y * b.y + b.z * b.z;
    ++f.n;
}

// Horn's closed form: the rotation r and scale taking gyro readings a to angular velocities b, b = scale * r a.
bool ImuOrientation::Solve(const Fit& f, Quat* r, double* scale, double* error) {
    if (f.n < kMinPairs || f.aa <= 0 || f.bb <= 0) return false;
    const auto& s = f.s;
    double n[4][4] = {
        {s[0][0] + s[1][1] + s[2][2], s[1][2] - s[2][1], s[2][0] - s[0][2], s[0][1] - s[1][0]},
        {s[1][2] - s[2][1], s[0][0] - s[1][1] - s[2][2], s[0][1] + s[1][0], s[2][0] + s[0][2]},
        {s[2][0] - s[0][2], s[0][1] + s[1][0], -s[0][0] + s[1][1] - s[2][2], s[1][2] + s[2][1]},
        {s[0][1] - s[1][0], s[2][0] + s[0][2], s[1][2] + s[2][1], -s[0][0] - s[1][1] + s[2][2]},
    };
    double q[4], lambda;
    LargestEigen(n, q, &lambda);
    if (!(lambda > 0)) return false;
    *r = Normalized(Quat{q[0], q[1], q[2], q[3]});
    *scale = lambda / f.aa;
    *error = std::max(0.0, (f.bb - lambda * lambda / f.aa) / f.bb);
    return std::isfinite(*scale) && std::isfinite(*error);
}

Vec ImuOrientation::ToDevice(Vec gyro) const { return math::Rotate(mount_, gyro - bias_) * scale_; }

void ImuOrientation::Observe(const vr::DriverPose_t& pose, double t) {
    if (!pose.deviceIsConnected) {
        Reset();
        return;
    }
    const int count = gyroCount_;
    const Vec g = count ? gyroSum_ * (1.0 / count) : Vec{}, a = count ? accelSum_ * (1.0 / count) : Vec{};
    gyroSum_ = accelSum_ = {};
    gyroCount_ = 0;
    if (!math::Usable(pose)) return;
    const Quat q = math::WorldRotation(pose);

    if (count > 0 && pose.result == vr::TrackingResult_Running_OK) {
        const Vec w{pose.vecAngularVelocity[0], pose.vecAngularVelocity[1], pose.vecAngularVelocity[2]};
        const Vec device[2] = {math::Rotate(math::Conjugate(q), math::Rotate(pose.qWorldFromDriverRotation, w)), w};
        bool added = false;
        for (int h = 0; h < 2; ++h)
            if (math::Finite(device[h]) && math::Length(device[h]) > kMinTurn && math::Length(device[h]) < 30) {
                Accumulate(fit_[h], g, device[h]);
                added = true;
            }
        if (added && ++pairs_ % 64 == 0) {
            // Refit both readings of the angular velocity, keep the better one.
            for (int h = 0; h < 2; ++h) {
                Quat r;
                double scale, error;
                if (Solve(fit_[h], &r, &scale, &error) && error < kMaxFitError && (!aligned_ || h == hypothesis_ || error < fitError_)) {
                    mount_ = r;
                    scale_ = scale;
                    fitError_ = error;
                    hypothesis_ = h;
                    aligned_ = true;
                }
            }
        }
        if (aligned_) {
            const Vec dev = device[hypothesis_];
            // Bias: what the gyro reads beyond the angular velocity lighthouse reports.
            const Vec expected = math::Rotate(math::Conjugate(mount_), dev) * (1 / scale_);
            if (math::Finite(expected)) {
                bias_ = haveBias_ ? Mix(bias_, g - expected, .002) : g - expected;
                haveBias_ = true;
            }
            // Gravity: the accelerometer's reading in the world while turning slowly.
            const double an = math::Length(a);
            if (math::Length(dev) < .2 && an > 0) {
                const Vec up = Normalized(math::Rotate(q, math::Rotate(mount_, a)));
                if (gNorm_ <= 0) up_ = up, gNorm_ = an;
                else up_ = Normalized(Mix(up_, up, .002)), gNorm_ += (an - gNorm_) * .002;
            }
        }
    }
    q_ = q;
    anchored_ = true;
    anchorT_ = t;
}

void ImuOrientation::Add(const ImuSample& s, double t) {
    if (!math::Finite(s.gyro) || !math::Finite(s.accel)) return;
    ++samples_;
    if (rateT0_ < 0) rateT0_ = t, rateN0_ = samples_;
    else if (t - rateT0_ >= 1) {
        rate_ = double(samples_ - rateN0_) / (t - rateT0_);
        rateT0_ = t;
        rateN0_ = samples_;
    }
    gyroSum_ = gyroSum_ + s.gyro;
    accelSum_ = accelSum_ + s.accel;
    ++gyroCount_;

    double dt = haveSampleClock_ ? s.sampleTime - lastSampleClock_ : 0;
    if (!(dt > 0 && dt < .02)) dt = lastSampleT_ >= 0 ? std::clamp(t - lastSampleT_, 0.0, .02) : 0; // device clock unusable
    lastSampleClock_ = s.sampleTime;
    haveSampleClock_ = true;
    lastSampleT_ = t;
    if (!aligned_ || !anchored_ || dt <= 0) return;

    const Vec w = ToDevice(s.gyro);
    omega_ = w;
    q_ = Normalized(math::Multiply(q_, Exp(w * dt))); // device-axes turn
    // Tilt: pull the accelerometer's world direction towards gravity while it reads about 1 g and the
    // device is not spinning (then it is mostly gravity, not motion). Over the first live occlusions the
    // tilt was off by 2-6 degrees after 1-7 s at 1/s, hence 2/s.
    if (gNorm_ > 0 && std::fabs(math::Length(s.accel) / gNorm_ - 1) < .1 && math::Length(w) < 1.5) {
        const Vec aw = Normalized(math::Rotate(q_, math::Rotate(mount_, s.accel)));
        q_ = Normalized(math::Multiply(Exp(Cross(aw, up_) * (2.0 * dt)), q_)); // world-axes turn, 2/s
    }
}

bool ImuOrientation::Rotation(double t, Quat* world) const {
    if (!aligned_ || !anchored_ || lastSampleT_ < 0 || t - lastSampleT_ > .1 || t - anchorT_ > kMaxFree) return false;
    // Samples arrive in bursts: carry on at the last rate up to `t` (at most 30 ms), so a fast turn is
    // output evenly rather than in uneven steps.
    *world = Normalized(math::Multiply(q_, Exp(omega_ * std::clamp(t - lastSampleT_, 0.0, .03))));
    return math::Unit(*world);
}

} // namespace ots::filter
