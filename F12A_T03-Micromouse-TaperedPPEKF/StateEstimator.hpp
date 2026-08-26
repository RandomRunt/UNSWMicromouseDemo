#pragma once

#include <Arduino.h>
#include <math.h>
#include "Encoder.hpp"
#include "IMU.hpp"
#include "LIDAR.hpp"
#include "MapRay.hpp"
#include "Model.hpp"
#include "tinyekf.h"

class StateEstimator {
public:
    static constexpr float kMaxWheelStep = 0.05f;
    static constexpr float kRangeGateM = 0.06f;
    static constexpr float kDefaultRangeNoiseVar = 4e-4f;  // (0.02 m)^2
    static constexpr float kMaxLidarStepM = 0.012f;
    static constexpr float kFarRangeM = 0.16f;
    static constexpr float kFarNoiseScale = 4.0f;
    // Body-face range must sit in VL6180X band (LIDAR.hpp 20–200 mm).
    static constexpr float kLidarMinBodyM = 0.020f;
    static constexpr float kLidarMaxBodyM = 0.200f;

    StateEstimator(Model& model, Encoder& encL, Encoder& encR, float wheelRadius, IMU* imu = nullptr)
        : model_(model), encL_(encL), encR_(encR), wheelRadius_(wheelRadius), imu_(imu) {
        reset();
    }

    void reset(float initialTheta = 0.0f) {
        _float_t pdiag[EKF_N] = {1e-4f, 1e-4f, 1e-3f};
        ekf_initialize(&ekf_, pdiag);
        ekf_.x[State::THETA] = initialTheta;
        theta0_ = initialTheta;
        wssResetCallback();
        if (imu_ != nullptr) {
            imu_->resetHeading();
        }
        leftDist_ = 0;
        rightDist_ = 0;
        lastUpdateUs_ = 0;
    }

    void update() {
        uint32_t nowUs = micros();
        if (lastUpdateUs_ == 0) {
            lastUpdateUs_ = nowUs;
            return;
        }

        float dt = (nowUs - lastUpdateUs_) * 1e-6f;
        lastUpdateUs_ = nowUs;
        if (dt <= 0.0f) {
            return;
        }

        float dL, dR;
        wssCallback(dL, dR);
        leftDist_ += dL;
        rightDist_ += dR;

        float gyroOmega = 0.0f;
        float imuTheta = 0.0f;
        bool haveGyro = readGyro(gyroOmega, imuTheta);

        // Heading: IMU only (gyro integrate + absolute IMU θ fuse). Encoders stay for x/y.
        float dTheta;
        if (haveGyro) {
            dTheta = (fabsf(gyroOmega) >= IMU::kRateDeadband) ? (gyroOmega * dt) : 0.0f;
        } else {
            dTheta = model_.encoderDTheta(dL, dR);
        }

        const float absD = 0.5f * (fabsf(dL) + fabsf(dR));
        const float absDTheta = fabsf(dTheta);
        const float absOmega = fabsf(gyroOmega);

        model_.process(ekf_.x, dL, dR, dTheta, scratch_.fx);
        model_.jacobian(ekf_.x, dL, dR, dTheta, workF_);

        float qdiag[EKF_N];
        model_.processNoiseDiag(qdiag, absD, absDTheta);
        ekf_predict_diag(&ekf_, scratch_.fx, workF_, qdiag, &scratch_);

        if (haveGyro) {
            if (!fuseTheta(theta0_ + imuTheta, model_.gyroHeadingNoise(absOmega))) {
                return;
            }
        }

        if (!stateIsFinite()) {
            reset(ekf_.x[State::THETA]);
        }
    }

    float getX() const { return ekf_.x[State::X]; }
    float getY() const { return ekf_.x[State::Y]; }
    float getTheta() const { return ekf_.x[State::THETA]; }
    float getLeftDist() const { return leftDist_; }
    float getRightDist() const { return rightDist_; }

    void setPose(float x, float y, float theta) {
        ekf_.x[State::X] = x;
        ekf_.x[State::Y] = y;
        ekf_.x[State::THETA] = theta;
        theta0_ = theta;
        if (imu_ != nullptr) {
            imu_->resetHeading();
        }
        ekf_.P[State::X * EKF_N + State::X] = 1e-5f;
        ekf_.P[State::Y * EKF_N + State::Y] = 1e-5f;
        ekf_.P[State::THETA * EKF_N + State::THETA] = 1e-4f;
    }

    // Keep world pose; re-zero IMU relative heading ref to current θ.
    // Call after a settled movement so gyro drift does not compound from t=0.
    void reanchorHeading() {
        theta0_ = ekf_.x[State::THETA];
        if (imu_ != nullptr) {
            imu_->resetHeading();
        }
    }

    // Front ToF vs MapRay (walls-only). Expect wall only when zHat is in lidar band.
    bool correctWithFrontLidar(
        const Lidar& lidar,
        float mountAlongBeamM,
        float noiseVar = kDefaultRangeNoiseVar,
        float innovationGateM = kRangeGateM) {
        if (!lidar.isTrusted()) {
            return false;
        }
        const int bodyMm = lidar.trustedBodyRange();
        if (bodyMm == Lidar::OUT_OF_RANGE_MM) {
            return false;
        }

        const float zMeas = bodyMm * 1e-3f;
        if (zMeas < kLidarMinBodyM || zMeas > kLidarMaxBodyM) {
            return false;
        }

        const float beamYaw = ekf_.x[State::THETA];
        float zHatCenter = 0.0f;
        const float castMax = kLidarMaxBodyM + mountAlongBeamM + 0.01f;
        if (!MapRay::expectedRangeM(
                ekf_.x[State::X],
                ekf_.x[State::Y],
                beamYaw,
                castMax,
                zHatCenter,
                /*includePoles=*/false)) {
            return false;
        }

        const float zHat = zHatCenter - mountAlongBeamM;
        if (!(zHat > 0.0f) || !isfinite(zHat)) {
            return false;
        }
        // Map must also expect a face within full lidar range.
        if (zHat < kLidarMinBodyM || zHat > kLidarMaxBodyM) {
            return false;
        }

        float r = noiseVar;
        if (zHat > kFarRangeM) {
            r *= kFarNoiseScale;
        }
        return fuseRangeAlongBeam(zMeas, zHat, beamYaw, r, innovationGateM);
    }

private:
    bool fuseRangeAlongBeam(
        float zMeas,
        float zHat,
        float beamYawRad,
        float noiseVar,
        float innovationGateM) {
        if (!isfinite(zMeas) || !isfinite(zHat) || !isfinite(noiseVar) || noiseVar <= 0.0f) {
            return false;
        }

        const float H[EKF_N] = {-cosf(beamYawRad), -sinf(beamYawRad), 0.0f};

        float innov = zMeas - zHat;
        if (!isfinite(innov)) {
            return false;
        }
        if (isfinite(innovationGateM) && fabsf(innov) > innovationGateM) {
            return false;
        }

        float w = 1.0f;
        if (isfinite(innovationGateM) && innovationGateM > 1e-6f) {
            const float a = fabsf(innov) / innovationGateM;
            w = 1.0f / (1.0f + 3.0f * a * a);
        }

        float PHt[EKF_N];
        for (int i = 0; i < EKF_N; ++i) {
            PHt[i] = 0.0f;
            for (int j = 0; j < EKF_N; ++j) {
                PHt[i] += ekf_.P[i * EKF_N + j] * H[j];
            }
        }

        float S = noiseVar;
        for (int i = 0; i < EKF_N; ++i) {
            S += H[i] * PHt[i];
        }
        if (!(S > 0.0f) || !isfinite(S)) {
            return false;
        }

        const float invS = 1.0f / S;
        float dx = 0.0f;
        float dy = 0.0f;
        float K[EKF_N];
        for (int i = 0; i < EKF_N; ++i) {
            K[i] = PHt[i] * invS * w;
            const float step = K[i] * innov;
            if (i == State::X) {
                dx = step;
            } else if (i == State::Y) {
                dy = step;
            }
        }

        const float stepNorm = sqrtf(dx * dx + dy * dy);
        float scale = 1.0f;
        if (stepNorm > kMaxLidarStepM && stepNorm > 1e-9f) {
            scale = kMaxLidarStepM / stepNorm;
        }
        for (int i = 0; i < EKF_N; ++i) {
            ekf_.x[i] += K[i] * innov * scale;
        }

        float Pnew[EKF_N * EKF_N];
        for (int i = 0; i < EKF_N; ++i) {
            for (int j = 0; j < EKF_N; ++j) {
                float sum = 0.0f;
                for (int k = 0; k < EKF_N; ++k) {
                    const float IKH_ik = ((i == k) ? 1.0f : 0.0f) - K[i] * H[k];
                    sum += IKH_ik * ekf_.P[k * EKF_N + j];
                }
                Pnew[i * EKF_N + j] = sum;
            }
        }
        for (int i = 0; i < EKF_N * EKF_N; ++i) {
            ekf_.P[i] = Pnew[i];
        }

        if (!stateIsFinite()) {
            reset(ekf_.x[State::THETA]);
            return false;
        }
        return true;
    }

    static float wrapAngle(float a) {
        while (a > PI) {
            a -= 2.0f * PI;
        }
        while (a < -PI) {
            a += 2.0f * PI;
        }
        return a;
    }

    bool fuseTheta(float measured, float noise) {
        float z = wrapAngle(measured);
        float innov = wrapAngle(z - ekf_.x[State::THETA]);
        z = ekf_.x[State::THETA] + innov;
        if (!ekf_update_scalar(&ekf_, State::THETA, z, noise)) {
            reset(ekf_.x[State::THETA]);
            return false;
        }
        return true;
    }

    bool readGyro(float& omega, float& theta) {
        if (imu_ == nullptr) {
            return false;
        }
        return imu_->read(omega, theta);
    }

    void wssCallback(float& dL, float& dR) {
        float l = encL_.getRadians();
        float r = encR_.getRadians();

        dL = (l - prevL_) * wheelRadius_;
        dR = (r - prevR_) * wheelRadius_;
        prevL_ = l;
        prevR_ = r;

        if (!isfinite(dL) || fabsf(dL) > kMaxWheelStep) {
            dL = 0.0f;
        }
        if (!isfinite(dR) || fabsf(dR) > kMaxWheelStep) {
            dR = 0.0f;
        }
    }

    bool stateIsFinite() const {
        for (int i = 0; i < EKF_N; ++i) {
            if (!isfinite(ekf_.x[i])) {
                return false;
            }
        }
        return true;
    }

    void wssResetCallback() {
        prevL_ = encL_.getRadians();
        prevR_ = encR_.getRadians();
    }

    ekf_t ekf_;
    ekf_scratch_t scratch_;
    float workF_[EKF_N * EKF_N];
    Model& model_;
    Encoder& encL_;
    Encoder& encR_;
    IMU* imu_;
    float wheelRadius_;
    float theta0_ = 0;
    float prevL_ = 0;
    float prevR_ = 0;
    float leftDist_ = 0;
    float rightDist_ = 0;
    uint32_t lastUpdateUs_ = 0;
};
