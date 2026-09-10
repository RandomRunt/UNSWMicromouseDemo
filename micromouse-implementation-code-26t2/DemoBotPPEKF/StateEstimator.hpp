#pragma once

#include <Arduino.h>
#include <math.h>

#include "DualEncoder.hpp"
#include "Imu.hpp"
#include "Lidar.hpp"
#include "MapRay.hpp"
#include "Model.hpp"
#include "TinyEkf.hpp"

namespace mtrn3100 {

class StateEstimator {
public:
    StateEstimator(
        Model& model,
        DualEncoder& encoders,
        float wheelRadiusM,
        Imu* imu)
        : model_(model),
          encoders_(encoders),
          wheelRadiusM_(wheelRadiusM),
          imu_(imu) {
        reset();
    }

    void reset(float initialHeading = 0.0f) {
        const float covarianceDiagonal[EKF_N] = {1e-4f, 1e-4f, 1e-3f};
        ekfInitialise(ekf_, covarianceDiagonal);
        ekf_.x[StateIndex::HEADING] = initialHeading;
        headingAnchor_ = initialHeading;
        previousLeftRadians_ = encoders_.leftForwardRadians();
        previousRightRadians_ = encoders_.rightForwardRadians();
        leftDistanceM_ = 0.0f;
        rightDistanceM_ = 0.0f;
        lastUpdateUs_ = 0;
        if (imu_ != nullptr) imu_->resetHeading();
    }

    bool update() {
        const uint32_t nowUs = micros();
        if (lastUpdateUs_ == 0) {
            lastUpdateUs_ = nowUs;
            return false;
        }
        const float dt = static_cast<float>(nowUs - lastUpdateUs_) * 1e-6f;
        lastUpdateUs_ = nowUs;
        if (!(dt > 0.0f) || dt > 0.1f) return false;

        float deltaLeft = 0.0f;
        float deltaRight = 0.0f;
        readWheelSteps(deltaLeft, deltaRight);
        leftDistanceM_ += deltaLeft;
        rightDistanceM_ += deltaRight;

        float gyroOmega = 0.0f;
        float imuHeading = 0.0f;
        const bool haveImu = imu_ != nullptr
                          && imu_->read(gyroOmega, imuHeading);
        const float deltaHeading = haveImu
            ? gyroOmega * dt
            : model_.encoderHeadingChange(deltaLeft, deltaRight);

        model_.process(
            ekf_.x,
            deltaLeft,
            deltaRight,
            deltaHeading,
            scratch_.predicted);
        model_.jacobian(
            ekf_.x,
            deltaLeft,
            deltaRight,
            deltaHeading,
            jacobian_);
        float processNoise[EKF_N];
        model_.processNoise(
            processNoise,
            0.5f * (fabsf(deltaLeft) + fabsf(deltaRight)),
            fabsf(deltaHeading));
        ekfPredictDiagonal(
            ekf_, scratch_.predicted, jacobian_, processNoise, scratch_);

        if (haveImu) {
            fuseHeading(
                headingAnchor_ + imuHeading,
                model_.gyroHeadingNoise(fabsf(gyroOmega)));
        }

        if (!stateIsFinite()) {
            reset();
            return false;
        }
        return true;
    }

    bool correctWithFrontLidar(const Lidar& lidar, float sensorMountM) {
        if (!lidar.isTrusted()) return false;
        const int measuredMm = lidar.trustedRangeMm();
        if (measuredMm == Lidar::INVALID_RANGE_MM) return false;
        const float measuredM = measuredMm * 1e-3f;
        if (measuredM < MIN_LIDAR_RANGE_M || measuredM > MAX_LIDAR_RANGE_M) {
            return false;
        }

        float expectedFromCentreM = 0.0f;
        const float castMaximumM = MAX_LIDAR_RANGE_M + sensorMountM + 0.01f;
        if (!MapRay::expectedRange(
                x(), y(), heading(), castMaximumM, expectedFromCentreM)) {
            return false;
        }
        const float expectedM = expectedFromCentreM - sensorMountM;
        if (expectedM < MIN_LIDAR_RANGE_M || expectedM > MAX_LIDAR_RANGE_M) {
            return false;
        }

        float noise = DEFAULT_RANGE_NOISE;
        if (expectedM > FAR_RANGE_M) noise *= FAR_NOISE_SCALE;
        return fuseRange(
            measuredM,
            expectedM,
            heading(),
            noise,
            RANGE_INNOVATION_GATE_M);
    }

    void reanchorHeading() {
        headingAnchor_ = heading();
        if (imu_ != nullptr) imu_->resetHeading();
    }

    float x() const { return ekf_.x[StateIndex::X]; }
    float y() const { return ekf_.x[StateIndex::Y]; }
    float heading() const { return ekf_.x[StateIndex::HEADING]; }
    float leftDistanceM() const { return leftDistanceM_; }
    float rightDistanceM() const { return rightDistanceM_; }

private:
    static constexpr float MAX_WHEEL_STEP_M = 0.05f;
    static constexpr float MIN_LIDAR_RANGE_M = 0.020f;
    static constexpr float MAX_LIDAR_RANGE_M = 0.200f;
    static constexpr float RANGE_INNOVATION_GATE_M = 0.060f;
    static constexpr float DEFAULT_RANGE_NOISE = 4e-4f;
    static constexpr float FAR_RANGE_M = 0.160f;
    static constexpr float FAR_NOISE_SCALE = 4.0f;
    static constexpr float MAX_LIDAR_CORRECTION_STEP_M = 0.012f;

    static float wrapAngle(float angle) {
        while (angle > PI) angle -= TWO_PI;
        while (angle < -PI) angle += TWO_PI;
        return angle;
    }

    void readWheelSteps(float& deltaLeft, float& deltaRight) {
        const float leftRadians = encoders_.leftForwardRadians();
        const float rightRadians = encoders_.rightForwardRadians();
        deltaLeft = (leftRadians - previousLeftRadians_) * wheelRadiusM_;
        deltaRight = (rightRadians - previousRightRadians_) * wheelRadiusM_;
        previousLeftRadians_ = leftRadians;
        previousRightRadians_ = rightRadians;
        if (!isfinite(deltaLeft) || fabsf(deltaLeft) > MAX_WHEEL_STEP_M) {
            deltaLeft = 0.0f;
        }
        if (!isfinite(deltaRight) || fabsf(deltaRight) > MAX_WHEEL_STEP_M) {
            deltaRight = 0.0f;
        }
    }

    bool fuseHeading(float measurement, float noise) {
        const float innovation = wrapAngle(measurement - heading());
        return ekfUpdateScalar(
            ekf_,
            StateIndex::HEADING,
            heading() + innovation,
            noise);
    }

    bool fuseRange(
        float measured,
        float expected,
        float beamHeading,
        float noise,
        float innovationGate) {
        const float innovation = measured - expected;
        if (!isfinite(innovation) || fabsf(innovation) > innovationGate) {
            return false;
        }

        const float observation[EKF_N] = {
            -cosf(beamHeading),
            -sinf(beamHeading),
            0.0f,
        };
        float covarianceTimesObservation[EKF_N] = {};
        for (uint8_t row = 0; row < EKF_N; ++row) {
            for (uint8_t column = 0; column < EKF_N; ++column) {
                covarianceTimesObservation[row] +=
                    ekf_.covariance[row * EKF_N + column]
                    * observation[column];
            }
        }

        float innovationVariance = noise;
        for (uint8_t i = 0; i < EKF_N; ++i) {
            innovationVariance += observation[i]
                                * covarianceTimesObservation[i];
        }
        if (!(innovationVariance > 0.0f) || !isfinite(innovationVariance)) {
            return false;
        }

        const float normalisedInnovation = fabsf(innovation) / innovationGate;
        const float robustWeight = 1.0f /
            (1.0f + 3.0f * normalisedInnovation * normalisedInnovation);
        float gain[EKF_N];
        for (uint8_t i = 0; i < EKF_N; ++i) {
            gain[i] = covarianceTimesObservation[i]
                    / innovationVariance * robustWeight;
        }

        const float proposedX = gain[StateIndex::X] * innovation;
        const float proposedY = gain[StateIndex::Y] * innovation;
        const float proposedNorm = hypotf(proposedX, proposedY);
        const float correctionScale = proposedNorm > MAX_LIDAR_CORRECTION_STEP_M
            ? MAX_LIDAR_CORRECTION_STEP_M / proposedNorm
            : 1.0f;
        for (uint8_t i = 0; i < EKF_N; ++i) {
            ekf_.x[i] += gain[i] * innovation * correctionScale;
        }

        float updatedCovariance[EKF_N * EKF_N];
        for (uint8_t row = 0; row < EKF_N; ++row) {
            for (uint8_t column = 0; column < EKF_N; ++column) {
                float sum = 0.0f;
                for (uint8_t k = 0; k < EKF_N; ++k) {
                    const float identityMinusGainH =
                        (row == k ? 1.0f : 0.0f)
                        - gain[row] * observation[k];
                    sum += identityMinusGainH
                         * ekf_.covariance[k * EKF_N + column];
                }
                updatedCovariance[row * EKF_N + column] = sum;
            }
        }
        memcpy(
            ekf_.covariance,
            updatedCovariance,
            sizeof(updatedCovariance));
        return stateIsFinite();
    }

    bool stateIsFinite() const {
        for (uint8_t i = 0; i < EKF_N; ++i) {
            if (!isfinite(ekf_.x[i])) return false;
        }
        return true;
    }

    EkfState ekf_;
    EkfScratch scratch_;
    float jacobian_[EKF_N * EKF_N];
    Model& model_;
    DualEncoder& encoders_;
    const float wheelRadiusM_;
    Imu* imu_;
    float previousLeftRadians_ = 0.0f;
    float previousRightRadians_ = 0.0f;
    float leftDistanceM_ = 0.0f;
    float rightDistanceM_ = 0.0f;
    float headingAnchor_ = 0.0f;
    uint32_t lastUpdateUs_ = 0;
};

}  // namespace mtrn3100
