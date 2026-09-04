#pragma once

#include <Arduino.h>
#include <math.h>

#define EKF_N 3

namespace State {
constexpr int X = 0;
constexpr int Y = 1;
constexpr int THETA = 2;
}

class Model {
public:
    // Baseline heading measurement variances (R); scaled dynamically in StateEstimator.
    static constexpr float kGyroHeadingNoise = 2e-4f;
    static constexpr float kEncHeadingNoise = 2e-2f;

    explicit Model(float wheelBase) : wheelBase_(wheelBase) {}

    float wheelBase() const { return wheelBase_; }

    void process(
        const float x[EKF_N], float dL, float dR, float dTheta,
        float fx[EKF_N]) const {
        float d = (dL + dR) * 0.5f;
        float midTheta = x[State::THETA] + dTheta * 0.5f;

        fx[State::X] = x[State::X] + d * cosf(midTheta);
        fx[State::Y] = x[State::Y] + d * sinf(midTheta);
        fx[State::THETA] = x[State::THETA] + dTheta;
    }

    void jacobian(
        const float x[EKF_N], float dL, float dR, float dTheta,
        float F[EKF_N * EKF_N]) const {
        float d = (dL + dR) * 0.5f;
        float midTheta = x[State::THETA] + dTheta * 0.5f;

        for (int i = 0; i < EKF_N * EKF_N; ++i) {
            F[i] = 0;
        }

        F[State::X * EKF_N + State::X] = 1;
        F[State::X * EKF_N + State::THETA] = -d * sinf(midTheta);

        F[State::Y * EKF_N + State::Y] = 1;
        F[State::Y * EKF_N + State::THETA] = d * cosf(midTheta);

        F[State::THETA * EKF_N + State::THETA] = 1;
    }

    // Q grows with translation and yaw step (more uncertainty when moving hard).
    void processNoiseDiag(float qdiag[EKF_N], float absD, float absDTheta) const {
        const float move = 1.0f + kQTransScale * absD + kQYawScale * absDTheta;
        qdiag[State::X] = 1e-5f * move;
        qdiag[State::Y] = 1e-5f * move;
        qdiag[State::THETA] = 5e-5f * (1.0f + kQYawScale * absDTheta);
    }

    // Near stop → inflate (bias looks like heading). Moving → baseline.
    float gyroHeadingNoise(float absOmega) const {
        return kGyroHeadingNoise *
            (1.0f + kGyroStopInflate * (kGyroStopOmega / (kGyroStopOmega + absOmega)));
    }

    // Fast turn or enc↔gyro disagreement → inflate (slip).
    float encHeadingNoise(float absOmega, float absSlipDTheta) const {
        return kEncHeadingNoise *
            (1.0f + absOmega / kOmegaRef) *
            (1.0f + absSlipDTheta / kSlipRef);
    }

    float encoderDTheta(float dL, float dR) const {
        return (dR - dL) / wheelBase_;
    }

private:
    static constexpr float kQTransScale = 50.0f;     // per metre of |d|
    static constexpr float kQYawScale = 20.0f;       // per radian of |dθ|
    static constexpr float kGyroStopOmega = 0.05f;   // rad/s
    static constexpr float kGyroStopInflate = 20.0f;
    static constexpr float kOmegaRef = 1.5f;         // rad/s (~startup spin)
    static constexpr float kSlipRef = 0.02f;         // rad per tick

    float wheelBase_;
};
