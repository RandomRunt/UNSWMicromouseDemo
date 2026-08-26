#pragma once

#include <Arduino.h>
#include <math.h>

#define EKF_N 3

namespace mtrn3100 {

namespace StateIndex {
constexpr uint8_t X = 0;
constexpr uint8_t Y = 1;
constexpr uint8_t HEADING = 2;
}

class Model {
public:
    explicit Model(float wheelBaseM) : wheelBaseM_(wheelBaseM) {}

    void process(
        const float state[EKF_N], float dLeft, float dRight, float dHeading,
        float predicted[EKF_N]) const {
        const float distance = 0.5f * (dLeft + dRight);
        const float middleHeading = state[StateIndex::HEADING] + 0.5f * dHeading;
        predicted[StateIndex::X] = state[StateIndex::X]
                                 + distance * cosf(middleHeading);
        predicted[StateIndex::Y] = state[StateIndex::Y]
                                 + distance * sinf(middleHeading);
        predicted[StateIndex::HEADING] = state[StateIndex::HEADING] + dHeading;
    }

    void jacobian(
        const float state[EKF_N], float dLeft, float dRight, float dHeading,
        float result[EKF_N * EKF_N]) const {
        const float distance = 0.5f * (dLeft + dRight);
        const float middleHeading = state[StateIndex::HEADING] + 0.5f * dHeading;
        for (uint8_t i = 0; i < EKF_N * EKF_N; ++i) result[i] = 0.0f;
        result[StateIndex::X * EKF_N + StateIndex::X] = 1.0f;
        result[StateIndex::X * EKF_N + StateIndex::HEADING] =
            -distance * sinf(middleHeading);
        result[StateIndex::Y * EKF_N + StateIndex::Y] = 1.0f;
        result[StateIndex::Y * EKF_N + StateIndex::HEADING] =
            distance * cosf(middleHeading);
        result[StateIndex::HEADING * EKF_N + StateIndex::HEADING] = 1.0f;
    }

    void processNoise(float diagonal[EKF_N], float distance, float heading) const {
        const float motionScale = 1.0f + 50.0f * fabsf(distance)
                                + 20.0f * fabsf(heading);
        diagonal[StateIndex::X] = 1e-5f * motionScale;
        diagonal[StateIndex::Y] = 1e-5f * motionScale;
        diagonal[StateIndex::HEADING] =
            5e-5f * (1.0f + 20.0f * fabsf(heading));
    }

    float encoderHeadingChange(float dLeft, float dRight) const {
        return (dRight - dLeft) / wheelBaseM_;
    }

    float gyroHeadingNoise(float absoluteOmega) const {
        constexpr float BASE = 2e-4f;
        constexpr float STOP_OMEGA = 0.05f;
        return BASE * (1.0f + 20.0f *
            (STOP_OMEGA / (STOP_OMEGA + absoluteOmega)));
    }

private:
    const float wheelBaseM_;
};

}  // namespace mtrn3100
