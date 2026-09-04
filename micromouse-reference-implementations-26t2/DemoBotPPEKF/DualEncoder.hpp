#pragma once

#include <Arduino.h>
#include <digitalWriteFast.h>
#include <util/atomic.h>

#include "Config.hpp"

namespace mtrn3100 {

class DualEncoder {
public:
    DualEncoder(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB)
        : leftA_(leftA), leftB_(leftB), rightA_(rightA), rightB_(rightB) {
        instance_ = this;
        pinModeFast(leftA_, INPUT_PULLUP);
        pinModeFast(leftB_, INPUT_PULLUP);
        pinModeFast(rightA_, INPUT_PULLUP);
        pinModeFast(rightB_, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(leftA_), leftISR, RISING);
        attachInterrupt(digitalPinToInterrupt(rightA_), rightISR, RISING);
    }

    float leftForwardRadians() const {
        return config::LEFT_ENCODER_FORWARD_SIGN * radiansFromCount(leftCount());
    }

    float rightForwardRadians() const {
        return config::RIGHT_ENCODER_FORWARD_SIGN * radiansFromCount(rightCount());
    }

    int32_t leftCount() const {
        int32_t snapshot;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { snapshot = leftCount_; }
        return snapshot;
    }

    int32_t rightCount() const {
        int32_t snapshot;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { snapshot = rightCount_; }
        return snapshot;
    }

    void reset() {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftCount_ = 0;
            rightCount_ = 0;
        }
    }

private:
    static float radiansFromCount(int32_t count) {
        constexpr float RADIANS_PER_COUNT =
            TWO_PI / config::ENCODER_COUNTS_PER_REVOLUTION;
        return static_cast<float>(count) * RADIANS_PER_COUNT;
    }

    void onLeftEdge() {
        leftCount_ += digitalReadFast(leftB_) ? 1 : -1;
    }

    void onRightEdge() {
        rightCount_ += digitalReadFast(rightB_) ? 1 : -1;
    }

    static void leftISR() {
        if (instance_ != nullptr) instance_->onLeftEdge();
    }

    static void rightISR() {
        if (instance_ != nullptr) instance_->onRightEdge();
    }

    const uint8_t leftA_;
    const uint8_t leftB_;
    const uint8_t rightA_;
    const uint8_t rightB_;
    volatile int32_t leftCount_ = 0;
    volatile int32_t rightCount_ = 0;
    static DualEncoder* instance_;
};

DualEncoder* DualEncoder::instance_ = nullptr;

}  // namespace mtrn3100
