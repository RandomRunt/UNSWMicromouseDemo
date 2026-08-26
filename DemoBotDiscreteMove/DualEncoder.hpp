#pragma once

#include <Arduino.h>
#include <util/atomic.h>

class DualEncoder {
public:
    DualEncoder(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB)
        : mLeftA(leftA),
          mLeftB(leftB),
          mRightA(rightA),
          mRightB(rightB) {
        sInstance = this;
        pinMode(mLeftA, INPUT_PULLUP);
        pinMode(mLeftB, INPUT_PULLUP);
        pinMode(mRightA, INPUT_PULLUP);
        pinMode(mRightB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(mLeftA), leftISR, RISING);
        attachInterrupt(digitalPinToInterrupt(mRightA), rightISR, RISING);
    }

    float leftRotation() const {
        int32_t snapshot;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            snapshot = mLeftCount;
        }
        return static_cast<float>(snapshot) * TWO_PI / COUNTS_PER_REVOLUTION;
    }

    float rightRotation() const {
        int32_t snapshot;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            snapshot = mRightCount;
        }
        return static_cast<float>(snapshot) * TWO_PI / COUNTS_PER_REVOLUTION;
    }

private:
    void countLeft() {
        mLeftCount += digitalRead(mLeftB) == HIGH ? 1 : -1;
    }

    void countRight() {
        mRightCount += digitalRead(mRightB) == HIGH ? 1 : -1;
    }

    static void leftISR() {
        if (sInstance != nullptr) sInstance->countLeft();
    }

    static void rightISR() {
        if (sInstance != nullptr) sInstance->countRight();
    }

    static constexpr float COUNTS_PER_REVOLUTION = 700.0f;
    const uint8_t mLeftA;
    const uint8_t mLeftB;
    const uint8_t mRightA;
    const uint8_t mRightB;
    volatile int32_t mLeftCount = 0;
    volatile int32_t mRightCount = 0;
    static DualEncoder* sInstance;
};

DualEncoder* DualEncoder::sInstance = nullptr;
