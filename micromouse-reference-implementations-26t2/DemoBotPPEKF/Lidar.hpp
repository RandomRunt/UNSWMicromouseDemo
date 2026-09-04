#pragma once

#include <Arduino.h>
#include <VL6180X.h>
#include <Wire.h>

namespace mtrn3100 {

class Lidar {
public:
    static constexpr int INVALID_RANGE_MM = -1;
    static constexpr uint8_t FILTER_WINDOW = 5;
    static constexpr uint8_t TRUST_FRAMES = 3;
    static constexpr int MIN_VALID_MM = 20;
    static constexpr int MAX_VALID_MM = 200;

    Lidar(uint8_t xshutPin, uint8_t address)
        : xshutPin_(xshutPin), address_(address) {}

    void holdInReset() {
        pinMode(xshutPin_, OUTPUT);
        digitalWrite(xshutPin_, LOW);
        ready_ = false;
        resetFilter();
    }

    bool begin(uint16_t periodMs) {
        digitalWrite(xshutPin_, HIGH);
        delay(50);

        Wire.clearWireTimeoutFlag();
        sensor_.init();
        sensor_.configureDefault();
        sensor_.setTimeout(5);
        sensor_.setAddress(address_);
        sensor_.startRangeContinuous(periodMs);
        Wire.beginTransmission(address_);
        const bool addressResponds = Wire.endTransmission() == 0;
        const bool ok = addressResponds
                     && !sensor_.timeoutOccurred()
                     && !Wire.getWireTimeoutFlag();
        Wire.clearWireTimeoutFlag();
        ready_ = ok;
        resetFilter();
        delay(50);
        return ready_;
    }

    bool pollIfReady() {
        if (!ready_) return false;
        Wire.clearWireTimeoutFlag();
        const uint8_t interruptStatus = sensor_.readReg(
            VL6180X::RESULT__INTERRUPT_STATUS_GPIO);
        if (Wire.getWireTimeoutFlag()
            || (interruptStatus & 0x07U) != 0x04U) {
            Wire.clearWireTimeoutFlag();
            return false;
        }

        const uint8_t raw = sensor_.readReg(VL6180X::RESULT__RANGE_VAL);
        const uint8_t rangeStatus = sensor_.readRangeStatus();
        sensor_.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01);
        if (Wire.getWireTimeoutFlag()) {
            Wire.clearWireTimeoutFlag();
            return false;
        }

        int rangeMm = INVALID_RANGE_MM;
        if (rangeStatus == VL6180X_ERROR_NONE) {
            rangeMm = static_cast<int>(sensor_.getScaling()) * raw;
        }
        updateFilter(rangeMm);
        return true;
    }

    bool isTrusted() const { return trusted_; }
    int trustedRangeMm() const {
        return trusted_ ? filteredMm_ : INVALID_RANGE_MM;
    }

private:
    static int median(const int* samples, uint8_t count) {
        int sorted[FILTER_WINDOW];
        for (uint8_t i = 0; i < count; ++i) sorted[i] = samples[i];
        for (uint8_t i = 1; i < count; ++i) {
            const int value = sorted[i];
            int8_t j = static_cast<int8_t>(i);
            while (j > 0 && sorted[j - 1] > value) {
                sorted[j] = sorted[j - 1];
                --j;
            }
            sorted[j] = value;
        }
        return count > 0 ? sorted[count / 2U] : INVALID_RANGE_MM;
    }

    void resetFilter() {
        historyLength_ = 0;
        goodFrames_ = 0;
        filteredMm_ = INVALID_RANGE_MM;
        trusted_ = false;
    }

    void updateFilter(int rangeMm) {
        if (rangeMm < MIN_VALID_MM || rangeMm > MAX_VALID_MM) {
            resetFilter();
            return;
        }
        if (historyLength_ < FILTER_WINDOW) {
            history_[historyLength_++] = rangeMm;
        } else {
            for (uint8_t i = 1; i < FILTER_WINDOW; ++i) {
                history_[i - 1] = history_[i];
            }
            history_[FILTER_WINDOW - 1] = rangeMm;
        }
        filteredMm_ = median(history_, historyLength_);
        if (goodFrames_ < TRUST_FRAMES) ++goodFrames_;
        trusted_ = goodFrames_ >= TRUST_FRAMES;
    }

    VL6180X sensor_;
    const uint8_t xshutPin_;
    const uint8_t address_;
    int history_[FILTER_WINDOW]{};
    uint8_t historyLength_ = 0;
    uint8_t goodFrames_ = 0;
    int filteredMm_ = INVALID_RANGE_MM;
    bool trusted_ = false;
    bool ready_ = false;
};

}  // namespace mtrn3100
