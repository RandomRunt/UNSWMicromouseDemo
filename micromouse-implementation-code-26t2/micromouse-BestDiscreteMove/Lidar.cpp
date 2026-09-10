// AI-ASSISTED FILE: Mapping observation changes generated with OpenAI Codex
// (2026-08-11). AI-authored sections are identified inline.
#include "Lidar.hpp"

Lidar::Lidar(uint8_t pin, uint8_t address)
    : mPin(pin), mAddress(address) {}

bool Lidar::initialise() {
    // XSHUT returns the physical sensor to 0x29, but the Pololu object keeps
    // the last address assigned by setAddress(). Recreate only the base-driver
    // state so a Task 4.3 startup retry talks to the reset hardware address.
    VL6180X freshDriver;
    freshDriver.last_status = 0;
    static_cast<VL6180X&>(*this) = freshDriver;
    mDistance = 0;
    mLastReadingTime = 0;
    mObservationSequence = 0;
    mRangeStatus = 0xFF;
    mReady = false;
    mHasReading = false;
    mHasObservation = false;

    pinMode(mPin, OUTPUT);
    digitalWrite(mPin, HIGH);
    delay(10);

    // All sensors wake at 0x29. Probe before running the library's sizeable
    // setup sequence so a missing device cannot create a burst of failed I2C.
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) {
        Wire.clearWireTimeoutFlag();
        return false;
    }

    init();
    configureDefault();
    setAddress(mAddress);

    const bool configurationTimedOut = Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    if (configurationTimedOut || last_status != 0) return false;

    Wire.beginTransmission(mAddress);
    mReady = Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    return mReady;
}

uint16_t Lidar::getDistance() const {
    return mDistance;
}

bool Lidar::hasReading() const {
    return mHasReading;
}

bool Lidar::hasFreshReading(uint32_t maxAgeMs) const {
    return mHasReading && (millis() - mLastReadingTime <= maxAgeMs);
}

bool Lidar::hasFreshObservation(uint32_t maxAgeMs) const {
    return mHasObservation
        && (millis() - mLastReadingTime <= maxAgeMs);
}

bool Lidar::getLatestObservation(LidarObservation& observation) const {
    if (!mHasObservation) return false;
    observation.distanceMm = mDistance;
    observation.sequence = mObservationSequence;
    observation.rangeStatus = mRangeStatus;
    observation.rangeValid = mHasReading;
    return true;
}

uint32_t Lidar::getLastReadingTime() const {
    return mLastReadingTime;
}

bool Lidar::startContinuous(uint16_t periodMs) {
    if (!mReady) return false;

    Wire.clearWireTimeoutFlag();
    startRangeContinuous(periodMs);
    const bool success = last_status == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    if (!success) {
        mReady = false;
        mHasReading = false;
    }
    return success;
}

bool Lidar::readRuntimeRegister(uint16_t reg, uint8_t& value) {
    if (!mReady) return false;

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(mAddress);
    Wire.write(static_cast<uint8_t>(reg >> 8));
    Wire.write(static_cast<uint8_t>(reg));
    if (Wire.endTransmission(false) != 0 || Wire.getWireTimeoutFlag()) {
        Wire.clearWireTimeoutFlag();
        return false;
    }

    if (Wire.requestFrom(mAddress, static_cast<uint8_t>(1)) != 1
        || Wire.getWireTimeoutFlag()
        || Wire.available() != 1) {
        Wire.clearWireTimeoutFlag();
        while (Wire.available()) Wire.read();
        return false;
    }

    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool Lidar::writeRuntimeRegister(uint16_t reg, uint8_t value) {
    if (!mReady) return false;

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(mAddress);
    Wire.write(static_cast<uint8_t>(reg >> 8));
    Wire.write(static_cast<uint8_t>(reg));
    Wire.write(value);
    const uint8_t status = Wire.endTransmission();
    const bool success = status == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    return success;
}

bool Lidar::updateDistance() {
    uint8_t status = 0;
    if (!readRuntimeRegister(RESULT__INTERRUPT_STATUS_GPIO, status)
        || (status & 0x07) != 0x04) return false;

    uint8_t rangeStatus = 0;
    if (!readRuntimeRegister(RESULT__RANGE_STATUS, rangeStatus)) return false;

    uint8_t rawDistance = 0;
    if (!readRuntimeRegister(RESULT__RANGE_VAL, rawDistance)
        || !writeRuntimeRegister(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    const uint16_t distance = static_cast<uint16_t>(getScaling()) * rawDistance;
    // Preserve every completed observation for diagnostics, while exposing it
    // to movement control only when the VL6180X marks the range as valid.
    mDistance = distance;
    mRangeStatus = rangeStatus >> 4;
    mLastReadingTime = millis();
    mHasObservation = true;
    if (++mObservationSequence == 0) ++mObservationSequence;
    mHasReading = distance != 0
               && mRangeStatus == VL6180X_ERROR_NONE;
    return mHasReading;
}
