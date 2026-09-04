#include "Lidar.hpp"

#include <Wire.h>

Lidar::Lidar(uint8_t xshutPin, uint8_t address)
    : mXshutPin(xshutPin), mAddress(address) {}

bool Lidar::initialise() {
    // XSHUT resets the hardware address to 0x29; reset the library object too.
    VL6180X freshDriver;
    freshDriver.last_status = 0;
    static_cast<VL6180X&>(*this) = freshDriver;
    mDistanceMm = 0;
    mUpdatedAtMs = 0;
    mSequence = 0;
    mReady = false;
    mReadingValid = false;

    pinMode(mXshutPin, OUTPUT);
    digitalWrite(mXshutPin, HIGH);
    delay(10);

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(0x29);
    if (Wire.endTransmission() != 0 || Wire.getWireTimeoutFlag()) {
        Wire.clearWireTimeoutFlag();
        return false;
    }

    init();
    configureDefault();
    setAddress(mAddress);
    const bool configurationFailed = last_status != 0
                                  || Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    if (configurationFailed) return false;

    Wire.beginTransmission(mAddress);
    mReady = Wire.endTransmission() == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    return mReady;
}

bool Lidar::startContinuous(uint16_t periodMs) {
    if (!mReady) return false;
    Wire.clearWireTimeoutFlag();
    startRangeContinuous(periodMs);
    const bool success = last_status == 0 && !Wire.getWireTimeoutFlag();
    Wire.clearWireTimeoutFlag();
    if (!success) {
        mReady = false;
        mReadingValid = false;
    }
    return success;
}

bool Lidar::readRegisterChecked(uint16_t reg, uint8_t& value) {
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

bool Lidar::writeRegisterChecked(uint16_t reg, uint8_t value) {
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
    uint8_t interruptStatus = 0;
    if (!readRegisterChecked(RESULT__INTERRUPT_STATUS_GPIO, interruptStatus)
        || (interruptStatus & 0x07) != 0x04) {
        return false;  // Measurement is not ready; never wait for it.
    }

    uint8_t rangeStatus = 0;
    uint8_t rawDistance = 0;
    if (!readRegisterChecked(RESULT__RANGE_STATUS, rangeStatus)
        || !readRegisterChecked(RESULT__RANGE_VAL, rawDistance)
        || !writeRegisterChecked(SYSTEM__INTERRUPT_CLEAR, 0x01)) {
        return false;
    }

    mDistanceMm = static_cast<uint16_t>(getScaling()) * rawDistance;
    mUpdatedAtMs = millis();
    if (++mSequence == 0) ++mSequence;
    mReadingValid = mDistanceMm != 0
                 && (rangeStatus >> 4) == VL6180X_ERROR_NONE;
    return mReadingValid;
}

bool Lidar::hasFreshReading(uint32_t maxAgeMs) const {
    return mReadingValid && millis() - mUpdatedAtMs <= maxAgeMs;
}

uint16_t Lidar::getDistance() const {
    return mDistanceMm;
}

uint16_t Lidar::getSequence() const {
    return mSequence;
}
