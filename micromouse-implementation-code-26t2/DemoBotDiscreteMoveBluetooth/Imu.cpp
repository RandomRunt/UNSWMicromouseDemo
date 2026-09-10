#include "Imu.hpp"

#include <Wire.h>

namespace {

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr uint8_t REG_SAMPLE_RATE_DIVIDER = 0x19;
constexpr uint8_t REG_FILTER_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_Z_HIGH = 0x47;
constexpr uint8_t REG_POWER_MANAGEMENT = 0x6B;
constexpr uint8_t REG_DEVICE_ID = 0x75;
constexpr uint8_t EXPECTED_DEVICE_ID = 0x68;
constexpr uint8_t STARTUP_PROBE_ATTEMPTS = 20;
constexpr float GYRO_LSB_PER_DPS = 65.5f;

bool wireTimedOut() {
    if (!Wire.getWireTimeoutFlag()) return false;
    Wire.clearWireTimeoutFlag();
    return true;
}

}  // namespace

bool Imu::writeRegister(uint8_t reg, uint8_t value) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    const uint8_t status = Wire.endTransmission();
    const bool timedOut = wireTimedOut();
    return status == 0 && !timedOut;
}

bool Imu::readRegister(uint8_t reg, uint8_t& value) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(reg);
    const uint8_t status = Wire.endTransmission(false);
    if (status != 0 || wireTimedOut()) return false;

    const uint8_t received = Wire.requestFrom(
        MPU_ADDRESS, static_cast<uint8_t>(1));
    if (received != 1 || wireTimedOut() || Wire.available() != 1) {
        while (Wire.available()) Wire.read();
        return false;
    }
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool Imu::readGyroZ(int16_t& raw) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(REG_GYRO_Z_HIGH);
    const uint8_t status = Wire.endTransmission(false);
    if (status != 0 || wireTimedOut()) return false;

    const uint8_t received = Wire.requestFrom(
        MPU_ADDRESS, static_cast<uint8_t>(2));
    if (received != 2 || wireTimedOut() || Wire.available() != 2) {
        while (Wire.available()) Wire.read();
        return false;
    }

    const uint8_t high = static_cast<uint8_t>(Wire.read());
    const uint8_t low = static_cast<uint8_t>(Wire.read());
    raw = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    return true;
}

bool Imu::initialise() {
    mYawDeg = 0.0f;
    mYawRateDps = 0.0f;
    mRawYawRateDps = 0.0f;
    mYawRateBiasDps = 0.0f;
    mHasSample = false;

    uint8_t deviceId = 0;
    for (uint8_t attempt = 0; attempt < STARTUP_PROBE_ATTEMPTS; ++attempt) {
        if (readRegister(REG_DEVICE_ID, deviceId)
            && deviceId == EXPECTED_DEVICE_ID) {
            break;
        }
        delay(10);
    }
    if (deviceId != EXPECTED_DEVICE_ID) return false;

    // Reset, use the X-gyro PLL clock, then configure filtered 200 Hz gyro Z.
    if (!writeRegister(REG_POWER_MANAGEMENT, 0x80)) return false;
    delay(100);
    if (!writeRegister(REG_POWER_MANAGEMENT, 0x01)
        || !writeRegister(REG_FILTER_CONFIG, 0x03)
        || !writeRegister(REG_SAMPLE_RATE_DIVIDER, 0x04)
        || !writeRegister(REG_GYRO_CONFIG, 0x08)) {
        return false;
    }

    delay(5000);  // Let cold-start gyro bias settle before calibration.
    mLastUpdateUs = micros();
    return update();
}

bool Imu::update() {
    int16_t raw = 0;
    if (!readGyroZ(raw)) {
        // Do not integrate the next valid sample across a missing interval.
        mLastUpdateUs = micros();
        return false;
    }

    const uint32_t nowUs = micros();
    mRawYawRateDps = static_cast<float>(raw) / GYRO_LSB_PER_DPS;
    mYawRateDps = mRawYawRateDps - mYawRateBiasDps;
    if (mHasSample) {
        const float dt = static_cast<float>(nowUs - mLastUpdateUs) / 1000000.0f;
        mYawDeg += mYawRateDps * dt;
    } else {
        mHasSample = true;
    }
    mLastUpdateUs = nowUs;
    return true;
}

float Imu::getYaw() const {
    return mYawDeg;
}

float Imu::getYawRate() const {
    return mYawRateDps;
}

float Imu::getRawYawRate() const {
    return mRawYawRateDps;
}

void Imu::setYawRateBias(float biasDps) {
    mYawRateBiasDps = biasDps;
    mYawRateDps = mRawYawRateDps - mYawRateBiasDps;
}
