#include "Imu.hpp"

#include <Wire.h>

namespace {

constexpr uint8_t MPU_ADDRESS = 0x68;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

// +/-500 degrees/s leaves ample turn-rate headroom while retaining 0.015 dps
// resolution. DLPF=3 gives a 44 Hz gyro bandwidth; divider 4 gives 200 Hz.
constexpr float GYRO_LSB_PER_DPS = 65.5f;
constexpr uint8_t EXPECTED_DEVICE_ID = 0x68;
constexpr uint8_t STARTUP_PROBE_ATTEMPTS = 20;

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
    const bool addressTimedOut = wireTimedOut();
    if (status != 0 || addressTimedOut) return false;

    const uint8_t received = Wire.requestFrom(
        MPU_ADDRESS, static_cast<uint8_t>(1));
    const bool readTimedOut = wireTimedOut();
    if (received != 1
        || readTimedOut
        || Wire.available() != 1) {
        while (Wire.available()) Wire.read();
        return false;
    }

    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool Imu::readGyroZ(int16_t& raw) {
    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(REG_GYRO_ZOUT_H);
    const uint8_t status = Wire.endTransmission(false);
    const bool addressTimedOut = wireTimedOut();
    if (status != 0 || addressTimedOut) return false;

    const uint8_t received = Wire.requestFrom(
        MPU_ADDRESS, static_cast<uint8_t>(2));
    const bool readTimedOut = wireTimedOut();
    if (received != 2
        || readTimedOut
        || Wire.available() != 2) {
        while (Wire.available()) Wire.read();
        return false;
    }

    const uint8_t high = static_cast<uint8_t>(Wire.read());
    const uint8_t low = static_cast<uint8_t>(Wire.read());
    raw = static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    return true;
}

bool Imu::initialise() {
    mAngleZ = 0.0f;
    mRateZ = 0.0f;
    mRawRateZ = 0.0f;
    mYawRateBiasDps = 0.0f;
    mHasRateSample = false;

    uint8_t deviceId = 0;
    uint8_t attempt = 0;
    while (attempt++ < STARTUP_PROBE_ATTEMPTS) {
        if (readRegister(REG_WHO_AM_I, deviceId)
            && deviceId == EXPECTED_DEVICE_ID) {
            break;
        }
        delay(10);
    }
    if (deviceId != EXPECTED_DEVICE_ID) return false;

    // Reset, select the X-gyro PLL clock, then configure a filtered 200 Hz
    // gyro-Z stream. Every write is checked before motion can be enabled.
    if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
    delay(100);
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)
        || !writeRegister(REG_CONFIG, 0x03)
        || !writeRegister(REG_SMPLRT_DIV, 0x04)
        || !writeRegister(REG_GYRO_CONFIG, 0x08)) {
        return false;
    }

    // Preserve the existing warm-up period: cold gyro bias changes quickly and
    // is the main source of progressively inaccurate turns.
    delay(5000);
    mLastUpdateUs = micros();
    return update();
}

bool Imu::update() {
    int16_t raw = 0;
    if (!readGyroZ(raw)) {
        // Do not apply the next good rate over an interval for which no sample
        // existed; that is how a short I2C failure becomes a permanent yaw jump.
        mLastUpdateUs = micros();
        return false;
    }

    const uint32_t nowUs = micros();
    mRawRateZ = static_cast<float>(raw) / GYRO_LSB_PER_DPS;
    mRateZ = mRawRateZ - mYawRateBiasDps;

    if (mHasRateSample) {
        const float dt = static_cast<float>(nowUs - mLastUpdateUs) / 1000000.0f;
        mAngleZ += mRateZ * dt;
    } else {
        mHasRateSample = true;
    }
    mLastUpdateUs = nowUs;
    return true;
}

float Imu::getYaw() const {
    return mAngleZ;
}

float Imu::getYawRate() const {
    return mRateZ;
}

float Imu::getRawYawRate() const {
    return mRawRateZ;
}

void Imu::setYawRateBias(float biasDps) {
    mYawRateBiasDps = biasDps;
    mRateZ = mRawRateZ - mYawRateBiasDps;
}
