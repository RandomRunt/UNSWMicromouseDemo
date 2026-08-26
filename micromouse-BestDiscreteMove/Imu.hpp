#pragma once

#include <Arduino.h>

// Yaw-only MPU6050 driver for the competition firmware. Reading only gyro Z
// cuts each control sample from 14 bytes to 2 and rejects incomplete I2C reads
// instead of integrating corrupt data into every later heading.
class Imu {
public:
    bool initialise();
    bool update();

    float getYaw() const;
    float getYawRate() const;
    float getRawYawRate() const;
    void setYawRateBias(float biasDps);

private:
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readGyroZ(int16_t& raw);

    float mAngleZ = 0.0f;
    float mRateZ = 0.0f;
    float mRawRateZ = 0.0f;
    float mYawRateBiasDps = 0.0f;
    uint32_t mLastUpdateUs = 0;
    bool mHasRateSample = false;
};
