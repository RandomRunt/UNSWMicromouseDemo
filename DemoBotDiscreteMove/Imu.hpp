#pragma once

#include <Arduino.h>

// Yaw-only MPU6050 driver. It reads only gyro Z and rejects incomplete I2C
// transactions so a failed sample cannot permanently corrupt the heading.
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

    float mYawDeg = 0.0f;
    float mYawRateDps = 0.0f;
    float mRawYawRateDps = 0.0f;
    float mYawRateBiasDps = 0.0f;
    uint32_t mLastUpdateUs = 0;
    bool mHasSample = false;
};
