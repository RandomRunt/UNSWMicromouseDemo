#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <math.h>

namespace mtrn3100 {

// A-style tilt-compensated planar yaw source. The gyro vector is projected
// onto the estimated world-up direction before integration.
class Imu {
public:
    explicit Imu(TwoWire& wire = Wire) : mpu_(wire) {}

    bool begin() {
        if (mpu_.begin() != 0) return false;
        delay(2000);
        mpu_.calcOffsets(true);
        mpu_.update();
        theta_ = 0.0f;
        omegaUp_ = 0.0f;
        lastUs_ = micros();
        refreshRate();
        ready_ = true;
        return true;
    }

    bool read(float& omegaRadS, float& thetaRad) {
        if (!ready_) return false;
        mpu_.update();
        const uint32_t nowUs = micros();
        float dt = static_cast<float>(nowUs - lastUs_) * 1e-6f;
        lastUs_ = nowUs;
        if (!(dt >= 0.0f) || dt > 0.1f) dt = 0.0f;
        refreshRate();
        if (dt > 0.0f) theta_ += omegaUp_ * dt;
        omegaRadS = omegaUp_;
        thetaRad = theta_;
        return true;
    }

    void resetHeading() {
        theta_ = 0.0f;
        omegaUp_ = 0.0f;
        if (!ready_) return;
        mpu_.update();
        lastUs_ = micros();
        refreshRate();
    }

    float omegaRadS() const { return omegaUp_; }
    bool isReady() const { return ready_; }
    void setHeadingSign(float sign) { headingSign_ = sign < 0.0f ? -1.0f : 1.0f; }

private:
    void refreshRate() {
        const float roll = mpu_.getAngleX() * DEG_TO_RAD;
        const float pitch = mpu_.getAngleY() * DEG_TO_RAD;
        float upX = -sinf(pitch);
        float upY = sinf(roll) * cosf(pitch);
        float upZ = cosf(roll) * cosf(pitch);
        const float norm = sqrtf(upX * upX + upY * upY + upZ * upZ);
        if (norm > 1e-6f) {
            upX /= norm;
            upY /= norm;
            upZ /= norm;
        } else {
            upX = 0.0f;
            upY = 0.0f;
            upZ = 1.0f;
        }

        const float gyroX = mpu_.getGyroX() * DEG_TO_RAD;
        const float gyroY = mpu_.getGyroY() * DEG_TO_RAD;
        const float gyroZ = mpu_.getGyroZ() * DEG_TO_RAD;
        omegaUp_ = headingSign_ *
            (gyroX * upX + gyroY * upY + gyroZ * upZ);
    }

    MPU6050 mpu_;
    bool ready_ = false;
    float theta_ = 0.0f;
    float omegaUp_ = 0.0f;
    float headingSign_ = 1.0f;
    uint32_t lastUs_ = 0;
};

}  // namespace mtrn3100
