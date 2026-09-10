#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>
#include <math.h>

// Planar heading from full attitude + full gyro, for a possibly tilted mount:
//   ω_up = ω_body · û   (û = sensor "up" from fused roll/pitch)
//   θ   = ∫ ω_up dt
// Quat is ZYX(roll, pitch, θ) so getQuat() matches what the EKF sees.
class IMU {
public:
    static constexpr uint8_t kAddress = 0x68;
    static constexpr float kRateDeadband = 0.0f;

    IMU() : mpu_(Wire) {}

    bool begin(uint8_t /*addr*/ = kAddress) {
        Wire.begin();
        Wire.setWireTimeout(3000, true);

        if (mpu_.begin() != 0) {
            ready_ = false;
            return false;
        }

        delay(2000);
        mpu_.calcOffsets(true);

        mpu_.update();
        theta_ = 0.0f;
        omegaUp_ = 0.0f;
        lastUs_ = micros();
        refreshRates();
        quatFromEulerZYX(roll_, pitch_, theta_, qw_, qx_, qy_, qz_);
        ready_ = true;
        return true;
    }

    bool read(float& omegaZ, float& theta) {
        if (!ready_) {
            return false;
        }

        mpu_.update();

        const uint32_t nowUs = micros();
        float dt = (nowUs - lastUs_) * 1e-6f;
        lastUs_ = nowUs;
        if (dt < 0.0f || dt > 0.1f) {
            dt = 0.0f;
        }

        refreshRates();

        omegaZ = (fabsf(omegaUp_) < kRateDeadband) ? 0.0f : omegaUp_;
        if (dt > 0.0f) {
            theta_ += omegaZ * dt;
        }
        quatFromEulerZYX(roll_, pitch_, theta_, qw_, qx_, qy_, qz_);

        theta = theta_;
        return true;
    }

    void resetHeading() {
        theta_ = 0.0f;
        omegaUp_ = 0.0f;
        if (!ready_) {
            qw_ = 1.0f;
            qx_ = qy_ = qz_ = 0.0f;
            return;
        }
        mpu_.update();
        lastUs_ = micros();
        refreshRates();
        quatFromEulerZYX(roll_, pitch_, theta_, qw_, qx_, qy_, qz_);
    }

    float getHeading() const { return theta_; }
    float getOmegaZ() const { return omegaUp_; }
    bool isReady() const { return ready_; }

    // Attitude quat (w, x, y, z): body from world, ZYX (roll, pitch, yaw=θ_up).
    void getQuat(float& w, float& x, float& y, float& z) const {
        w = qw_;
        x = qx_;
        y = qy_;
        z = qz_;
    }

    float quatW() const { return qw_; }
    float quatX() const { return qx_; }
    float quatY() const { return qy_; }
    float quatZ() const { return qz_; }

    void setHeadingSign(float sign) { headingSign_ = (sign < 0.0f) ? -1.0f : 1.0f; }

private:
    void refreshRates() {
        roll_ = mpu_.getAngleX() * DEG_TO_RAD;
        pitch_ = mpu_.getAngleY() * DEG_TO_RAD;

        float ux = -sinf(pitch_);
        float uy = sinf(roll_) * cosf(pitch_);
        float uz = cosf(roll_) * cosf(pitch_);
        const float n = sqrtf(ux * ux + uy * uy + uz * uz);
        if (n > 1e-6f) {
            ux /= n;
            uy /= n;
            uz /= n;
        } else {
            ux = 0.0f;
            uy = 0.0f;
            uz = 1.0f;
        }

        const float wx = mpu_.getGyroX() * DEG_TO_RAD;
        const float wy = mpu_.getGyroY() * DEG_TO_RAD;
        const float wz = mpu_.getGyroZ() * DEG_TO_RAD;
        omegaUp_ = headingSign_ * (wx * ux + wy * uy + wz * uz);
    }

    static void quatFromEulerZYX(
        float roll, float pitch, float yaw,
        float& qw, float& qx, float& qy, float& qz) {
        const float cr = cosf(roll * 0.5f);
        const float sr = sinf(roll * 0.5f);
        const float cp = cosf(pitch * 0.5f);
        const float sp = sinf(pitch * 0.5f);
        const float cy = cosf(yaw * 0.5f);
        const float sy = sinf(yaw * 0.5f);

        qw = cr * cp * cy + sr * sp * sy;
        qx = sr * cp * cy - cr * sp * sy;
        qy = cr * sp * cy + sr * cp * sy;
        qz = cr * cp * sy - sr * sp * cy;
    }

    MPU6050 mpu_;
    bool ready_ = false;
    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float theta_ = 0.0f;
    float omegaUp_ = 0.0f;
    float headingSign_ = 1.0f;
    uint32_t lastUs_ = 0;
    float qw_ = 1.0f;
    float qx_ = 0.0f;
    float qy_ = 0.0f;
    float qz_ = 0.0f;
};
