#pragma once

#include <Arduino.h>
#include <math.h>

#include "Motor.hpp"
#include "PID.hpp"

class VelocityPID {
public:
    static constexpr float kMaxWheelSpeed = 2.0f;

    VelocityPID(Motor& motL, Motor& motR, PID& pidL, PID& pidR,
                float pwmPerMps, float velocityFilter = 0.2f)
        : motL_(motL), motR_(motR), pidL_(pidL), pidR_(pidR),
          pwmPerMps_(pwmPerMps), velocityFilter_(velocityFilter) {}

    void reset(float leftDist, float rightDist) {
        prevLeftDist_ = leftDist;
        prevRightDist_ = rightDist;
        vLeftFilt_ = 0;
        vRightFilt_ = 0;
        pidL_.reset();
        pidR_.reset();
    }

    void drive(float leftDist, float rightDist, float dt, float vLeftCmd, float vRightCmd) {
        if (dt <= 0) {
            return;
        }

        float vLeftRaw = clampVelocity((leftDist - prevLeftDist_) / dt);
        float vRightRaw = clampVelocity((rightDist - prevRightDist_) / dt);
        prevLeftDist_ = leftDist;
        prevRightDist_ = rightDist;

        vLeftFilt_ += velocityFilter_ * (vLeftRaw - vLeftFilt_);
        vRightFilt_ += velocityFilter_ * (vRightRaw - vRightFilt_);

        pidL_.setTarget(vLeftCmd);
        pidR_.setTarget(vRightCmd);
        int16_t pwmL = constrain(
            (int16_t)(pidL_.compute(vLeftFilt_) + vLeftCmd * pwmPerMps_), -255, 255);
        int16_t pwmR = constrain(
            (int16_t)(pidR_.compute(vRightFilt_) + vRightCmd * pwmPerMps_), -255, 255);
        motL_.setPWM(pwmL);
        motR_.setPWM(pwmR);
    }

    void stop() {
        motL_.stop();
        motR_.stop();
        vLeftFilt_ = 0;
        vRightFilt_ = 0;
        pidL_.reset();
        pidR_.reset();
    }

private:
    static float clampVelocity(float v) {
        if (!isfinite(v)) {
            return 0.0f;
        }
        return constrain(v, -kMaxWheelSpeed, kMaxWheelSpeed);
    }

    Motor& motL_;
    Motor& motR_;
    PID& pidL_;
    PID& pidR_;
    float pwmPerMps_;
    float velocityFilter_;
    float prevLeftDist_ = 0;
    float prevRightDist_ = 0;
    float vLeftFilt_ = 0;
    float vRightFilt_ = 0;
};
