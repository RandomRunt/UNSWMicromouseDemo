#pragma once

#include <Arduino.h>

class PID {
public:
    PID(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd) {}

    void setTarget(float target) { target_ = target; }

    void reset() {
        integral_ = 0;
        prevError_ = 0;
        error_ = 0;
        errorVelocity_ = 0;
        prevTime_ = micros();
    }

    float compute(float input) {
        unsigned long now = micros();
        float dt = (now - prevTime_) / 1e6f;
        prevTime_ = now;

        if (dt <= 0) return 0;

        float error = target_ - input;
        integral_ += error * dt;
        float derivative = (error - prevError_) / dt;
        prevError_ = error;
        error_ = error;
        errorVelocity_ = derivative;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void tune(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    float getError() const { return error_; }
    float getErrorVelocity() const { return errorVelocity_; }

private:
    float kp_, ki_, kd_;
    float target_ = 0;
    float integral_ = 0;
    float prevError_ = 0;
    float error_ = 0;
    float errorVelocity_ = 0;
    unsigned long prevTime_ = 0;
};
