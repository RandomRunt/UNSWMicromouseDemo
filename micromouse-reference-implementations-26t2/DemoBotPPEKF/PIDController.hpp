#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    float update(float setpoint, float measured, float dt) {
        if (!(dt > 0.0f) || !isfinite(dt)) return 0.0f;
        const float error = setpoint - measured;
        integral_ += error * dt;
        const float derivative = firstRun_ ? 0.0f : (error - previousError_) / dt;
        previousError_ = error;
        firstRun_ = false;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset() {
        integral_ = 0.0f;
        previousError_ = 0.0f;
        firstRun_ = true;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float integral_ = 0.0f;
    float previousError_ = 0.0f;
    bool firstRun_ = true;
};

}  // namespace mtrn3100
