#pragma once

#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    float compute(float input);
    float getError() const;
    void zeroAndSetTarget(float zero, float target);

private:
    float kp, ki, kd;
    float error, integral;
    float prev_error;
    float setpoint;
    float zero_ref;
    uint32_t prev_time;
};

}  // namespace mtrn3100
