#include "PIDController.hpp"

namespace mtrn3100 {

PIDController::PIDController(float kp, float ki, float kd)
    : kp(kp),
      ki(ki),
      kd(kd),
      error(0),
      integral(0),
      prev_error(0),
      setpoint(0),
      zero_ref(0) {
    prev_time = micros();
}

float PIDController::compute(float input) {
    const uint32_t now = micros();
    const float dt = static_cast<float>(now - prev_time) / 1000000.0f;
    prev_time = now;

    error = setpoint - (input - zero_ref);
    integral += error * dt;
    const float derivative = (dt > 0.0f) ? (error - prev_error) / dt : 0.0f;
    float output = kp * error + ki * integral + kd * derivative;
    prev_error = error;

    return constrain(output, -255.0f, 255.0f);
}

float PIDController::getError() const {
    return error;
}

void PIDController::zeroAndSetTarget(float zero, float target) {
    prev_time = micros();
    zero_ref = zero;
    setpoint = target;

    error = 0;
    prev_error = 0;
    integral = 0;
}

}  // namespace mtrn3100
