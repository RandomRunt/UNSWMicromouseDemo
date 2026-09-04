#pragma once

namespace mtrn3100 {
  class PIDController {
    public:
      PIDController(float kp, float ki, float kd)
        : Kp(kp), Ki(ki), Kd(kd), integral(0), prevError(0), firstRun(true) {}

      float update(float setpoint, float measured, float dt) {
        float error = setpoint - measured;  // Calculate error (desired value - measured)
        integral += error * dt;             // Add the area under error curve (error multiplied by time taken for the error)

        float derivative = 0;
        if (!firstRun) {
          derivative = (error - prevError) / dt;  // Calculate the gradient at a point (essentially how steep the approach angle is)
        } else {
          firstRun = false;
        }

        prevError = error;

        return Kp * error + Ki * integral + Kd * derivative;  // Return PID!
      }

      void reset() {
        integral = 0;
        prevError = 0;
        firstRun = true;
      }
    
    private:
      float Kp, Ki, Kd;
      float integral;
      float prevError;
      bool firstRun;
  };
}