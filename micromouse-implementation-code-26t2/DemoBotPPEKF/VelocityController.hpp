#pragma once

#include <Arduino.h>
#include <math.h>

#include "Config.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

namespace mtrn3100 {

class VelocityController {
public:
    VelocityController(Motor& leftMotor, Motor& rightMotor)
        : leftMotor_(leftMotor),
          rightMotor_(rightMotor),
          leftPID_(config::VELOCITY_KP, config::VELOCITY_KI, config::VELOCITY_KD),
          rightPID_(config::VELOCITY_KP, config::VELOCITY_KI, config::VELOCITY_KD) {}

    void reset(float leftDistanceM, float rightDistanceM) {
        previousLeftDistanceM_ = leftDistanceM;
        previousRightDistanceM_ = rightDistanceM;
        filteredLeftMps_ = 0.0f;
        filteredRightMps_ = 0.0f;
        previousLeftPWM_ = 0;
        previousRightPWM_ = 0;
        leftPID_.reset();
        rightPID_.reset();
    }

    void drive(
        float leftDistanceM,
        float rightDistanceM,
        float dt,
        float leftTargetMps,
        float rightTargetMps) {
        if (!(dt > 0.0f) || dt > 0.1f) {
            stop();
            return;
        }

        float rawLeftMps = (leftDistanceM - previousLeftDistanceM_) / dt;
        float rawRightMps = (rightDistanceM - previousRightDistanceM_) / dt;
        previousLeftDistanceM_ = leftDistanceM;
        previousRightDistanceM_ = rightDistanceM;
        rawLeftMps = saneVelocity(rawLeftMps);
        rawRightMps = saneVelocity(rawRightMps);
        filteredLeftMps_ += config::VELOCITY_FILTER_ALPHA
                          * (rawLeftMps - filteredLeftMps_);
        filteredRightMps_ += config::VELOCITY_FILTER_ALPHA
                           * (rawRightMps - filteredRightMps_);

        const float leftFeedback = leftPID_.update(
            leftTargetMps, filteredLeftMps_, dt);
        const float rightFeedback = rightPID_.update(
            rightTargetMps, filteredRightMps_, dt);
        int16_t leftPWM = static_cast<int16_t>(
            leftTargetMps * config::PWM_PER_MPS + leftFeedback);
        int16_t rightPWM = static_cast<int16_t>(
            rightTargetMps * config::PWM_PER_MPS + rightFeedback);
        leftPWM = rampAndClamp(leftPWM, previousLeftPWM_);
        rightPWM = rampAndClamp(rightPWM, previousRightPWM_);
        previousLeftPWM_ = leftPWM;
        previousRightPWM_ = rightPWM;

        leftMotor_.setPWM(config::LEFT_MOTOR_FORWARD_SIGN * applyDeadzone(leftPWM));
        rightMotor_.setPWM(config::RIGHT_MOTOR_FORWARD_SIGN * applyDeadzone(rightPWM));
    }

    void stop() {
        leftMotor_.stop();
        rightMotor_.stop();
        previousLeftPWM_ = 0;
        previousRightPWM_ = 0;
        filteredLeftMps_ = 0.0f;
        filteredRightMps_ = 0.0f;
        leftPID_.reset();
        rightPID_.reset();
    }

private:
    static float saneVelocity(float velocity) {
        if (!isfinite(velocity)) return 0.0f;
        return constrain(velocity, -2.0f, 2.0f);
    }

    static int16_t rampAndClamp(int16_t target, int16_t previous) {
        target = constrain(target, -config::PWM_MAX, config::PWM_MAX);
        const int16_t delta = constrain(
            target - previous,
            -config::PWM_MAX_STEP,
            config::PWM_MAX_STEP);
        return previous + delta;
    }

    static int16_t applyDeadzone(int16_t pwm) {
        const int16_t magnitude = abs(pwm);
        if (magnitude == 0 || magnitude >= config::PWM_THRESHOLD) return pwm;
        const int16_t mapped = map(
            magnitude, 0, config::PWM_THRESHOLD,
            config::PWM_MIN, config::PWM_THRESHOLD);
        return pwm < 0 ? -mapped : mapped;
    }

    Motor& leftMotor_;
    Motor& rightMotor_;
    PIDController leftPID_;
    PIDController rightPID_;
    float previousLeftDistanceM_ = 0.0f;
    float previousRightDistanceM_ = 0.0f;
    float filteredLeftMps_ = 0.0f;
    float filteredRightMps_ = 0.0f;
    int16_t previousLeftPWM_ = 0;
    int16_t previousRightPWM_ = 0;
};

}  // namespace mtrn3100
