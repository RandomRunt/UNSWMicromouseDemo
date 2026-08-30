#pragma once

#include <Arduino.h>

// DemoBot motor convention: positive PWM drives DIR high. The left motor is
// inverted once in drive(), so callers can use positive values for forward on
// both wheels.
class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin)
        : mPwmPin(pwmPin), mDirectionPin(directionPin) {
        pinMode(mPwmPin, OUTPUT);
        pinMode(mDirectionPin, OUTPUT);
        analogWrite(mPwmPin, 0);
        digitalWrite(mDirectionPin, HIGH);
    }

    void setPWM(int16_t pwm) {
        const int16_t command = constrain(
            pwm, static_cast<int16_t>(-255), static_cast<int16_t>(255));
        const bool directionHigh = command >= 0;

        // Remove PWM before reversing the H-bridge direction.
        if (directionHigh != mDirectionHigh) {
            analogWrite(mPwmPin, 0);
            digitalWrite(mDirectionPin, directionHigh ? HIGH : LOW);
            mDirectionHigh = directionHigh;
        }

        analogWrite(
            mPwmPin,
            static_cast<uint8_t>(command < 0 ? -command : command));
    }

private:
    const uint8_t mPwmPin;
    const uint8_t mDirectionPin;
    bool mDirectionHigh = true;
};
