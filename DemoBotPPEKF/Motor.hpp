#pragma once

#include <Arduino.h>
#include <digitalWriteFast.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin)
        : pwmPin_(pwmPin), directionPin_(directionPin) {
        pinModeFast(pwmPin_, OUTPUT);
        pinModeFast(directionPin_, OUTPUT);
        setPWM(0);
    }

    void setPWM(int16_t pwm) {
        const bool forwardPolarity = pwm >= 0;
        int16_t magnitude = pwm < 0 ? -pwm : pwm;
        if (magnitude > 255) magnitude = 255;
        digitalWriteFast(directionPin_, forwardPolarity ? HIGH : LOW);
        analogWrite(pwmPin_, magnitude);
    }

    void stop() { setPWM(0); }

private:
    const uint8_t pwmPin_;
    const uint8_t directionPin_;
};

}  // namespace mtrn3100
