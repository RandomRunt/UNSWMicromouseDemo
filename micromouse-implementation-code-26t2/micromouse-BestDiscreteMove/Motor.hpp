#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin);
    void setPWM(int16_t pwm);

private:
    const uint8_t pwmPin;
    const uint8_t directionPin;
    bool reverseDirection = false;
};

}  // namespace mtrn3100
