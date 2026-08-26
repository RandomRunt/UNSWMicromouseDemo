#pragma once

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t dirPin) : pwmPin_(pwmPin), dirPin_(dirPin) {
        pinMode(pwmPin_, OUTPUT);
        pinMode(dirPin_, OUTPUT);
    }

    void setPWM(int16_t pwm) {
        digitalWrite(dirPin_, pwm >= 0 ? HIGH : LOW);
        analogWrite(pwmPin_, constrain(abs(pwm), 0, 255));
    }

    void stop() { setPWM(0); }

private:
    uint8_t pwmPin_;
    uint8_t dirPin_;
};
