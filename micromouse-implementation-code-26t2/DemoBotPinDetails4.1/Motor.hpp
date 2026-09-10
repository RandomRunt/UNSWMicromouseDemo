#pragma once

#include <digitalWriteFast.h>
#include <Arduino.h>
#include <math.h>

namespace mtrn3100 {

class Motor {
  public:
    Motor(uint8_t pwm_pin, uint8_t in2)
      : pwm_pin(pwm_pin), dir_pin(in2) {
      pinModeFast(pwm_pin, OUTPUT);
      pinModeFast(dir_pin, OUTPUT);


      digitalWriteFast(dir_pin, LOW);
      analogWrite(pwm_pin, 0);
    }


    // This function outputs the desired motor direction and the PWM signal.
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {
      bool direction = true;
      int output = pwm;

      if (pwm < 0) {
        direction = false;
        output = -pwm;
      }

      if (output > 255) output = 255;

      digitalWriteFast(dir_pin, direction ? HIGH : LOW);
      analogWrite(pwm_pin, output);
    }

  private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
  };
}
