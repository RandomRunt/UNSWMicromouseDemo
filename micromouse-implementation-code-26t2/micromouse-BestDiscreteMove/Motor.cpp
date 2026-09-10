#include "Motor.hpp"

namespace mtrn3100 {

Motor::Motor(uint8_t pwmPin, uint8_t directionPin)
    : pwmPin(pwmPin), directionPin(directionPin) {
    pinMode(pwmPin, OUTPUT);
    pinMode(directionPin, OUTPUT);
    analogWrite(pwmPin, 0);
    digitalWrite(directionPin, LOW);
}

void Motor::setPWM(int16_t pwm) {
    const int16_t command = constrain(pwm, static_cast<int16_t>(-255),
                                     static_cast<int16_t>(255));
    const bool requestedReverse = command < 0;

    // Never change direction while the bridge still has a non-zero duty cycle.
    // The old ordering briefly applied the previous PWM in the new direction,
    // creating an avoidable current spike and motor-brush EMI impulse.
    if (requestedReverse != reverseDirection) {
        analogWrite(pwmPin, 0);
        digitalWrite(directionPin, requestedReverse ? HIGH : LOW);
        reverseDirection = requestedReverse;
    }

    const uint8_t duty = static_cast<uint8_t>(command < 0 ? -command : command);
    analogWrite(pwmPin, duty);
}

}  // namespace mtrn3100
