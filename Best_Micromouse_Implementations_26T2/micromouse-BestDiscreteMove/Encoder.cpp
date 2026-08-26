#include "Encoder.hpp"

#include <util/atomic.h>

namespace mtrn3100 {

Encoder* Encoder::instances[2] = {nullptr, nullptr};
uint8_t Encoder::instance_count = 0;

Encoder::Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
    pinMode(encoder1_pin, INPUT_PULLUP);
    pinMode(encoder2_pin, INPUT_PULLUP);

    if (instance_count >= 2) return;

    const uint8_t instanceIndex = instance_count++;
    instances[instanceIndex] = this;
    if (instanceIndex == 0) {
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoder1ISR, RISING);
    } else {
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoder2ISR, RISING);
    }
}

void Encoder::onEdge() {
    // AVR enters an ISR with interrupts already disabled. Calling interrupts()
    // here would allow the ISR to nest and eventually corrupt the small stack.
    if (digitalRead(encoder2_pin) == HIGH) {
        ++count;
    } else {
        --count;
    }
}

float Encoder::getRotation() const {
    int32_t countSnapshot;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        countSnapshot = count;
    }

    // Keep slow AVR floating-point work outside the atomic section so Timer0
    // and I2C interrupts are delayed for only the four-byte count copy.
    constexpr float RADIANS_PER_COUNT = TWO_PI / COUNTS_PER_REVOLUTION;
    return static_cast<float>(countSnapshot) * RADIANS_PER_COUNT;
}

void Encoder::readEncoder1ISR() {
    if (instances[0] != nullptr) {
        instances[0]->onEdge();
    }
}

void Encoder::readEncoder2ISR() {
    if (instances[1] != nullptr) {
        instances[1]->onEdge();
    }
}

}  // namespace mtrn3100
