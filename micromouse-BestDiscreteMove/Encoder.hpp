#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace mtrn3100 {

// Counts one encoder edge per interrupt. The ISR deliberately does the minimum
// possible work; callers receive an atomic snapshot of the count.
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2);
    float getRotation() const;

private:
    void onEdge();
    static void readEncoder1ISR();
    static void readEncoder2ISR();

    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int32_t count = 0;

    static constexpr uint16_t COUNTS_PER_REVOLUTION = 690;
    static Encoder* instances[2];
    static uint8_t instance_count;
};

}  // namespace mtrn3100
