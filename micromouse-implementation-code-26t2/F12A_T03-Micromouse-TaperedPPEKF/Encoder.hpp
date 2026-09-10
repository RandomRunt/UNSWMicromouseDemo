#pragma once

#include <Arduino.h>

class Encoder {
public:
    Encoder(uint8_t pinA, uint8_t pinB, uint16_t countsPerRev = 700)
        : countsPerRev_(countsPerRev) {
        pinMode(pinA, INPUT_PULLUP);
        pinMode(pinB, INPUT_PULLUP);

        uint8_t irq = digitalPinToInterrupt(pinA);
        if (irq > 1) {
            return;
        }

        counts_[irq] = &count_;
        portIn_[irq] = portInputRegister(digitalPinToPort(pinB));
        bitMask_[irq] = digitalPinToBitMask(pinB);

        if (irq == 0) {
            attachInterrupt(irq, isr0, RISING);
        } else {
            attachInterrupt(irq, isr1, RISING);
        }
    }

    long getCount() const {
        noInterrupts();
        long c = count_;
        interrupts();
        return c;
    }

    float getRadians() const {
        return (2.0f * PI * getCount()) / countsPerRev_;
    }

    void reset() {
        noInterrupts();
        count_ = 0;
        interrupts();
    }

private:
    static void isr0() {
        if (*portIn_[0] & bitMask_[0]) {
            (*counts_[0])++;
        } else {
            (*counts_[0])--;
        }
    }

    static void isr1() {
        if (*portIn_[1] & bitMask_[1]) {
            (*counts_[1])++;
        } else {
            (*counts_[1])--;
        }
    }

    static volatile long* counts_[2];
    static volatile uint8_t* portIn_[2];
    static uint8_t bitMask_[2];

    uint16_t countsPerRev_;
    volatile long count_ = 0;
};

volatile long* Encoder::counts_[2] = {nullptr, nullptr};
volatile uint8_t* Encoder::portIn_[2] = {nullptr, nullptr};
uint8_t Encoder::bitMask_[2] = {0, 0};
