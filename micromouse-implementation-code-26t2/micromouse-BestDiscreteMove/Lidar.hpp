// AI-ASSISTED FILE: Mapping observation interface generated with OpenAI Codex
// (2026-08-11). AI-authored sections are identified inline.
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

// Generative-AI-assisted mapping observation interface (OpenAI Codex,
// 2026-08-11). It preserves the VL6180X status so an open passage cannot be
// confused with a failed I2C transaction.
struct LidarObservation {
    uint16_t distanceMm = 0;
    uint16_t sequence = 0;
    uint8_t rangeStatus = 0xFF;
    bool rangeValid = false;
};

class Lidar : public VL6180X {
public:
    Lidar(uint8_t pin, uint8_t address);
    bool initialise();
    bool startContinuous(uint16_t periodMs);
    bool updateDistance();
    bool hasReading() const;
    bool hasFreshReading(uint32_t maxAgeMs) const;
    bool hasFreshObservation(uint32_t maxAgeMs) const;
    bool getLatestObservation(LidarObservation& observation) const;
    uint32_t getLastReadingTime() const;
    uint16_t getDistance() const;

private:
    bool readRuntimeRegister(uint16_t reg, uint8_t& value);
    bool writeRuntimeRegister(uint16_t reg, uint8_t value);

    const uint8_t mPin;
    const uint8_t mAddress;
    uint16_t mDistance = 0;
    uint32_t mLastReadingTime = 0;
    uint16_t mObservationSequence = 0;
    uint8_t mRangeStatus = 0xFF;
    bool mReady = false;
    bool mHasReading = false;
    bool mHasObservation = false;
};
