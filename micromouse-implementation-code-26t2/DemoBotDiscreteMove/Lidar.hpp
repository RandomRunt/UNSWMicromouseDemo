#pragma once

#include <Arduino.h>
#include <VL6180X.h>

// Non-blocking VL6180X wrapper. Only completed, status-valid measurements are
// exposed to movement control.
class Lidar : private VL6180X {
public:
    Lidar(uint8_t xshutPin, uint8_t address);
    bool initialise();
    bool startContinuous(uint16_t periodMs);
    bool updateDistance();
    bool hasFreshReading(uint32_t maxAgeMs) const;
    uint16_t getDistance() const;
    uint16_t getSequence() const;

private:
    bool readRegisterChecked(uint16_t reg, uint8_t& value);
    bool writeRegisterChecked(uint16_t reg, uint8_t value);

    const uint8_t mXshutPin;
    const uint8_t mAddress;
    uint16_t mDistanceMm = 0;
    uint32_t mUpdatedAtMs = 0;
    uint16_t mSequence = 0;
    bool mReady = false;
    bool mReadingValid = false;
};
