// AI-ASSISTED FILE: Framebuffer-free Task 4.3 renderer generated with OpenAI
// Codex (2026-08-11). AI-authored sections are identified inline.
#pragma once

#include <Arduino.h>

#include "Maze.hpp"
#include "MotionTypes.hpp"

namespace mtrn3100 {

// Small, framebuffer-free SSD1306 display driver. It draws one 16-pixel-high
// lidar row at a time, so the Nano never pays the 1 KB SRAM cost of a bitmap
// framebuffer.
class MicromouseDisplay {
public:
    bool initialise();
    bool isReady() const;
    void showReading(uint8_t sensorIndex, uint16_t distanceMm, bool valid);
    void showMap(
        const Maze& maze,
        const Pose& pose,
        const Pose& start,
        uint8_t goalRow,
        uint8_t goalColumn,
        MappingPhase phase);

private:
    bool writeCommands(const uint8_t* commands, uint8_t count);
    bool writeFlashCommands(const uint8_t* commands, uint8_t count);
    bool writeData(const uint8_t* data, uint8_t count);
    bool setPageAndColumn(uint8_t page, uint8_t column);
    bool clear();
    void disable();

    bool mReady = false;
};

}  // namespace mtrn3100
