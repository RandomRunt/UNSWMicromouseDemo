#pragma once

#include <Arduino.h>

namespace mtrn3100 {

// A's known Task 4.1 maze, used only to predict front-ToF measurements.
constexpr float MAP_CELL_M = 0.1800f;
constexpr float MAP_WALL_THICKNESS_M = 0.0030f;
constexpr float MAP_POST_M = 0.0080f;
constexpr uint8_t MAP_POLE_COLUMNS = 10;
constexpr uint8_t MAP_POLE_ROWS = 10;
constexpr uint8_t MAP_HORIZONTAL_STRIDE = 9;
constexpr uint8_t MAP_VERTICAL_STRIDE = 9;
constexpr float MAP_ORIGIN_X_M = 0.6300f;
constexpr float MAP_ORIGIN_Y_M = 1.3500f;

struct LineSegment {
    float x1;
    float y1;
    float x2;
    float y2;
};

const uint8_t MAP_HORIZONTAL_WALLS[] PROGMEM = {
    0x7C, 0x04, 0x0D, 0x04, 0x20, 0x88,
    0x2F, 0xAF, 0xB0, 0x92, 0x88, 0x00,
};

const uint8_t MAP_VERTICAL_WALLS[] PROGMEM = {
    0x30, 0xF4, 0x5D, 0x05, 0x0C, 0x0C,
    0x00, 0xE0, 0xBE, 0x82, 0xF8, 0x00,
};

const LineSegment MAP_EXTRA_WALL_FACES[] PROGMEM = {
    { 0.7300f, -1.0314f,  0.3114f, -1.4500f},
    { 0.7450f, -1.0659f,  0.3459f, -1.4650f},
    {-0.6714f, -1.4500f, -1.0900f, -1.0314f},
    {-0.7059f, -1.4650f, -1.1050f, -1.0659f},
    {-1.0900f, -0.0486f, -0.6714f,  0.3700f},
    {-1.1050f, -0.0141f, -0.7059f,  0.3850f},
    { 0.3114f,  0.3700f,  0.7300f, -0.0486f},
    { 0.3459f,  0.3850f,  0.7450f, -0.0141f},
    { 0.3114f, -1.4500f, -0.6714f, -1.4500f},
    { 0.3459f, -1.4650f, -0.7059f, -1.4650f},
    {-1.0900f, -1.0314f, -1.0900f, -0.0486f},
    {-1.1050f, -1.0659f, -1.1050f, -0.0141f},
    {-0.6714f,  0.3700f,  0.3114f,  0.3700f},
    {-0.7059f,  0.3850f,  0.3459f,  0.3850f},
    { 0.7300f, -0.0486f,  0.7300f, -1.0314f},
    { 0.7450f, -0.0141f,  0.7450f, -1.0659f},
};

constexpr size_t MAP_EXTRA_WALL_FACE_COUNT =
    sizeof(MAP_EXTRA_WALL_FACES) / sizeof(MAP_EXTRA_WALL_FACES[0]);

}  // namespace mtrn3100
