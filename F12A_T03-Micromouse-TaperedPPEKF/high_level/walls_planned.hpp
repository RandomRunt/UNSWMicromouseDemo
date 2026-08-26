#pragma once

#include <Arduino.h>

// Orthogonal maze: wall/pole presence bitsets + fixed geometry.
// Maze frame: x right, y down, origin at pole (0,0). Robot origin = start cell centre.
// H index = poleRow * MAP_H_STRIDE + westPoleCol
// V index = poleCol * MAP_V_STRIDE + northPoleRow
// Pole index = poleRow * MAP_POLE_COLS + poleCol
// MAP_LEGACY_FACES: non-grid obstacles in robot frame (outer shell, diagonals, extras).
static constexpr float MAP_CELL_M = 0.1800f;
static constexpr float MAP_WALL_TH_M = 0.0030f;
static constexpr float MAP_POST_M = 0.0080f;
static constexpr uint8_t MAP_POLE_COLS = 10;
static constexpr uint8_t MAP_POLE_ROWS = 10;
static constexpr uint8_t MAP_H_STRIDE = 9;
static constexpr uint8_t MAP_V_STRIDE = 9;
static constexpr uint16_t MAP_H_BITS = 90;
static constexpr uint16_t MAP_V_BITS = 90;
static constexpr uint16_t MAP_POLE_BITS = 100;
static constexpr float MAP_ORIGIN_X_M = 1.5300f;
static constexpr float MAP_ORIGIN_Y_M = 0.4500f;

struct LineSeg2D { float x1; float y1; float x2; float y2; };

const uint8_t MAP_H_WALLS[] PROGMEM = {
  0x7Cu, 0x74u, 0x15u, 0x34u, 0x10u, 0xA0u, 0xC0u, 0x9Eu, 0x9Au, 0x82u, 0xF8u, 0x00u
};
const size_t MAP_H_WALLS_LEN = sizeof(MAP_H_WALLS) / sizeof(MAP_H_WALLS[0]);

const uint8_t MAP_V_WALLS[] PROGMEM = {
  0x7Cu, 0x04u, 0x25u, 0xB4u, 0x07u, 0x08u, 0x18u, 0xE0u, 0xA0u, 0xBEu, 0xF8u, 0x00u
};
const size_t MAP_V_WALLS_LEN = sizeof(MAP_V_WALLS) / sizeof(MAP_V_WALLS[0]);

const uint8_t MAP_POLES[] PROGMEM = {
  0xFCu, 0xF8u, 0xF7u, 0xF0u, 0xC3u, 0x0Fu, 0x3Fu, 0xFCu, 0xFFu, 0xFFu, 0xFEu, 0xF1u, 
  0x03u
};
const size_t MAP_POLES_LEN = sizeof(MAP_POLES) / sizeof(MAP_POLES[0]);

const LineSeg2D MAP_LEGACY_FACES[] PROGMEM = {
  {1.6300f, -0.1314f, 1.2114f, -0.5500f},
  {1.6450f, -0.1659f, 1.2459f, -0.5650f},
  {0.2286f, -0.5500f, -0.1900f, -0.1314f},
  {0.1941f, -0.5650f, -0.2050f, -0.1659f},
  {-0.1900f, 0.8514f, 0.2286f, 1.2700f},
  {-0.2050f, 0.8859f, 0.1941f, 1.2850f},
  {1.2114f, 1.2700f, 1.6300f, 0.8514f},
  {1.2459f, 1.2850f, 1.6450f, 0.8859f},
  {1.2114f, -0.5500f, 0.2286f, -0.5500f},
  {1.2459f, -0.5650f, 0.1941f, -0.5650f},
  {-0.1900f, -0.1314f, -0.1900f, 0.8514f},
  {-0.2050f, -0.1659f, -0.2050f, 0.8859f},
  {0.2286f, 1.2700f, 1.2114f, 1.2700f},
  {0.1941f, 1.2850f, 1.2459f, 1.2850f},
  {1.6300f, 0.8514f, 1.6300f, -0.1314f},
  {1.6450f, 0.8859f, 1.6450f, -0.1659f},
};
const size_t MAP_LEGACY_FACES_LEN = sizeof(MAP_LEGACY_FACES) / sizeof(MAP_LEGACY_FACES[0]);
