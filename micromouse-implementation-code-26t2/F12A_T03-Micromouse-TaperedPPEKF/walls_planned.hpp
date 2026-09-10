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
static constexpr float MAP_ORIGIN_X_M = 0.6300f;
static constexpr float MAP_ORIGIN_Y_M = 1.3500f;

struct LineSeg2D { float x1; float y1; float x2; float y2; };

const uint8_t MAP_H_WALLS[] PROGMEM = {
  0x7Cu, 0x04u, 0x0Du, 0x04u, 0x20u, 0x88u, 0x2Fu, 0xAFu, 0xB0u, 0x92u, 0x88u, 0x00u
};
const size_t MAP_H_WALLS_LEN = sizeof(MAP_H_WALLS) / sizeof(MAP_H_WALLS[0]);

const uint8_t MAP_V_WALLS[] PROGMEM = {
  0x30u, 0xF4u, 0x5Du, 0x05u, 0x0Cu, 0x0Cu, 0x00u, 0xE0u, 0xBEu, 0x82u, 0xF8u, 0x00u
};
const size_t MAP_V_WALLS_LEN = sizeof(MAP_V_WALLS) / sizeof(MAP_V_WALLS[0]);

const uint8_t MAP_POLES[] PROGMEM = {
  0xFCu, 0x18u, 0x76u, 0xF8u, 0xE1u, 0x87u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFEu, 0xF1u, 
  0x03u
};
const size_t MAP_POLES_LEN = sizeof(MAP_POLES) / sizeof(MAP_POLES[0]);

const LineSeg2D MAP_LEGACY_FACES[] PROGMEM = {
  {0.7300f, -1.0314f, 0.3114f, -1.4500f},
  {0.7450f, -1.0659f, 0.3459f, -1.4650f},
  {-0.6714f, -1.4500f, -1.0900f, -1.0314f},
  {-0.7059f, -1.4650f, -1.1050f, -1.0659f},
  {-1.0900f, -0.0486f, -0.6714f, 0.3700f},
  {-1.1050f, -0.0141f, -0.7059f, 0.3850f},
  {0.3114f, 0.3700f, 0.7300f, -0.0486f},
  {0.3459f, 0.3850f, 0.7450f, -0.0141f},
  {0.3114f, -1.4500f, -0.6714f, -1.4500f},
  {0.3459f, -1.4650f, -0.7059f, -1.4650f},
  {-1.0900f, -1.0314f, -1.0900f, -0.0486f},
  {-1.1050f, -1.0659f, -1.1050f, -0.0141f},
  {-0.6714f, 0.3700f, 0.3114f, 0.3700f},
  {-0.7059f, 0.3850f, 0.3459f, 0.3850f},
  {0.7300f, -0.0486f, 0.7300f, -1.0314f},
  {0.7450f, -0.0141f, 0.7450f, -1.0659f},
};
const size_t MAP_LEGACY_FACES_LEN = sizeof(MAP_LEGACY_FACES) / sizeof(MAP_LEGACY_FACES[0]);
