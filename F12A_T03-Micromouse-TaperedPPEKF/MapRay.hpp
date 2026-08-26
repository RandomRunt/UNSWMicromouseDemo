#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <math.h>

#include "walls_planned.hpp"

struct MapRay {
  static constexpr float kMaxRangeM = 0.30f;

  static LineSeg2D readSegProgmem(const LineSeg2D* segs, size_t i) {
    LineSeg2D s;
    memcpy_P(&s, &segs[i], sizeof(LineSeg2D));
    return s;
  }

  static bool bitProgmem(const uint8_t* bits, uint16_t index) {
    const uint8_t byte = pgm_read_byte(&bits[index >> 3]);
    return (byte & (uint8_t)(1u << (index & 7u))) != 0;
  }

  static bool raySegmentHit(
      float ox,
      float oy,
      float dx,
      float dy,
      float x1,
      float y1,
      float x2,
      float y2,
      float tMax,
      float& tHit) {
    const float vx = x2 - x1;
    const float vy = y2 - y1;
    const float det = dx * vy - dy * vx;
    if (fabsf(det) < 1e-9f) {
      return false;
    }
    const float rx = x1 - ox;
    const float ry = y1 - oy;
    const float t = (rx * vy - ry * vx) / det;
    if (t < 0.0f || t > tMax) {
      return false;
    }
    const float u = (rx * dy - ry * dx) / det;
    if (u < 0.0f || u > 1.0f) {
      return false;
    }
    tHit = t;
    return true;
  }

  static void considerSeg(
      float ox,
      float oy,
      float dx,
      float dy,
      float x1,
      float y1,
      float x2,
      float y2,
      float& bestT,
      bool& hit) {
    float t = bestT;
    if (raySegmentHit(ox, oy, dx, dy, x1, y1, x2, y2, bestT, t)) {
      bestT = t;
      hit = true;
    }
  }

  static bool castSegmentsProgmem(
      float ox,
      float oy,
      float dx,
      float dy,
      const LineSeg2D* segs,
      size_t len,
      float& bestT) {
    bool hit = false;
    for (size_t i = 0; i < len; ++i) {
      const LineSeg2D s = readSegProgmem(segs, i);
      considerSeg(ox, oy, dx, dy, s.x1, s.y1, s.x2, s.y2, bestT, hit);
    }
    return hit;
  }

  // Horizontal wall faces (±th/2) at pole-row pr, edge west-col pc.
  static void castHWall(
      float ox,
      float oy,
      float dx,
      float dy,
      uint8_t pr,
      uint8_t pc,
      float& bestT,
      bool& hit) {
    const float y0 = (float)pr * MAP_CELL_M;
    const float x0 = (float)pc * MAP_CELL_M;
    const float x1 = (float)(pc + 1u) * MAP_CELL_M;
    const float half = 0.5f * MAP_WALL_TH_M;
    considerSeg(ox, oy, dx, dy, x0, y0 - half, x1, y0 - half, bestT, hit);
    considerSeg(ox, oy, dx, dy, x0, y0 + half, x1, y0 + half, bestT, hit);
  }

  // Vertical wall faces (±th/2) at pole-col pc, edge north-row pr.
  static void castVWall(
      float ox,
      float oy,
      float dx,
      float dy,
      uint8_t pc,
      uint8_t pr,
      float& bestT,
      bool& hit) {
    const float x0 = (float)pc * MAP_CELL_M;
    const float y0 = (float)pr * MAP_CELL_M;
    const float y1 = (float)(pr + 1u) * MAP_CELL_M;
    const float half = 0.5f * MAP_WALL_TH_M;
    considerSeg(ox, oy, dx, dy, x0 - half, y0, x0 - half, y1, bestT, hit);
    considerSeg(ox, oy, dx, dy, x0 + half, y0, x0 + half, y1, bestT, hit);
  }

  static void castPoleSquare(
      float ox,
      float oy,
      float dx,
      float dy,
      uint8_t i,
      uint8_t j,
      float& bestT,
      bool& hit) {
    const float cx = (float)i * MAP_CELL_M;
    const float cy = (float)j * MAP_CELL_M;
    const float h = 0.5f * MAP_POST_M;
    const float x0 = cx - h;
    const float x1 = cx + h;
    const float y0 = cy - h;
    const float y1 = cy + h;
    considerSeg(ox, oy, dx, dy, x0, y0, x1, y0, bestT, hit);
    considerSeg(ox, oy, dx, dy, x1, y0, x1, y1, bestT, hit);
    considerSeg(ox, oy, dx, dy, x1, y1, x0, y1, bestT, hit);
    considerSeg(ox, oy, dx, dy, x0, y1, x0, y0, bestT, hit);
  }

  static bool castGridMaze(
      float ox,
      float oy,
      float dx,
      float dy,
      float& bestT,
      bool includePoles) {
    bool hit = false;

    for (uint8_t pr = 0; pr < MAP_POLE_ROWS; ++pr) {
      for (uint8_t pc = 0; pc < MAP_H_STRIDE; ++pc) {
        const uint16_t idx = (uint16_t)pr * (uint16_t)MAP_H_STRIDE + (uint16_t)pc;
        if (bitProgmem(MAP_H_WALLS, idx)) {
          castHWall(ox, oy, dx, dy, pr, pc, bestT, hit);
        }
      }
    }

    for (uint8_t pc = 0; pc < MAP_POLE_COLS; ++pc) {
      for (uint8_t pr = 0; pr < MAP_V_STRIDE; ++pr) {
        const uint16_t idx = (uint16_t)pc * (uint16_t)MAP_V_STRIDE + (uint16_t)pr;
        if (bitProgmem(MAP_V_WALLS, idx)) {
          castVWall(ox, oy, dx, dy, pc, pr, bestT, hit);
        }
      }
    }

    // Poles are thin and ToF often misses / glance-hits them → non-deterministic zHat.
    if (includePoles) {
      for (uint8_t j = 0; j < MAP_POLE_ROWS; ++j) {
        for (uint8_t i = 0; i < MAP_POLE_COLS; ++i) {
          const uint16_t idx = (uint16_t)j * (uint16_t)MAP_POLE_COLS + (uint16_t)i;
          if (bitProgmem(MAP_POLES, idx)) {
            castPoleSquare(ox, oy, dx, dy, i, j, bestT, hit);
          }
        }
      }
    }

    return hit;
  }

  // Expected wall range for localisation. Poles off by default (unstable vs ToF).
  static bool expectedRangeM(
      float oxRobot,
      float oyRobot,
      float beamYawRad,
      float maxRangeM,
      float& rangeM,
      bool includePoles = false) {
    // Robot (x left, y down) → maze (x right, y down).
    const float ox = MAP_ORIGIN_X_M - oxRobot;
    const float oy = MAP_ORIGIN_Y_M + oyRobot;
    const float dxR = cosf(beamYawRad);
    const float dyR = sinf(beamYawRad);
    const float dx = -dxR;
    const float dy = dyR;

    float bestT = maxRangeM;
    bool hit = castGridMaze(ox, oy, dx, dy, bestT, includePoles);

    hit |= castSegmentsProgmem(
        oxRobot, oyRobot, dxR, dyR, MAP_LEGACY_FACES, MAP_LEGACY_FACES_LEN, bestT);

    if (!hit) {
      return false;
    }
    rangeM = bestT;
    return true;
  }
};
