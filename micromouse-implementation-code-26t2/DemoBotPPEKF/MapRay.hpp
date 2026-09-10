#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>
#include <math.h>

#include "MazeMap.hpp"

namespace mtrn3100 {

class MapRay {
public:
    static bool expectedRange(
        float robotX,
        float robotY,
        float beamHeading,
        float maximumRange,
        float& range) {
        // Robot frame x points along the initial route convention used by A;
        // the stored maze x axis is mirrored relative to that robot frame.
        const float originX = MAP_ORIGIN_X_M - robotX;
        const float originY = MAP_ORIGIN_Y_M + robotY;
        const float robotDirectionX = cosf(beamHeading);
        const float robotDirectionY = sinf(beamHeading);
        const float directionX = -robotDirectionX;
        const float directionY = robotDirectionY;

        float best = maximumRange;
        bool hit = castGrid(
            originX, originY, directionX, directionY, best);
        hit |= castExtraFaces(
            robotX, robotY, robotDirectionX, robotDirectionY, best);
        if (!hit) return false;
        range = best;
        return true;
    }

private:
    static bool wallBit(const uint8_t* bits, uint16_t index) {
        return (pgm_read_byte(&bits[index >> 3U])
              & static_cast<uint8_t>(1U << (index & 7U))) != 0;
    }

    static bool raySegmentIntersection(
        float originX,
        float originY,
        float directionX,
        float directionY,
        float x1,
        float y1,
        float x2,
        float y2,
        float maximum,
        float& distance) {
        const float segmentX = x2 - x1;
        const float segmentY = y2 - y1;
        const float determinant = directionX * segmentY
                                - directionY * segmentX;
        if (fabsf(determinant) < 1e-9f) return false;
        const float relativeX = x1 - originX;
        const float relativeY = y1 - originY;
        const float alongRay =
            (relativeX * segmentY - relativeY * segmentX) / determinant;
        const float alongSegment =
            (relativeX * directionY - relativeY * directionX) / determinant;
        if (alongRay < 0.0f || alongRay > maximum
            || alongSegment < 0.0f || alongSegment > 1.0f) {
            return false;
        }
        distance = alongRay;
        return true;
    }

    static void considerSegment(
        float originX,
        float originY,
        float directionX,
        float directionY,
        float x1,
        float y1,
        float x2,
        float y2,
        float& best,
        bool& hit) {
        float candidate = best;
        if (raySegmentIntersection(
                originX, originY, directionX, directionY,
                x1, y1, x2, y2, best, candidate)) {
            best = candidate;
            hit = true;
        }
    }

    static bool castGrid(
        float originX,
        float originY,
        float directionX,
        float directionY,
        float& best) {
        bool hit = false;
        const float halfThickness = 0.5f * MAP_WALL_THICKNESS_M;
        for (uint8_t row = 0; row < MAP_POLE_ROWS; ++row) {
            for (uint8_t column = 0; column < MAP_HORIZONTAL_STRIDE; ++column) {
                const uint16_t index = static_cast<uint16_t>(row)
                                     * MAP_HORIZONTAL_STRIDE + column;
                if (!wallBit(MAP_HORIZONTAL_WALLS, index)) continue;
                const float y = static_cast<float>(row) * MAP_CELL_M;
                const float x0 = static_cast<float>(column) * MAP_CELL_M;
                const float x1 = static_cast<float>(column + 1U) * MAP_CELL_M;
                considerSegment(originX, originY, directionX, directionY,
                    x0, y - halfThickness, x1, y - halfThickness, best, hit);
                considerSegment(originX, originY, directionX, directionY,
                    x0, y + halfThickness, x1, y + halfThickness, best, hit);
            }
        }

        for (uint8_t column = 0; column < MAP_POLE_COLUMNS; ++column) {
            for (uint8_t row = 0; row < MAP_VERTICAL_STRIDE; ++row) {
                const uint16_t index = static_cast<uint16_t>(column)
                                     * MAP_VERTICAL_STRIDE + row;
                if (!wallBit(MAP_VERTICAL_WALLS, index)) continue;
                const float x = static_cast<float>(column) * MAP_CELL_M;
                const float y0 = static_cast<float>(row) * MAP_CELL_M;
                const float y1 = static_cast<float>(row + 1U) * MAP_CELL_M;
                considerSegment(originX, originY, directionX, directionY,
                    x - halfThickness, y0, x - halfThickness, y1, best, hit);
                considerSegment(originX, originY, directionX, directionY,
                    x + halfThickness, y0, x + halfThickness, y1, best, hit);
            }
        }
        return hit;
    }

    static bool castExtraFaces(
        float originX,
        float originY,
        float directionX,
        float directionY,
        float& best) {
        bool hit = false;
        for (size_t i = 0; i < MAP_EXTRA_WALL_FACE_COUNT; ++i) {
            LineSegment segment;
            memcpy_P(
                &segment,
                &MAP_EXTRA_WALL_FACES[i],
                sizeof(LineSegment));
            considerSegment(
                originX, originY, directionX, directionY,
                segment.x1, segment.y1, segment.x2, segment.y2,
                best, hit);
        }
        return hit;
    }
};

}  // namespace mtrn3100
