#pragma once

#include <Arduino.h>
#include <avr/pgmspace.h>

namespace mtrn3100 {

struct Waypoint {
    float x;
    float y;
};

// Same basic command syntax as DemoBotDiscreteMove:
//   f = advance one maze cell
//   l = turn the planned direction 90 degrees left
//   r = turn the planned direction 90 degrees right
//
// The default string reproduces A's original checked-in waypoint route. It is
// stored in flash so editing it does not use additional SRAM.
// const char ROUTE_COMMANDS[] PROGMEM = "flflfrfrflflfrfflfrflflfflfrfrflflfrfrffrfffff"; // Left side of maze
const char ROUTE_COMMANDS[] PROGMEM = "frflfrflfrfrflflfrfrflffrflfrfrffrflflfrfrflffrf"; // Right side of maze

constexpr float CELL_SIZE_M = 0.180f;
constexpr float INITIAL_HEADING_RAD = -1.5708f;
constexpr size_t MAX_ROUTE_WAYPOINTS = 48;

// Filled once in setup(). Waypoint zero is the starting position and each 'f'
// appends one more waypoint.
Waypoint WAYPOINTS[MAX_ROUTE_WAYPOINTS];
size_t WAYPOINT_COUNT = 0;

constexpr int16_t CYLINDER_SEGMENT_FIRST = -1;
constexpr int16_t CYLINDER_SEGMENT_LAST = -1;

// If ROUTE_COMMANDS changes, update these segments or disable front-LiDAR EKF
// correction until the route and MazeMap.hpp are consistent.
const uint8_t FRONT_WALL_CORRECTION_SEGMENTS[] = {0, 1, 3};
constexpr size_t FRONT_WALL_CORRECTION_SEGMENT_COUNT =
    sizeof(FRONT_WALL_CORRECTION_SEGMENTS)
    / sizeof(FRONT_WALL_CORRECTION_SEGMENTS[0]);

inline float wrapRouteHeading(float heading) {
    while (heading > PI) heading -= 2.0f * PI;
    while (heading <= -PI) heading += 2.0f * PI;
    return heading;
}

inline bool buildWaypointRoute() {
    WAYPOINT_COUNT = 1;
    WAYPOINTS[0] = {0.0f, 0.0f};

    float x = 0.0f;
    float y = 0.0f;
    float heading = INITIAL_HEADING_RAD;

    for (size_t i = 0; i < sizeof(ROUTE_COMMANDS); ++i) {
        char command = static_cast<char>(pgm_read_byte(&ROUTE_COMMANDS[i]));
        if (command == '\0') break;

        if (command >= 'A' && command <= 'Z') {
            command = static_cast<char>(command - 'A' + 'a');
        }

        if (command == 'l') {
            heading = wrapRouteHeading(heading + PI * 0.5f);
        } else if (command == 'r') {
            heading = wrapRouteHeading(heading - PI * 0.5f);
        } else if (command == 'f') {
            if (WAYPOINT_COUNT >= MAX_ROUTE_WAYPOINTS) {
                WAYPOINT_COUNT = 0;
                return false;
            }
            x += CELL_SIZE_M * cosf(heading);
            y += CELL_SIZE_M * sinf(heading);
            WAYPOINTS[WAYPOINT_COUNT++] = {x, y};
        } else if (command != ' ' && command != '\t'
                   && command != '\r' && command != '\n') {
            WAYPOINT_COUNT = 0;
            return false;
        }
    }

    // Pure pursuit needs a start and at least one destination.
    if (WAYPOINT_COUNT < 2) {
        WAYPOINT_COUNT = 0;
        return false;
    }
    return true;
}

}  // namespace mtrn3100
