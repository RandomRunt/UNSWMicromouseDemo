#pragma once

#define INITIAL_HEADING_RAD (-1.5708f)
#define CYLINDER_SEG_LO (-1)
#define CYLINDER_SEG_HI (-1)

const uint8_t FRONT_WALL_SEGS[] = {
  0, 1, 3,
};
const size_t FRONT_WALL_SEG_COUNT = sizeof(FRONT_WALL_SEGS) / sizeof(FRONT_WALL_SEGS[0]);

const Waypoint2D WAYPOINTS[] = {
  {-0.0000f, 0.0000f},
  {-0.0000f, -0.1800f},
  {0.1800f, -0.1800f},
  {0.1800f, 0.0000f},
  {0.3600f, 0.0000f},
  {0.3600f, -0.1800f},
  {0.3600f, -0.3600f},
  {0.1800f, -0.3600f},
  {-0.0000f, -0.3600f},
  {-0.1800f, -0.3600f},
};
const size_t WAYPOINT_LEN = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
