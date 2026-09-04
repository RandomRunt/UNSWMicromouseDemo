#pragma once

// Generative-AI-assisted implementation (OpenAI Codex, 2026-08-11).
// Dimensions are transcribed from dimensions.txt. They describe the robot,
// not a particular maze layout, so Task 4.3 still hard-codes only start/goal.

namespace mtrn3100 {
namespace geometry {

constexpr float CELL_SIZE_MM = 180.0f;
constexpr float CHASSIS_WIDTH_MM = 75.0f;
constexpr float CHASSIS_LENGTH_MM = 80.0f;

// Front lidar is at the front centre. Side lidars are on the side edges and
// 20 mm ahead of the chassis centre.
constexpr float FRONT_LIDAR_FORWARD_MM = CHASSIS_LENGTH_MM * 0.5f;
constexpr float SIDE_LIDAR_LATERAL_MM = CHASSIS_WIDTH_MM * 0.5f;
constexpr float SIDE_LIDAR_FORWARD_MM = 20.0f;

// Expected readings when the chassis is centred in a 180 mm cell.
constexpr float FRONT_WALL_CENTRED_MM = CELL_SIZE_MM * 0.5f
                                      - FRONT_LIDAR_FORWARD_MM;
constexpr float SIDE_WALL_CENTRED_MM = CELL_SIZE_MM * 0.5f
                                     - SIDE_LIDAR_LATERAL_MM;
constexpr float FRONT_WALL_ONE_CELL_AWAY_MM = FRONT_WALL_CENTRED_MM
                                            + CELL_SIZE_MM;
constexpr float SIDE_WALL_ONE_CELL_AWAY_MM = SIDE_WALL_CENTRED_MM
                                           + CELL_SIZE_MM;

// Conservative classification bands. The 30 mm grey band prevents mounting
// error and angled readings from being promoted directly to an open edge.
// These are starting values and must be checked against recorded sensor data.
constexpr float FRONT_WALL_MAX_MM = 100.0f;
constexpr float SIDE_WALL_MAX_MM = 100.0f;
constexpr float OPEN_MIN_MM = 130.0f;

}  // namespace geometry
}  // namespace mtrn3100
