#pragma once

#include <Arduino.h>

namespace config {

// DemoBot-Task4.1V2 wiring.
constexpr uint8_t LEFT_MOTOR_PWM_PIN = 11;
constexpr uint8_t LEFT_MOTOR_DIR_PIN = 12;
constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 9;
constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 10;
constexpr uint8_t LEFT_ENCODER_A_PIN = 2;
constexpr uint8_t LEFT_ENCODER_B_PIN = 7;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 3;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 8;

constexpr uint8_t LEFT_LIDAR_XSHUT_PIN = A0;
constexpr uint8_t FRONT_LIDAR_XSHUT_PIN = A1;
constexpr uint8_t RIGHT_LIDAR_XSHUT_PIN = A2;
constexpr uint8_t LEFT_LIDAR_ADDRESS = 0x54;
constexpr uint8_t FRONT_LIDAR_ADDRESS = 0x56;
constexpr uint8_t RIGHT_LIDAR_ADDRESS = 0x58;

// DemoBot geometry and encoder conventions.
constexpr float WHEEL_RADIUS_M = 0.0161f;
constexpr float WHEEL_BASE_M = 0.075f; // Most Demo Bots 0.075f; 0.071f for grey;
constexpr uint16_t ENCODER_COUNTS_PER_REVOLUTION = 700;
constexpr int8_t LEFT_ENCODER_FORWARD_SIGN = -1;
constexpr int8_t RIGHT_ENCODER_FORWARD_SIGN = +1;
constexpr int8_t LEFT_MOTOR_FORWARD_SIGN = -1;
constexpr int8_t RIGHT_MOTOR_FORWARD_SIGN = +1;

// B's motor limits, used around A's wheel-velocity controller.
constexpr int16_t PWM_MAX = 250;
constexpr int16_t PWM_MIN = 15;
constexpr int16_t PWM_THRESHOLD = 30;
constexpr int16_t PWM_MAX_STEP = 50;

// A's tapered pure-pursuit parameters, with B's 90 mm wheelbase above. [ORIGINAL]
constexpr float PP_LOOKAHEAD_M = 0.055f;
constexpr float PP_SPEED_LOOKAHEAD_M = 0.200f;
constexpr float PP_CRUISE_SPEED_MPS = 0.300f;
constexpr float PP_CYLINDER_SPEED_MPS = 0.200f;
constexpr float PP_MAX_OMEGA_RAD_S = 9.0f;
constexpr float PP_TURN_SLOW = 0.300f;

// // A's tapered pure-pursuit parameters, with B's 90 mm wheelbase above.
// constexpr float PP_LOOKAHEAD_M = 0.085f; // was 0.055
// constexpr float PP_SPEED_LOOKAHEAD_M = 0.200f;
// constexpr float PP_CRUISE_SPEED_MPS = 0.400f; // was 0.300f
// constexpr float PP_CYLINDER_SPEED_MPS = 0.200f;
// constexpr float PP_MAX_OMEGA_RAD_S = 9.0f;
// constexpr float PP_TURN_SLOW = 0.200f; // was 0.300f

constexpr float VELOCITY_KP = 30.0f;
constexpr float VELOCITY_KI = 1.0f;
constexpr float VELOCITY_KD = 1.0f;
constexpr float PWM_PER_MPS = 700.0f;
constexpr float VELOCITY_FILTER_ALPHA = 0.20f; // was 0.20f - increasing this reduces encoder-velocity delay at turn exit

constexpr uint32_t CONTROL_PERIOD_US = 5000;
constexpr uint16_t LIDAR_RANGE_PERIOD_MS = 50;
constexpr uint16_t LIDAR_POLL_SLOT_MS = 5;
constexpr uint32_t I2C_CLOCK_HZ = 100000UL;
constexpr uint32_t I2C_TIMEOUT_US = 5000UL;

// B expects about 54 mm from its front sensor at a centred 180 mm cell.
// That implies a roughly 36 mm sensor offset from the robot centre. Measure
// this on the actual chassis before enabling map-based range correction.
constexpr float FRONT_LIDAR_MOUNT_M = 0.036f;

constexpr bool MOTORS_ENABLED = true;
constexpr bool IMU_ENABLED = true;
constexpr bool FRONT_LIDAR_EKF_ENABLED = true;
constexpr bool SERIAL_TELEMETRY_ENABLED = false;

}  // namespace config
