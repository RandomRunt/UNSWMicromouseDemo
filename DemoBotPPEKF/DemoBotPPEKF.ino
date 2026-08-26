#include <Wire.h>

#include "Config.hpp"
#include "DualEncoder.hpp"
#include "Imu.hpp"
#include "Lidar.hpp"
#include "Model.hpp"
#include "Motor.hpp"
#include "PurePursuit.hpp"
#include "Route.hpp"
#include "StateEstimator.hpp"
#include "VelocityController.hpp"

using namespace mtrn3100;

Motor leftMotor(config::LEFT_MOTOR_PWM_PIN, config::LEFT_MOTOR_DIR_PIN);
Motor rightMotor(config::RIGHT_MOTOR_PWM_PIN, config::RIGHT_MOTOR_DIR_PIN);
DualEncoder encoders(
    config::LEFT_ENCODER_A_PIN,
    config::LEFT_ENCODER_B_PIN,
    config::RIGHT_ENCODER_A_PIN,
    config::RIGHT_ENCODER_B_PIN);
Imu imu;
Lidar leftLidar(
    config::LEFT_LIDAR_XSHUT_PIN,
    config::LEFT_LIDAR_ADDRESS);
Lidar frontLidar(
    config::FRONT_LIDAR_XSHUT_PIN,
    config::FRONT_LIDAR_ADDRESS);
Lidar rightLidar(
    config::RIGHT_LIDAR_XSHUT_PIN,
    config::RIGHT_LIDAR_ADDRESS);
Model motionModel(config::WHEEL_BASE_M);
StateEstimator estimator(
    motionModel,
    encoders,
    config::WHEEL_RADIUS_M,
    config::IMU_ENABLED ? &imu : nullptr);
PurePursuit pursuit(
    config::WHEEL_BASE_M,
    config::PP_LOOKAHEAD_M,
    config::PP_CRUISE_SPEED_MPS,
    config::PP_MAX_OMEGA_RAD_S,
    config::PP_TURN_SLOW,
    config::PP_SPEED_LOOKAHEAD_M);
VelocityController wheelVelocity(leftMotor, rightMotor);

bool robotReady = false;
uint32_t previousControlUs = 0;
uint32_t previousLidarPollMs = 0;
uint8_t nextLidar = 0;
int16_t previousPursuitSegment = -1;
bool headingReanchorPending = false;

bool segmentUsesFrontWall(size_t segment) {
    if (WAYPOINT_COUNT < 2U || segment >= WAYPOINT_COUNT - 1U) return false;
    if (CYLINDER_SEGMENT_FIRST >= 0
        && static_cast<int16_t>(segment) >= CYLINDER_SEGMENT_FIRST
        && static_cast<int16_t>(segment) <= CYLINDER_SEGMENT_LAST) {
        return false;
    }
    for (size_t i = 0; i < FRONT_WALL_CORRECTION_SEGMENT_COUNT; ++i) {
        if (FRONT_WALL_CORRECTION_SEGMENTS[i] == segment) return true;
    }
    return false;
}

void stopRobot() {
    wheelVelocity.stop();
}

void pollOneLidar() {
    const uint32_t nowMs = millis();
    if (nowMs - previousLidarPollMs < config::LIDAR_POLL_SLOT_MS) return;
    previousLidarPollMs = nowMs;

    bool freshFront = false;
    switch (nextLidar) {
        case 0:
            leftLidar.pollIfReady();
            break;
        case 1:
            freshFront = frontLidar.pollIfReady();
            break;
        default:
            rightLidar.pollIfReady();
            break;
    }
    nextLidar = (nextLidar + 1U) % 3U;

    if (freshFront
        && config::FRONT_LIDAR_EKF_ENABLED
        && segmentUsesFrontWall(pursuit.segment())) {
        estimator.correctWithFrontLidar(
            frontLidar,
            config::FRONT_LIDAR_MOUNT_M);
    }
}

void maybeReanchorHeading() {
    const int16_t segment = static_cast<int16_t>(pursuit.segment());
    if (previousPursuitSegment >= 0 && segment > previousPursuitSegment) {
        headingReanchorPending = true;
    }
    previousPursuitSegment = segment;

    if (headingReanchorPending
        && fabsf(pursuit.lastCurvature()) < 1.0f
        && fabsf(imu.omegaRadS()) <= 0.20f) {
        estimator.reanchorHeading();
        headingReanchorPending = false;
    }
}

void updateMotion() {
    const uint32_t nowUs = micros();
    if (nowUs - previousControlUs < config::CONTROL_PERIOD_US) return;
    const float dt = static_cast<float>(nowUs - previousControlUs) * 1e-6f;
    previousControlUs = nowUs;

    estimator.update();
    const Pose2D pose = {
        estimator.x(),
        estimator.y(),
        estimator.heading(),
    };

    const int16_t segment = static_cast<int16_t>(pursuit.segment());
    const bool cylinderSegment = CYLINDER_SEGMENT_FIRST >= 0
        && segment >= CYLINDER_SEGMENT_FIRST
        && segment <= CYLINDER_SEGMENT_LAST;
    pursuit.setCruiseSpeed(
        cylinderSegment
            ? config::PP_CYLINDER_SPEED_MPS
            : config::PP_CRUISE_SPEED_MPS);

    float leftTargetMps = 0.0f;
    float rightTargetMps = 0.0f;
    if (!pursuit.compute(pose, leftTargetMps, rightTargetMps)
        || !config::MOTORS_ENABLED) {
        stopRobot();
        return;
    }

    if (config::IMU_ENABLED) maybeReanchorHeading();
    wheelVelocity.drive(
        estimator.leftDistanceM(),
        estimator.rightDistanceM(),
        dt,
        leftTargetMps,
        rightTargetMps);
}

void setup() {
    delay(1000);
    if (config::SERIAL_TELEMETRY_ENABLED) Serial.begin(115200);

    if (!buildWaypointRoute()) {
        stopRobot();
        return;
    }

    Wire.begin();
    Wire.setClock(config::I2C_CLOCK_HZ);
    Wire.setWireTimeout(config::I2C_TIMEOUT_US, true);
    Wire.clearWireTimeoutFlag();

    leftLidar.holdInReset();
    frontLidar.holdInReset();
    rightLidar.holdInReset();
    delay(100);
    const bool lidarsReady =
        leftLidar.begin(config::LIDAR_RANGE_PERIOD_MS)
        && frontLidar.begin(config::LIDAR_RANGE_PERIOD_MS)
        && rightLidar.begin(config::LIDAR_RANGE_PERIOD_MS);
    const bool imuReady = !config::IMU_ENABLED || imu.begin();
    if (!lidarsReady || !imuReady) {
        stopRobot();
        return;
    }

    encoders.reset();
    estimator.reset(INITIAL_HEADING_RAD);
    pursuit.setPath(WAYPOINTS, WAYPOINT_COUNT);
    wheelVelocity.reset(
        estimator.leftDistanceM(),
        estimator.rightDistanceM());
    previousControlUs = micros();
    previousLidarPollMs = millis();
    robotReady = true;
}

void loop() {
    if (!robotReady) {
        stopRobot();
        return;
    }
    updateMotion();
    pollOneLidar();
}
