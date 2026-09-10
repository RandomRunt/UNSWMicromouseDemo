#include "Motor.hpp"
#include "Encoder.hpp"
#include "PID.hpp"
#include "IMU.hpp"
#include "LIDAR.hpp"
#include "Model.hpp"
#include "StateEstimator.hpp"
#include "PurePursuit.hpp"
#include "VelocityPID.hpp"
#include "waypoints_planned.hpp"

#include <Wire.h>

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8

#define WHEEL_DIAMETER 0.032f
#define WHEEL_RADIUS   (WHEEL_DIAMETER / 2.0f)
#define WHEEL_BASE     0.08f

#define PID_KP 30.0f
#define PID_KI 1.0f
#define PID_KD 1.0f

#define PP_LOOKAHEAD       0.055f
#define PP_SPEED_LOOKAHEAD 0.2f   // longer preview → slow earlier before turns
#define PP_CRUISE_SPEED    0.3f
#define PP_CYLINDER_SPEED  0.2f
#define PP_MAX_OMEGA       9.0f
#define PP_TURN_SLOW       0.3f
#define PWM_PER_MPS        700.0f
#define VELOCITY_FILTER    0.2f

#define MOTORS_ENABLED true
#define IMU_ENABLED    true
#define FRONT_LIDAR_CORRECT true
#define IMU_REANCHOR_ENABLED true

Motor motorL(MOT1PWM, MOT1DIR);
Motor motorR(MOT2PWM, MOT2DIR);
Encoder encL(EN_1_A, EN_1_B);
Encoder encR(EN_2_A, EN_2_B);
PID pidL(PID_KP, PID_KI, PID_KD);
PID pidR(PID_KP, PID_KI, PID_KD);
Model model(WHEEL_BASE);
IMU imu;
StateEstimator stateEstimator(model, encL, encR, WHEEL_RADIUS, IMU_ENABLED ? &imu : nullptr);
PurePursuit purePursuit(
    WHEEL_BASE, PP_LOOKAHEAD, PP_CRUISE_SPEED, PP_MAX_OMEGA, PP_TURN_SLOW,
    PP_SPEED_LOOKAHEAD);
VelocityPID velocityPID(motorL, motorR, pidL, pidR, PWM_PER_MPS, VELOCITY_FILTER);

static constexpr int LIDAR_F_EDGE_OFFSET_MM = 30;
static constexpr int LIDAR_L_EDGE_OFFSET_MM = 3;
static constexpr int LIDAR_R_EDGE_OFFSET_MM = 3;
static constexpr float LIDAR_MOUNT_F_M = 0.0375f;

Lidar lidarL(A0, LIDAR_L_EDGE_OFFSET_MM);
Lidar lidarR(A1, LIDAR_R_EDGE_OFFSET_MM);
Lidar lidarF(A2, LIDAR_F_EDGE_OFFSET_MM);

uint32_t lastControlUs = 0;
uint32_t lastLidarPollMs = 0;
static constexpr uint32_t kLidarPollIntervalMs = 5;

#if IMU_REANCHOR_ENABLED
static int lastPpSegment = -1;
static bool imuReanchorPending = false;
static constexpr float kReanchorOmegaMax = 0.20f;   // rad/s
static constexpr float kReanchorStraightCurv = 1.0f; // 1/m — same idea as side-center
#endif

#ifndef FRONT_WALL_SEG_COUNT
#define FRONT_WALL_SEG_COUNT 0
#endif

static bool segmentInCylinder(int seg) {
  return CYLINDER_SEG_LO >= 0 && seg >= CYLINDER_SEG_LO && seg <= CYLINDER_SEG_HI;
}

static bool segmentAllowsFrontWallCorrect(int seg) {
  if (seg < 0 || segmentInCylinder(seg)) {
    return false;
  }
  for (size_t i = 0; i < FRONT_WALL_SEG_COUNT; ++i) {
    if (static_cast<int>(FRONT_WALL_SEGS[i]) == seg) {
      return true;
    }
  }
  return false;
}

#if IMU_REANCHOR_ENABLED
// After a segment advance, re-anchor only once PP is locally straight again
// (and rate is quiet). No timeout — prefer skip on pure zigzags over mid-turn.
static void maybeReanchorImuHeading(int seg) {
  if (!IMU_ENABLED) {
    return;
  }

  if (lastPpSegment >= 0 && seg > lastPpSegment) {
    imuReanchorPending = true;
  }
  lastPpSegment = seg;

  if (!imuReanchorPending) {
    return;
  }

  const bool straight = fabsf(purePursuit.lastCurvature()) < kReanchorStraightCurv;
  const bool settled = fabsf(imu.getOmegaZ()) <= kReanchorOmegaMax;
  if (straight && settled) {
    stateEstimator.reanchorHeading();
    imuReanchorPending = false;
  }
}
#endif

void updateMotion() {
  stateEstimator.update();

  if (!MOTORS_ENABLED) {
    velocityPID.stop();
    return;
  }

  uint32_t nowUs = micros();
  float dt = 0;
  if (lastControlUs != 0) {
    dt = (nowUs - lastControlUs) * 1e-6f;
  }
  lastControlUs = nowUs;
  if (dt <= 0) {
    return;
  }

  float leftDist = stateEstimator.getLeftDist();
  float rightDist = stateEstimator.getRightDist();

  Pose2D pose = {
    stateEstimator.getX(),
    stateEstimator.getY(),
    stateEstimator.getTheta(),
  };

  const int seg = static_cast<int>(purePursuit.getSegment());
  if (segmentInCylinder(seg)) {
    purePursuit.setCruiseSpeed(PP_CYLINDER_SPEED);
  } else {
    purePursuit.setCruiseSpeed(PP_CRUISE_SPEED);
  }

  float vLeftCmd = 0;
  float vRightCmd = 0;
  if (purePursuit.compute(pose, vLeftCmd, vRightCmd)) {
#if IMU_REANCHOR_ENABLED
    // Segment may advance inside compute(); re-read after.
    maybeReanchorImuHeading(static_cast<int>(purePursuit.getSegment()));
#endif
    velocityPID.drive(leftDist, rightDist, dt, vLeftCmd, vRightCmd);
  } else {
#if IMU_REANCHOR_ENABLED
    maybeReanchorImuHeading(static_cast<int>(purePursuit.getSegment()));
#endif
    velocityPID.stop();
  }
}

void pollFrontLidarCorrect() {
  if (!FRONT_LIDAR_CORRECT) {
    return;
  }

  uint32_t nowMs = millis();
  if (nowMs - lastLidarPollMs < kLidarPollIntervalMs) {
    return;
  }
  lastLidarPollMs = nowMs;

  if (!lidarF.pollIfReady()) {
    return;
  }

  const int seg = static_cast<int>(purePursuit.getSegment());
  if (!segmentAllowsFrontWallCorrect(seg)) {
    return;
  }

  stateEstimator.correctWithFrontLidar(lidarF, LIDAR_MOUNT_F_M);
}

void setup() {
  delay(1000);

  Wire.begin();
  Lidar::initAll();

  if (IMU_ENABLED) {
    imu.begin();
  }

  stateEstimator.reset(INITIAL_HEADING_RAD);

  lastControlUs = micros();
  velocityPID.reset(
      stateEstimator.getLeftDist(),
      stateEstimator.getRightDist());
  purePursuit.setPath(WAYPOINTS, WAYPOINT_LEN);
#if IMU_REANCHOR_ENABLED
  lastPpSegment = -1;
  imuReanchorPending = false;
#endif
}

void loop() {
  updateMotion();
  pollFrontLidarCorrect();
}
