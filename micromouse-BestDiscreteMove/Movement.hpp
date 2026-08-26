// AI-ASSISTED FILE: Task 4.2/4.3 interface additions generated with OpenAI
// Codex (2026-08-11 to 2026-08-12). AI-authored sections are identified inline.
#pragma once

#include <Arduino.h>

#include "Encoder.hpp"
#include "Display.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "Imu.hpp"
#include "Lidar.hpp"
#include "MotionTypes.hpp"

namespace mtrn3100 {

// Competition movement surface for section 4.1.2. The optional diagnostic
// display is framebuffer-free and rate-limited; earlier bench-task modes stay
// excluded so the Nano keeps ample stack and a lightly loaded shared I2C bus.
class Movement {
public:
    Movement();
    bool initialise(
        bool enableDisplay = false,
        bool showLidarDiagnostics = true);
    bool moveForward(float distanceMm);
    CellMoveResult moveDeadReckoned(float distanceMm);
    CellMoveResult moveOneCell();
    void adjusttWall();
    bool observeWalls(WallObservation& observation);
    bool alignAndRecalibrateFromThreeWalls();
    bool recalibrateAtCurrentHeading();
    bool rotateLeft(float angleDeg);
    bool rotateRight(float angleDeg);
    bool rotateLeftTask41(float angleDeg);
    bool rotateRightTask41(float angleDeg);
    bool rotateLeftDeadReckoned(float angleDeg);
    bool rotateRightDeadReckoned(float angleDeg);
    void adoptCurrentHeading();
    void settle(uint16_t durationMs);
    void stop();
    void showMap(
        const Maze& maze,
        const Pose& pose,
        const Pose& start,
        uint8_t goalRow,
        uint8_t goalColumn,
        MappingPhase phase);

private:
    float correctedHeading() const;
    bool calibrateYawRateBias(uint16_t durationMs);
    bool sampleThreeWallRanges(
        float& frontMm,
        float& leftMm,
        float& rightMm);
    bool moveForwardWithin(
        float distanceMm,
        uint32_t timeoutMs,
        bool task41FastProfile);
    bool turnByImu(
        float signedAngleDeg,
        bool serviceLidarsAfterTurn,
        bool task41FastProfile);
    bool settleImuOnly(uint16_t durationMs);
    void serviceNextLidar();
    void serviceDisplay();

    Motor LeftMotor;
    Motor RightMotor;
    Encoder LeftEncoder;
    Encoder RightEncoder;
    PIDController turnPIDController;
    Imu imu;
    Lidar FrontLidar;
    Lidar LeftLidar;
    Lidar RightLidar;
    MicromouseDisplay display;
    uint32_t lastLidarPollMs = 0;
    uint32_t lastDisplayUpdateMs = 0;
    float headingReference = 0.0f;
    float heading = 0.0f;
    uint8_t nextLidar = 0;
    uint8_t nextDisplayLine = 0;
    bool yawRateBiasValid = false;
    bool lidarDiagnosticsEnabled = false;
};

}  // namespace mtrn3100
