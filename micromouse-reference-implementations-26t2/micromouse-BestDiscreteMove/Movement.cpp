// AI-ASSISTED FILE: Task 4.2/4.3 sensing/motion generated with OpenAI Codex
// (2026-08-11 to 2026-08-12). AI-authored sections are identified by comments.
#include "Movement.hpp"

#include <Wire.h>

#include "MappingObservation.hpp"
#include "RobotGeometry.hpp"

namespace mtrn3100 {

namespace {

// Kit pin map (Nano).
constexpr uint8_t LMOTOR_PWM = 11;
constexpr uint8_t LMOTOR_DIR = 12;
constexpr uint8_t RMOTOR_PWM = 9;
constexpr uint8_t RMOTOR_DIR = 10;
constexpr uint8_t LENCODER_A = 3;
constexpr uint8_t LENCODER_B = 8;
constexpr uint8_t RENCODER_A = 2;
constexpr uint8_t RENCODER_B = 7;
constexpr uint8_t LEFT_LIDAR_PIN = A0;
constexpr uint8_t RIGHT_LIDAR_PIN = A1;
constexpr uint8_t FRONT_LIDAR_PIN = A2;

constexpr float WHEEL_RADIUS_MM = 16.1f;
constexpr float WHEEL_RADIUS_DR_MM = 16.9f;

// Positive values always mean physical forward or counter-clockwise.
constexpr int8_t L_MOTOR_FWD = +1;
constexpr int8_t R_MOTOR_FWD = -1;
constexpr int8_t L_ENC_FWD = +1;
constexpr int8_t R_ENC_FWD = -1;
constexpr int8_t YAW_CCW_SIGN = +1;

constexpr float KP_HOLD = 8.0f;
constexpr int MIN_MOVE_PWM = 125;
constexpr int SIMPLE_MOVE_PWM = 175;
constexpr int TASK41_CRUISE_PWM = 175;
constexpr float TASK41_DECEL_PWM_PER_MM = 0.9f;
constexpr int FORWARD_PWM_RISE_STEP = 8;
constexpr int FORWARD_PWM_FALL_STEP = 16;
constexpr float K_LIDAR_HEADING = 0.25f;
constexpr float MAX_LIDAR_ANGLE = 8.0f;
constexpr float LIDAR_HEADING_ALPHA = 0.25f;
// Preserve the empirically tuned Task 4.1.2 targets. Task 4.3 classification
// uses the placement-derived values in RobotGeometry.hpp separately.
constexpr float SIDE_WALL_TARGET_MM = 50.0f;
constexpr float SIDE_WALL_MAX_MM = 100.0f;
constexpr float CENTRE_DEADBAND_MM = 5.0f;
constexpr float MAX_STEERING_PWM = 50.0f;

constexpr float FRONT_TARGET_MM = 52.0f;
constexpr float FRONT_TOL_MM = 3.0f;
constexpr float FRONT_EMERGENCY_MM = 45.0f;
constexpr float FRONT_ENABLE_REMAINING_MM = 30.0f;
constexpr float FRONT_MAX_VALID_MM = 100.0f;
constexpr float FRONT_MATCH_TOL_MM = 20.0f;
constexpr float FRONT_MAX_EXTRA_TRAVEL_MM = 20.0f;
constexpr float FORWARD_ENCODER_TOLERANCE_MM = 5.0f;
constexpr uint32_t FRONT_READING_MAX_AGE_MS = 150;
constexpr uint32_t SIDE_READING_MAX_AGE_MS = 150;
constexpr uint8_t FRONT_CONFIRM_READINGS = 2;

constexpr int TURN_MIN_PWM = 30;
constexpr int TURN_MAX_PWM = 120;
constexpr float TURN_TOL_DEG = 1.0f;
constexpr float TURN_STILL_DPS = 2.0f;
constexpr float BIAS_MAX_VARIANCE = 0.25f;
constexpr float BIAS_MAX_ENCODER_DELTA_RAD = 0.02f;
constexpr uint16_t TURN_SETTLE_MS = 100;
constexpr uint16_t TASK41_TURN_POST_SETTLE_MS = 60;
constexpr uint16_t BIAS_SAMPLE_GUARD_MS = 50;
constexpr uint16_t BIAS_MIN_SAMPLES = 20;
constexpr uint8_t BIAS_MAX_INVALID_SAMPLES = 5;

// Leave margin below the marking requirement's ten-second ceiling for the
// final I2C transaction, motor stop, and stationary settling.
constexpr uint32_t MOVE_TIMEOUT_MS = 9000;
constexpr uint32_t TURN_TIMEOUT_MS = 7000;
constexpr uint8_t MAX_CONSECUTIVE_IMU_MISSES = 30;
constexpr uint8_t LOOP_DELAY_MS = 5;

// One lidar is polled per slot; each still receives one poll every 60 ms.
// This avoids the original synchronized three-sensor I2C burst.
constexpr uint16_t LIDAR_PERIOD_MS = 60;
constexpr uint16_t LIDAR_SLOT_MS = LIDAR_PERIOD_MS / 3;
// Update one large row at a time. Every sensor is refreshed on screen every
// 600 ms without creating a continuous stream of OLED traffic.
constexpr uint16_t DISPLAY_LINE_PERIOD_MS = 200;
constexpr uint32_t DISPLAY_READING_MAX_AGE_MS = 150;
constexpr uint32_t I2C_CLOCK_HZ = 100000UL;
constexpr uint32_t I2C_TIMEOUT_US = 5000UL;

// Task 4.2 follows a camera-planned metric path using only encoder distance
// and IMU heading. Its controller is deliberately isolated from wall-following
// and front-wall stopping because cylindrical obstacles are not maze walls.
// The current planning ROI has a theoretical corner-to-corner segment just
// below 1580 mm. Keep the parser and motion primitive aligned at 1600 mm.
constexpr float TASK42_MAX_LEG_MM = 1600.0f;
constexpr int TASK42_CRUISE_PWM = 150;
constexpr int TASK42_APPROACH_PWM = 140;
constexpr int TASK42_MAX_PWM = 170;
constexpr int TASK42_MAX_STEERING_PWM = 20;
constexpr float TASK42_APPROACH_DISTANCE_MM = 35.0f;
constexpr float TASK42_STOP_LEAD_MM = 1.0f;
constexpr uint16_t TASK42_ARRIVAL_SETTLE_MS = 180;

// Adapter tolerances for reusing the 4.1.2 forward primitive as an atomic
// Task 4.3 cell transaction. Do not restart a sub-stiction 6-10 mm residual.
constexpr float TASK43_CELL_RETRY_TOLERANCE_MM = 10.0f;
constexpr uint16_t TASK43_CELL_SETTLE_MS = 180;

// After a completed Task 4.3 cell movement, use an immediate front wall as a
// longitudinal reference. The five-millimetre band is the requested +/-0.5 cm
// accuracy; the travel limit prevents a bad range from becoming another cell
// movement before the two-second timeout expires.
constexpr float WALL_ADJUST_TARGET_MM = geometry::FRONT_WALL_CENTRED_MM;
constexpr float WALL_ADJUST_TOLERANCE_MM = 15.0f;
constexpr float WALL_ADJUST_KP = 1.5f;
constexpr float WALL_ADJUST_MAX_TRAVEL_MM = 40.0f;
constexpr int WALL_ADJUST_MIN_PWM = 75;
constexpr int WALL_ADJUST_MAX_PWM = SIMPLE_MOVE_PWM;
constexpr uint16_t WALL_ADJUST_ZERO_SETTLE_MS = 180;
constexpr uint16_t WALL_ADJUST_TIMEOUT_MS = 2000;
constexpr uint8_t WALL_ADJUST_ZERO_SAMPLES = 2;

constexpr uint8_t MAP_SCAN_SAMPLES = 5;
constexpr uint8_t MAP_SCAN_CONSENSUS = 4;
constexpr uint16_t MAP_SCAN_STILL_MS = 120;
constexpr uint16_t MAP_SCAN_TIMEOUT_MS = 1000;
constexpr float MAP_SCAN_MAX_YAW_RATE_DPS = 2.0f;
constexpr float MAP_SCAN_MAX_ENCODER_RAD = 0.02f;

// A confirmed three-wall cell is an occasional absolute-heading landmark.
// Sample a deliberately wide symmetric bracket: the front range is convex
// around the wall-normal direction, while the two side ranges ensure the
// in-place sweep remains inside the same dead end. Near-zero curvature is
// rejected rather than turning on range noise.
constexpr float LANDMARK_PROBE_DEG = 12.0f;
constexpr float LANDMARK_MAX_CORRECTION_DEG = 20.0f;
constexpr float LANDMARK_MIN_CURVATURE_MM = 1.0f;
constexpr float LANDMARK_MAX_VALIDATION_INCREASE_MM = 2.0f;
constexpr float LANDMARK_MIN_CLEARANCE_MM = 25.0f;
constexpr uint8_t LANDMARK_RANGE_SAMPLES = 5;
constexpr uint16_t LANDMARK_SAMPLE_TIMEOUT_MS = 750;

int turnCommand(float output, bool inBand) {
    if (inBand) return 0;
    const int magnitude = constrain(
        static_cast<int>(fabs(output)), TURN_MIN_PWM, TURN_MAX_PWM);
    return output < 0.0f ? -magnitude : magnitude;
}

float wrapAngle180(float angle) {
    if (!isfinite(angle)) return 0.0f;
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

int slewPwm(int current, int target) {
    if (target > current) {
        return min(target, current + FORWARD_PWM_RISE_STEP);
    }
    return max(target, current - FORWARD_PWM_FALL_STEP);
}

struct SensorVotes {
    uint16_t sequence = 0;
    uint8_t observations = 0;
    uint8_t walls = 0;
    uint8_t openings = 0;
};

void countObservation(
    Lidar& lidar,
    float wallMaximumMm,
    SensorVotes& votes) {
    LidarObservation observation;
    if (!lidar.getLatestObservation(observation)
        || observation.sequence == votes.sequence
        || votes.observations >= MAP_SCAN_SAMPLES) {
        return;
    }

    votes.sequence = observation.sequence;
    ++votes.observations;
    const ObservedEdge edge = classifyMappingRange(
        observation.rangeValid,
        observation.distanceMm,
        observation.rangeStatus,
        wallMaximumMm,
        geometry::OPEN_MIN_MM);
    if (edge == ObservedEdge::Wall) {
        ++votes.walls;
    } else if (edge == ObservedEdge::Open) {
        ++votes.openings;
    }
}

ObservedEdge consensus(const SensorVotes& votes) {
    if (votes.walls >= MAP_SCAN_CONSENSUS && votes.openings == 0) {
        return ObservedEdge::Wall;
    }
    // False-open is the dangerous error, so require every completed sample to
    // agree before a passage becomes traversable.
    if (votes.openings == MAP_SCAN_SAMPLES && votes.walls == 0) {
        return ObservedEdge::Open;
    }
    return ObservedEdge::Unknown;
}

bool recordRangeSample(
    Lidar& lidar,
    uint16_t& sequence,
    uint16_t* samples,
    uint8_t& sampleCount) {
    LidarObservation observation;
    if (!lidar.getLatestObservation(observation)
        || observation.sequence == sequence) {
        return false;
    }

    sequence = observation.sequence;
    if (!observation.rangeValid
        || observation.distanceMm < LANDMARK_MIN_CLEARANCE_MM
        || observation.distanceMm > geometry::FRONT_WALL_MAX_MM
        || sampleCount >= LANDMARK_RANGE_SAMPLES) {
        return false;
    }

    samples[sampleCount++] = observation.distanceMm;
    return true;
}

uint16_t medianRange(uint16_t* samples) {
    for (uint8_t i = 1; i < LANDMARK_RANGE_SAMPLES; ++i) {
        const uint16_t value = samples[i];
        uint8_t j = i;
        while (j > 0 && samples[j - 1U] > value) {
            samples[j] = samples[j - 1U];
            --j;
        }
        samples[j] = value;
    }
    return samples[LANDMARK_RANGE_SAMPLES / 2U];
}

}  // namespace

Movement::Movement()
    : LeftMotor(LMOTOR_PWM, LMOTOR_DIR),
      RightMotor(RMOTOR_PWM, RMOTOR_DIR),
      LeftEncoder(LENCODER_A, LENCODER_B),
      RightEncoder(RENCODER_A, RENCODER_B),
      turnPIDController(6.0f, 0.0f, 0.3f),
      FrontLidar(FRONT_LIDAR_PIN, 0x30),
      LeftLidar(LEFT_LIDAR_PIN, 0x31),
      RightLidar(RIGHT_LIDAR_PIN, 0x32) {}

bool Movement::initialise(bool enableDisplay, bool showLidarDiagnostics) {
    stop();

    // Wire's AVR default timeout is disabled. A single electrical bus fault
    // would therefore leave the last PWM command active forever. Resetting the
    // TWI peripheral after 5 ms makes a transient sample loss recoverable.
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setWireTimeout(I2C_TIMEOUT_US, true);
    Wire.clearWireTimeoutFlag();

    if (!imu.initialise()) return false;

    // Hold all identical lidars down, then wake/re-address one at a time.
    pinMode(FRONT_LIDAR_PIN, OUTPUT);
    pinMode(LEFT_LIDAR_PIN, OUTPUT);
    pinMode(RIGHT_LIDAR_PIN, OUTPUT);
    digitalWrite(FRONT_LIDAR_PIN, LOW);
    digitalWrite(LEFT_LIDAR_PIN, LOW);
    digitalWrite(RIGHT_LIDAR_PIN, LOW);
    delay(100);

    if (!FrontLidar.initialise()
        || !LeftLidar.initialise()
        || !RightLidar.initialise()) {
        return false;
    }

    // Phase-shift the continuous conversions as well as the host-side polls.
    if (!FrontLidar.startContinuous(LIDAR_PERIOD_MS)) return false;
    delay(LIDAR_SLOT_MS);
    if (!LeftLidar.startContinuous(LIDAR_PERIOD_MS)) return false;
    delay(LIDAR_SLOT_MS);
    if (!RightLidar.startContinuous(LIDAR_PERIOD_MS)) return false;
    lastLidarPollMs = millis();

    // The OLED is diagnostic-only: a missing or failed display must never
    // prevent the maze run. It reads cached lidar values and does not perform
    // additional sensor measurements.
    if (enableDisplay) display.initialise();
    lidarDiagnosticsEnabled = enableDisplay && showLidarDiagnostics;
    lastDisplayUpdateMs = millis();

    heading = 0.0f;
    yawRateBiasValid = calibrateYawRateBias(1000);
    if (!yawRateBiasValid || !imu.update()) return false;

    // Define maze zero after stationary bias calibration so the calibration
    // window cannot appear as a real turn.
    headingReference = YAW_CCW_SIGN * imu.getYaw();
    return true;
}

float Movement::correctedHeading() const {
    return YAW_CCW_SIGN * imu.getYaw() - headingReference;
}

void Movement::serviceNextLidar() {
    const uint32_t now = millis();
    if (now - lastLidarPollMs >= LIDAR_SLOT_MS) {
        lastLidarPollMs = now;
        switch (nextLidar) {
            case 0: FrontLidar.updateDistance(); break;
            case 1: LeftLidar.updateDistance(); break;
            default: RightLidar.updateDistance(); break;
        }
        nextLidar = (nextLidar + 1) % 3;
        return;
    }

    // Keep the OLED on a different control-loop slot from lidar transactions,
    // avoiding a large back-to-back burst on the shared I2C bus.
    serviceDisplay();
}

void Movement::serviceDisplay() {
    if (!lidarDiagnosticsEnabled || !display.isReady()) return;

    const uint32_t now = millis();
    if (now - lastDisplayUpdateMs < DISPLAY_LINE_PERIOD_MS) return;
    lastDisplayUpdateMs = now;

    switch (nextDisplayLine) {
        case 0:
            display.showReading(
                0,
                FrontLidar.getDistance(),
                FrontLidar.hasFreshObservation(DISPLAY_READING_MAX_AGE_MS));
            break;
        case 1:
            display.showReading(
                1,
                LeftLidar.getDistance(),
                LeftLidar.hasFreshObservation(DISPLAY_READING_MAX_AGE_MS));
            break;
        default:
            display.showReading(
                2,
                RightLidar.getDistance(),
                RightLidar.hasFreshObservation(DISPLAY_READING_MAX_AGE_MS));
            break;
    }
    nextDisplayLine = (nextDisplayLine + 1) % 3;
}

bool Movement::moveForward(float distanceMm) {
    return moveForwardWithin(distanceMm, MOVE_TIMEOUT_MS, true);
}

bool Movement::moveForwardWithin(
    float distanceMm,
    uint32_t timeoutMs,
    bool task41FastProfile) {
    const uint32_t boundedTimeoutMs = timeoutMs < MOVE_TIMEOUT_MS
                                    ? timeoutMs
                                    : MOVE_TIMEOUT_MS;
    if (!yawRateBiasValid
        || !isfinite(distanceMm)
        || distanceMm <= 0.0f
        || boundedTimeoutMs == 0) {
        stop();
        return false;
    }

    const float nominalHeading = heading;
    const float leftStart = L_ENC_FWD * LeftEncoder.getRotation();
    const float rightStart = R_ENC_FWD * RightEncoder.getRotation();
    const float targetRotation = distanceMm / WHEEL_RADIUS_MM;
    const uint32_t startedAt = millis();

    uint32_t lastLidarControlUpdate = 0;
    uint32_t lastProcessedFrontReading = 0;
    float lidarHeadingOffset = 0.0f;
    bool frontApproachActive = false;
    uint8_t frontCloseReadings = 0;
    uint8_t consecutiveImuMisses = 0;
    int appliedLeftPwm = 0;
    int appliedRightPwm = 0;
    bool completed = false;

    while (true) {
        // Heading is sampled first. Lidar work is scheduled after the new motor
        // command so a rare bounded I2C timeout cannot leave a stale command
        // active for longer than one transaction.
        if (imu.update()) {
            consecutiveImuMisses = 0;
        } else if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            stop();
            return false;
        }

        const float leftTravel = L_ENC_FWD * LeftEncoder.getRotation() - leftStart;
        const float rightTravel = R_ENC_FWD * RightEncoder.getRotation() - rightStart;

        const float rotationTravelled = (leftTravel + rightTravel) * 0.5f;
        const float remainingRotation = targetRotation - rotationTravelled;
        const float remainingMm = remainingRotation * WHEEL_RADIUS_MM;
        const float travelledMm = rotationTravelled * WHEEL_RADIUS_MM;

        const bool frontFresh = FrontLidar.hasFreshReading(
            FRONT_READING_MAX_AGE_MS);
        const float frontDistance = FrontLidar.getDistance();
        const uint32_t frontReadingTime = FrontLidar.getLastReadingTime();
        const bool frontUsable = frontFresh
                              && frontDistance <= FRONT_MAX_VALID_MM;

        if (remainingMm <= FRONT_ENABLE_REMAINING_MM
            && frontUsable
            && frontReadingTime != lastProcessedFrontReading) {
            lastProcessedFrontReading = frontReadingTime;
            const float expectedFrontDistance = FRONT_TARGET_MM
                                              + max(remainingMm, 0.0f);

            if (!frontApproachActive
                && fabs(frontDistance - expectedFrontDistance)
                    <= FRONT_MATCH_TOL_MM) {
                frontApproachActive = true;
            }

            if (frontApproachActive
                && frontDistance <= FRONT_TARGET_MM + FRONT_TOL_MM) {
                if (frontDistance <= FRONT_EMERGENCY_MM) {
                    frontCloseReadings = FRONT_CONFIRM_READINGS;
                } else {
                    if (frontCloseReadings < FRONT_CONFIRM_READINGS) {
                        ++frontCloseReadings;
                    }
                }
            } else if (frontApproachActive) {
                frontCloseReadings = 0;
            }
        }

        if (frontCloseReadings >= FRONT_CONFIRM_READINGS) {
            completed = true;
            break;
        }
        if (frontCloseReadings > 0) {
            // Wait for one independent confirmation sample with motors off.
            appliedLeftPwm = 0;
            appliedRightPwm = 0;
            LeftMotor.setPWM(0);
            RightMotor.setPWM(0);
            serviceNextLidar();
            if (!frontUsable) {
                // Sensor loss cannot be promoted to successful route progress.
                break;
            }
            if (millis() - startedAt >= boundedTimeoutMs) break;
            delay(LOOP_DELAY_MS);
            continue;
        }

        // A missing front sample is not allowed to trap the robot in a stopped
        // lidar-approach state. Resume using the encoder target immediately.
        if (frontApproachActive && !frontUsable) {
            frontApproachActive = false;
        }

        if (frontApproachActive) {
            if (travelledMm >= distanceMm + FRONT_MAX_EXTRA_TRAVEL_MM) break;
        } else if (remainingMm <= FORWARD_ENCODER_TOLERANCE_MM) {
            completed = true;
            break;
        }
        if (millis() - startedAt >= boundedTimeoutMs) break;

        // The original radians-based profile starts a one-cell move below the
        // motor floor and therefore remains at MIN_MOVE_PWM. Task 4.1 uses a
        // distance-based profile so every full cell reaches the established
        // 175 PWM cap, then decelerates over the final ~56 mm. Task 4.3 keeps
        // the original conservative profile.
        int basePwm = task41FastProfile
            ? constrain(
                MIN_MOVE_PWM
                    + static_cast<int>(
                        max(remainingMm, 0.0f) * TASK41_DECEL_PWM_PER_MM),
                MIN_MOVE_PWM,
                TASK41_CRUISE_PWM)
            : constrain(
                static_cast<int>(remainingRotation * KP_HOLD),
                MIN_MOVE_PWM,
                SIMPLE_MOVE_PWM);
        if (remainingMm <= 10.0f) basePwm = 50;

        const uint32_t now = millis();
        if (now - lastLidarControlUpdate >= LIDAR_PERIOD_MS) {
            lastLidarControlUpdate = now;

            const float leftDistance = LeftLidar.getDistance();
            const float rightDistance = RightLidar.getDistance();
            const bool leftWall = LeftLidar.hasFreshReading(
                                      SIDE_READING_MAX_AGE_MS)
                               && leftDistance > 0.0f
                               && leftDistance <= SIDE_WALL_MAX_MM;
            const bool rightWall = RightLidar.hasFreshReading(
                                       SIDE_READING_MAX_AGE_MS)
                                && rightDistance > 0.0f
                                && rightDistance <= SIDE_WALL_MAX_MM;

            float centreError = 0.0f;
            if (leftWall && rightWall) {
                centreError = (rightDistance - leftDistance) * 0.5f;
            } else if (leftWall) {
                centreError = SIDE_WALL_TARGET_MM - leftDistance;
            } else if (rightWall) {
                centreError = rightDistance - SIDE_WALL_TARGET_MM;
            }
            if (fabs(centreError) < CENTRE_DEADBAND_MM) centreError = 0.0f;

            const float requestedOffset = constrain(
                -K_LIDAR_HEADING * centreError,
                -MAX_LIDAR_ANGLE,
                MAX_LIDAR_ANGLE);
            lidarHeadingOffset += LIDAR_HEADING_ALPHA
                                * (requestedOffset - lidarHeadingOffset);
        }

        const float steeringLimit = min(
            static_cast<float>(basePwm), MAX_STEERING_PWM);
        // Positive steering makes the left wheel faster and turns clockwise.
        const float desiredHeading = nominalHeading + lidarHeadingOffset;
        const float headingError = wrapAngle180(
            correctedHeading() - desiredHeading);
        const float steering = constrain(
            KP_HOLD * headingError, -steeringLimit, steeringLimit);

        const int targetLeftPwm = constrain(
            basePwm + static_cast<int>(steering), 0, 255);
        const int targetRightPwm = constrain(
            basePwm - static_cast<int>(steering), 0, 255);

        // Slewing both propulsion and steering commands reduces supply dips and
        // motor-brush EMI without changing the steady-state controller output.
        appliedLeftPwm = slewPwm(appliedLeftPwm, targetLeftPwm);
        appliedRightPwm = slewPwm(appliedRightPwm, targetRightPwm);
        LeftMotor.setPWM(L_MOTOR_FWD * appliedLeftPwm);
        RightMotor.setPWM(R_MOTOR_FWD * appliedRightPwm);

        serviceNextLidar();
        delay(LOOP_DELAY_MS);
    }

    stop();
    return completed;
}

CellMoveResult Movement::moveDeadReckoned(float distanceMm) {
    if (!yawRateBiasValid
        || !isfinite(distanceMm)
        || distanceMm <= 0.0f
        || distanceMm > TASK42_MAX_LEG_MM) {
        stop();
        return CellMoveResult::InvalidRequest;
    }

    const float nominalHeading = heading;
    const float leftStart = L_ENC_FWD * LeftEncoder.getRotation();
    const float rightStart = R_ENC_FWD * RightEncoder.getRotation();
    const float targetRotation = distanceMm / WHEEL_RADIUS_DR_MM;
    const uint32_t startedAt = millis();
    int appliedLeftPwm = 0;
    int appliedRightPwm = 0;
    CellMoveResult result = CellMoveResult::Timeout;

    // Keep Task 4.2 lidar-free, but do not let a missing encoder transition
    // hold the motors on forever. A failed IMU sample retains the last valid
    // heading estimate until another sample arrives or this deadline expires.
    while (true) {
        imu.update();

        const float leftTravel = L_ENC_FWD * LeftEncoder.getRotation()
                               - leftStart;
        const float rightTravel = R_ENC_FWD * RightEncoder.getRotation()
                                - rightStart;
        const float meanTravel = (leftTravel + rightTravel) * 0.5f;
        const float remainingMm = (targetRotation - meanTravel)
                                * WHEEL_RADIUS_DR_MM;
        const float headingError = wrapAngle180(
            correctedHeading() - nominalHeading);
        if (remainingMm <= TASK42_STOP_LEAD_MM) {
            result = CellMoveResult::Arrived;
            break;
        }
        if (millis() - startedAt >= MOVE_TIMEOUT_MS) break;

        const int basePwm = remainingMm <= TASK42_APPROACH_DISTANCE_MM
                          ? TASK42_APPROACH_PWM
                          : TASK42_CRUISE_PWM;
        const int steering = constrain(
            static_cast<int>(KP_HOLD * headingError),
            -TASK42_MAX_STEERING_PWM,
            TASK42_MAX_STEERING_PWM);

        // Only accelerate the outer wheel. Neither side is pulled below the
        // empirically reliable motor floor, avoiding a one-wheel spiral.
        int targetLeftPwm = basePwm;
        int targetRightPwm = basePwm;
        if (steering > 0) {
            targetLeftPwm += steering;
        } else {
            targetRightPwm -= steering;
        }
        targetLeftPwm = min(targetLeftPwm, TASK42_MAX_PWM);
        targetRightPwm = min(targetRightPwm, TASK42_MAX_PWM);

        appliedLeftPwm = slewPwm(appliedLeftPwm, targetLeftPwm);
        appliedRightPwm = slewPwm(appliedRightPwm, targetRightPwm);
        LeftMotor.setPWM(L_MOTOR_FWD * appliedLeftPwm);
        RightMotor.setPWM(R_MOTOR_FWD * appliedRightPwm);

        // No lidar or OLED I2C transactions occur during Task 4.2 motion.
        delay(LOOP_DELAY_MS);
    }

    stop();
    // Wait out ordinary mechanical coast without turning the stopped interval
    // into another pass/fail gate. Keep it lidar-free beside Task 4.2 objects.
    const uint32_t settleStartedAt = millis();
    while (millis() - settleStartedAt < TASK42_ARRIVAL_SETTLE_MS) {
        imu.update();
        delay(LOOP_DELAY_MS);
    }
    return result;
}

CellMoveResult Movement::moveOneCell() {
    if (!yawRateBiasValid) {
        stop();
        return CellMoveResult::InvalidRequest;
    }

    // Task 4.3 deliberately reuses the section 4.1.2 forward primitive. Keep
    // one encoder origin for the complete cell transaction so a recoverable
    // partial attempt resumes only the remaining distance instead of issuing
    // another full 180 mm request and corrupting the logical maze pose.
    const float leftStart = L_ENC_FWD * LeftEncoder.getRotation();
    const float rightStart = R_ENC_FWD * RightEncoder.getRotation();
    const float cellDistanceMm = geometry::CELL_SIZE_MM;
    const uint32_t movementStartedAt = millis();
    bool distanceReached = false;

    while (millis() - movementStartedAt < MOVE_TIMEOUT_MS) {
        const float leftTravel = L_ENC_FWD * LeftEncoder.getRotation()
                               - leftStart;
        const float rightTravel = R_ENC_FWD * RightEncoder.getRotation()
                                - rightStart;
        const float travelledMm = (leftTravel + rightTravel) * 0.5f
                                * WHEEL_RADIUS_MM;
        const float remainingMm = cellDistanceMm - travelledMm;
        if (!isfinite(remainingMm)) {
            stop();
            return CellMoveResult::ArrivalUncertain;
        }
        if (remainingMm <= TASK43_CELL_RETRY_TOLERANCE_MM) {
            distanceReached = true;
            break;
        }

        // A true result includes the 4.1.2 controller's carefully gated front
        // wall completion, which is a valid absolute longitudinal reference.
        const uint32_t elapsedMs = millis() - movementStartedAt;
        const uint32_t remainingTimeMs = elapsedMs < MOVE_TIMEOUT_MS
                                       ? MOVE_TIMEOUT_MS - elapsedMs
                                       : 0;
        if (moveForwardWithin(remainingMm, remainingTimeMs, false)) {
            distanceReached = true;
            break;
        }

        // A false result may follow partial travel. Stop, refresh stationary
        // sensors, then recompute the remainder from the original baseline.
        settle(TASK43_CELL_SETTLE_MS);
    }

    stop();
    const float leftTravelMm =
        (L_ENC_FWD * LeftEncoder.getRotation() - leftStart)
        * WHEEL_RADIUS_MM;
    const float rightTravelMm =
        (R_ENC_FWD * RightEncoder.getRotation() - rightStart)
        * WHEEL_RADIUS_MM;
    if (!distanceReached) {
        return fabs(leftTravelMm) <= TASK43_CELL_RETRY_TOLERANCE_MM
            && fabs(rightTravelMm) <= TASK43_CELL_RETRY_TOLERANCE_MM
             ? CellMoveResult::Timeout
             : CellMoveResult::ArrivalUncertain;
    }
    if (!isfinite(leftTravelMm) || !isfinite(rightTravelMm)) {
        return CellMoveResult::ArrivalUncertain;
    }

    // The 4.1.2 side-wall controller may finish with a small temporary heading
    // offset. Let mechanical coast finish, then use the same turn controller
    // to restore the committed cardinal heading before Task 4.3 scans walls.
    settle(TASK43_CELL_SETTLE_MS);
    return turnByImu(0.0f, true, false)
         ? CellMoveResult::Arrived
         : CellMoveResult::HeadingLost;
}

void Movement::adjusttWall() {
    stop();
    const uint32_t startedAt = millis();

    // First require the normal stationary mapping consensus. In particular, a
    // distant wall seen through an open edge must not pull the robot into the
    // next cell.
    WallObservation walls;
    if (!observeWalls(walls)
        || walls.front != ObservedEdge::Wall
        || millis() - startedAt >= WALL_ADJUST_TIMEOUT_MS) {
        stop();
        return;
    }

    if (!imu.update()) {
        stop();
        return;
    }

    const float nominalHeading = heading;
    const float leftStart = L_ENC_FWD * LeftEncoder.getRotation();
    const float rightStart = R_ENC_FWD * RightEncoder.getRotation();
    uint16_t lastSequence = 0;
    LidarObservation latest;
    if (FrontLidar.getLatestObservation(latest)) {
        lastSequence = latest.sequence;
    }

    uint32_t zeroSince = 0;
    float zeroLeftStart = leftStart;
    float zeroRightStart = rightStart;
    uint8_t zeroSamples = 0;
    uint8_t consecutiveImuMisses = 0;

    while (millis() - startedAt < WALL_ADJUST_TIMEOUT_MS) {
        if (imu.update()) {
            consecutiveImuMisses = 0;
        } else if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            break;
        }

        serviceNextLidar();
        if (!FrontLidar.getLatestObservation(latest)
            || latest.sequence == lastSequence) {
            if (!FrontLidar.hasFreshReading(FRONT_READING_MAX_AGE_MS)) {
                LeftMotor.setPWM(0);
                RightMotor.setPWM(0);
                zeroSince = 0;
                zeroSamples = 0;
            }
            delay(LOOP_DELAY_MS);
            continue;
        }
        lastSequence = latest.sequence;

        // The P controller requires a fresh numeric range. Completed sensor
        // error/no-target observations may help mapping, but cannot control a
        // physical wall gap safely.
        if (!latest.rangeValid
            || !FrontLidar.hasFreshReading(FRONT_READING_MAX_AGE_MS)
            || latest.distanceMm > geometry::FRONT_WALL_MAX_MM) {
            LeftMotor.setPWM(0);
            RightMotor.setPWM(0);
            zeroSince = 0;
            zeroSamples = 0;
            delay(LOOP_DELAY_MS);
            continue;
        }

        const float leftPosition = L_ENC_FWD * LeftEncoder.getRotation();
        const float rightPosition = R_ENC_FWD * RightEncoder.getRotation();
        const float leftTravelMm = (leftPosition - leftStart) * WHEEL_RADIUS_MM;
        const float rightTravelMm = (rightPosition - rightStart) * WHEEL_RADIUS_MM;
        if (fabs(leftTravelMm) > WALL_ADJUST_MAX_TRAVEL_MM
            || fabs(rightTravelMm) > WALL_ADJUST_MAX_TRAVEL_MM) {
            break;
        }

        const float wallError = static_cast<float>(latest.distanceMm)
                              - WALL_ADJUST_TARGET_MM;
        if (fabs(wallError) <= WALL_ADJUST_TOLERANCE_MM) {
            // Zero is commanded explicitly inside the target band. Require it
            // to remain zero across fresh samples and a settling interval; a
            // timeout stop below is therefore not mistaken for convergence.
            LeftMotor.setPWM(0);
            RightMotor.setPWM(0);
            if (zeroSince == 0) {
                zeroSince = millis();
                zeroLeftStart = leftPosition;
                zeroRightStart = rightPosition;
                zeroSamples = 1;
            } else if (zeroSamples < UINT8_MAX) {
                ++zeroSamples;
            }

            if (millis() - zeroSince >= WALL_ADJUST_ZERO_SETTLE_MS
                && zeroSamples >= WALL_ADJUST_ZERO_SAMPLES
                && fabs(leftPosition - zeroLeftStart)
                    <= MAP_SCAN_MAX_ENCODER_RAD
                && fabs(rightPosition - zeroRightStart)
                    <= MAP_SCAN_MAX_ENCODER_RAD
                && fabs(imu.getYawRate()) <= MAP_SCAN_MAX_YAW_RATE_DPS) {
                stop();
                return;
            }
            if (millis() - zeroSince >= WALL_ADJUST_ZERO_SETTLE_MS) {
                // Coasting invalidates this zero-PWM window. Start a new one
                // from the current resting candidate instead of allowing an
                // early encoder delta to prevent convergence until timeout.
                zeroSince = millis();
                zeroLeftStart = leftPosition;
                zeroRightStart = rightPosition;
                zeroSamples = 1;
            }
            delay(LOOP_DELAY_MS);
            continue;
        }

        zeroSince = 0;
        zeroSamples = 0;
        const int direction = wallError > 0.0f ? 1 : -1;
        const int basePwm = constrain(
            static_cast<int>(fabs(wallError) * WALL_ADJUST_KP),
            WALL_ADJUST_MIN_PWM,
            WALL_ADJUST_MAX_PWM);
        const float headingError = wrapAngle180(
            correctedHeading() - nominalHeading);
        const int steering = static_cast<int>(constrain(
            KP_HOLD * headingError,
            -MAX_STEERING_PWM,
            MAX_STEERING_PWM));
        const int signedBasePwm = direction * basePwm;
        const int leftCommand = constrain(
            signedBasePwm + steering, -255, 255);
        const int rightCommand = constrain(
            signedBasePwm - steering, -255, 255);

        LeftMotor.setPWM(L_MOTOR_FWD * leftCommand);
        RightMotor.setPWM(R_MOTOR_FWD * rightCommand);
        delay(LOOP_DELAY_MS);
    }

    stop();
}

bool Movement::observeWalls(WallObservation& observation) {
    observation = WallObservation{};
    stop();

    const float leftStillStart = L_ENC_FWD * LeftEncoder.getRotation();
    const float rightStillStart = R_ENC_FWD * RightEncoder.getRotation();
    const uint32_t stillStartedAt = millis();
    uint8_t imuMisses = 0;
    while (millis() - stillStartedAt < MAP_SCAN_STILL_MS) {
        if (imu.update()) {
            imuMisses = 0;
        } else if (++imuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            return false;
        }
        serviceNextLidar();
        delay(LOOP_DELAY_MS);
    }

    const float leftStillDelta = L_ENC_FWD * LeftEncoder.getRotation()
                               - leftStillStart;
    const float rightStillDelta = R_ENC_FWD * RightEncoder.getRotation()
                                - rightStillStart;
    if (fabs(leftStillDelta) > MAP_SCAN_MAX_ENCODER_RAD
        || fabs(rightStillDelta) > MAP_SCAN_MAX_ENCODER_RAD
        || fabs(imu.getYawRate()) > MAP_SCAN_MAX_YAW_RATE_DPS) {
        return false;
    }

    SensorVotes front;
    SensorVotes left;
    SensorVotes right;
    LidarObservation latest;
    if (FrontLidar.getLatestObservation(latest)) front.sequence = latest.sequence;
    if (LeftLidar.getLatestObservation(latest)) left.sequence = latest.sequence;
    if (RightLidar.getLatestObservation(latest)) right.sequence = latest.sequence;

    const uint32_t scanStartedAt = millis();
    while (millis() - scanStartedAt < MAP_SCAN_TIMEOUT_MS) {
        if (!imu.update()) {
            if (++imuMisses >= MAX_CONSECUTIVE_IMU_MISSES) return false;
        } else {
            imuMisses = 0;
        }
        serviceNextLidar();
        countObservation(FrontLidar, geometry::FRONT_WALL_MAX_MM, front);
        countObservation(LeftLidar, geometry::SIDE_WALL_MAX_MM, left);
        countObservation(RightLidar, geometry::SIDE_WALL_MAX_MM, right);
        if (front.observations >= MAP_SCAN_SAMPLES
            && left.observations >= MAP_SCAN_SAMPLES
            && right.observations >= MAP_SCAN_SAMPLES) {
            break;
        }
        delay(LOOP_DELAY_MS);
    }

    observation.front = consensus(front);
    observation.left = consensus(left);
    observation.right = consensus(right);
    const float finalLeftStillDelta = L_ENC_FWD * LeftEncoder.getRotation()
                                    - leftStillStart;
    const float finalRightStillDelta = R_ENC_FWD * RightEncoder.getRotation()
                                     - rightStillStart;
    const bool remainedStationary =
        fabs(finalLeftStillDelta) <= MAP_SCAN_MAX_ENCODER_RAD
        && fabs(finalRightStillDelta) <= MAP_SCAN_MAX_ENCODER_RAD
        && fabs(imu.getYawRate()) <= MAP_SCAN_MAX_YAW_RATE_DPS;
    const bool hasUsableSensor =
        front.observations >= MAP_SCAN_SAMPLES
        || left.observations >= MAP_SCAN_SAMPLES
        || right.observations >= MAP_SCAN_SAMPLES;
    return remainedStationary && hasUsableSensor;
}

bool Movement::sampleThreeWallRanges(
    float& frontMm,
    float& leftMm,
    float& rightMm) {
    stop();

    uint16_t frontSamples[LANDMARK_RANGE_SAMPLES];
    uint16_t leftSamples[LANDMARK_RANGE_SAMPLES];
    uint16_t rightSamples[LANDMARK_RANGE_SAMPLES];
    uint8_t frontCount = 0;
    uint8_t leftCount = 0;
    uint8_t rightCount = 0;
    uint16_t frontSequence = 0;
    uint16_t leftSequence = 0;
    uint16_t rightSequence = 0;
    LidarObservation latest;
    if (FrontLidar.getLatestObservation(latest)) {
        frontSequence = latest.sequence;
    }
    if (LeftLidar.getLatestObservation(latest)) {
        leftSequence = latest.sequence;
    }
    if (RightLidar.getLatestObservation(latest)) {
        rightSequence = latest.sequence;
    }

    uint8_t consecutiveImuMisses = 0;
    const uint32_t startedAt = millis();
    while (millis() - startedAt < LANDMARK_SAMPLE_TIMEOUT_MS) {
        if (imu.update()) {
            consecutiveImuMisses = 0;
        } else if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            return false;
        }

        serviceNextLidar();
        recordRangeSample(
            FrontLidar, frontSequence, frontSamples, frontCount);
        recordRangeSample(
            LeftLidar, leftSequence, leftSamples, leftCount);
        recordRangeSample(
            RightLidar, rightSequence, rightSamples, rightCount);
        if (frontCount >= LANDMARK_RANGE_SAMPLES
            && leftCount >= LANDMARK_RANGE_SAMPLES
            && rightCount >= LANDMARK_RANGE_SAMPLES) {
            frontMm = medianRange(frontSamples);
            leftMm = medianRange(leftSamples);
            rightMm = medianRange(rightSamples);
            return true;
        }
        delay(LOOP_DELAY_MS);
    }
    return false;
}

bool Movement::alignAndRecalibrateFromThreeWalls() {
    const float nominalHeading = heading;
    float centreFront = 0.0f;
    float centreLeft = 0.0f;
    float centreRight = 0.0f;
    if (!sampleThreeWallRanges(
            centreFront, centreLeft, centreRight)) {
        // The caller already has classified wall observations. Raw numeric
        // ranges are deliberately stricter; inability to align is not a motion
        // fault while the robot has remained at the original heading.
        return true;
    }

    if (!turnByImu(LANDMARK_PROBE_DEG, true, false)) return false;
    float leftProbeFront = 0.0f;
    float leftProbeLeft = 0.0f;
    float leftProbeRight = 0.0f;
    if (!sampleThreeWallRanges(
            leftProbeFront, leftProbeLeft, leftProbeRight)) {
        return turnByImu(-LANDMARK_PROBE_DEG, true, false);
    }

    if (!turnByImu(-2.0f * LANDMARK_PROBE_DEG, true, false)) return false;
    float rightProbeFront = 0.0f;
    float rightProbeLeft = 0.0f;
    float rightProbeRight = 0.0f;
    if (!sampleThreeWallRanges(
            rightProbeFront, rightProbeLeft, rightProbeRight)) {
        return turnByImu(LANDMARK_PROBE_DEG, true, false);
    }

    // Return to the entry orientation before deciding whether the range curve
    // is trustworthy. This makes every rejected calibration non-destructive.
    if (!turnByImu(LANDMARK_PROBE_DEG, true, false)) return false;

    const float curvature = rightProbeFront
                          - 2.0f * centreFront
                          + leftProbeFront;
    if (curvature < LANDMARK_MIN_CURVATURE_MM) {
        // A stationary bias refresh is still useful, but without a clear range
        // minimum there is no absolute angle observation to anchor against.
        if (calibrateYawRateBias(1000)) yawRateBiasValid = true;
        return true;
    }

    const float correctionDeg = LANDMARK_PROBE_DEG
        * (rightProbeFront - leftProbeFront)
        / (2.0f * curvature);
    if (!isfinite(correctionDeg)
        || fabs(correctionDeg) > LANDMARK_MAX_CORRECTION_DEG) {
        if (calibrateYawRateBias(1000)) yawRateBiasValid = true;
        return true;
    }

    if (!turnByImu(correctionDeg, true, false)) return false;

    float alignedFront = 0.0f;
    float alignedLeft = 0.0f;
    float alignedRight = 0.0f;
    if (!sampleThreeWallRanges(
            alignedFront, alignedLeft, alignedRight)
        || alignedFront
            > centreFront + LANDMARK_MAX_VALIDATION_INCREASE_MM) {
        // Restore the estimator-consistent entry orientation if the proposed
        // minimum did not validate. A failed restoration is a real heading
        // fault and must stop autonomous mapping.
        return turnByImu(-correctionDeg, true, false);
    }

    // The range minimum supplies the absolute cardinal direction. Refresh the
    // zero-rate bias only while stationary, then make the physically aligned
    // pose equal the logical heading that mapping had on entry. If the optional
    // bias refresh fails, retain the old estimator reference; the validated
    // physical alignment is still harmless and later motion remains relative.
    if (!calibrateYawRateBias(1000) || !imu.update()) return true;
    yawRateBiasValid = true;
    heading = nominalHeading;
    headingReference = YAW_CCW_SIGN * imu.getYaw() - heading;
    return true;
}

bool Movement::recalibrateAtCurrentHeading() {
    stop();
    // A failed refresh must not invalidate the bias that already carried the
    // robot through the preceding route. Task 4.2 may deliberately fall back
    // to that last known-good calibration after bounded refresh attempts.
    if (!calibrateYawRateBias(1000) || !imu.update()) return false;
    yawRateBiasValid = true;

    // Preserve the logical heading while discarding accumulated stationary
    // gyro drift before a coordinate course or final shortest-path run.
    headingReference = YAW_CCW_SIGN * imu.getYaw() - heading;
    return true;
}

void Movement::showMap(
    const Maze& maze,
    const Pose& pose,
    const Pose& start,
    uint8_t goalRow,
    uint8_t goalColumn,
    MappingPhase phase) {
    stop();
    display.showMap(maze, pose, start, goalRow, goalColumn, phase);
}

bool Movement::turnByImu(
    float signedAngleDeg,
    bool serviceLidarsAfterTurn,
    bool task41FastProfile) {
    if (!yawRateBiasValid
        || !isfinite(signedAngleDeg)
        || fabs(signedAngleDeg) > 180.0f) {
        stop();
        return false;
    }

    if (!imu.update()) {
        stop();
        return false;
    }
    const float targetHeading = heading + signedAngleDeg;
    turnPIDController.zeroAndSetTarget(headingReference, targetHeading);

    bool success = false;
    uint32_t settleSince = 0;
    uint8_t consecutiveImuMisses = 0;
    const uint32_t startedAt = millis();

    while (millis() - startedAt <= TURN_TIMEOUT_MS) {
        if (!imu.update()) {
            LeftMotor.setPWM(0);
            RightMotor.setPWM(0);
            settleSince = 0;
            if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) break;
            delay(LOOP_DELAY_MS);
            continue;
        }
        consecutiveImuMisses = 0;

        const float output = turnPIDController.compute(
            YAW_CCW_SIGN * imu.getYaw());
        const float error = turnPIDController.getError();
        const bool inBand = fabs(error) < TURN_TOL_DEG;
        const bool stopped = fabs(imu.getYawRate()) < TURN_STILL_DPS;
        const int spin = turnCommand(output, inBand);

        LeftMotor.setPWM(L_MOTOR_FWD * -spin);
        RightMotor.setPWM(R_MOTOR_FWD * spin);

        if (inBand && stopped) {
            if (settleSince == 0) settleSince = millis();
            if (millis() - settleSince >= TURN_SETTLE_MS) {
                success = true;
                break;
            }
        } else {
            settleSince = 0;
        }
        delay(LOOP_DELAY_MS);
    }

    stop();
    bool restValid = true;
    if (serviceLidarsAfterTurn) {
        settle(task41FastProfile ? TASK41_TURN_POST_SETTLE_MS : 250);
    } else {
        restValid = settleImuOnly(250);
    }
    if (!success || !restValid) return false;

    heading = targetHeading;
    return true;
}

bool Movement::rotateLeft(float angleDeg) {
    return turnByImu(angleDeg, true, false);
}

bool Movement::rotateRight(float angleDeg) {
    return turnByImu(-angleDeg, true, false);
}

bool Movement::rotateLeftTask41(float angleDeg) {
    return turnByImu(angleDeg, true, true);
}

bool Movement::rotateRightTask41(float angleDeg) {
    return turnByImu(-angleDeg, true, true);
}

bool Movement::rotateLeftDeadReckoned(float angleDeg) {
    return turnByImu(angleDeg, false, false);
}

bool Movement::rotateRightDeadReckoned(float angleDeg) {
    return turnByImu(-angleDeg, false, false);
}

void Movement::adoptCurrentHeading() {
    stop();
    const float currentHeading = correctedHeading();
    if (isfinite(currentHeading)) heading = currentHeading;
}

bool Movement::settleImuOnly(uint16_t durationMs) {
    stop();
    uint8_t consecutiveImuMisses = 0;
    bool receivedSample = false;
    const uint32_t startedAt = millis();
    while (millis() - startedAt < durationMs) {
        if (imu.update()) {
            receivedSample = true;
            consecutiveImuMisses = 0;
        } else if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            return false;
        }
        delay(LOOP_DELAY_MS);
    }
    return receivedSample;
}

void Movement::settle(uint16_t durationMs) {
    stop();
    const uint32_t startedAt = millis();
    while (millis() - startedAt < durationMs) {
        imu.update();
        serviceNextLidar();
        delay(LOOP_DELAY_MS);
    }
}

bool Movement::calibrateYawRateBias(uint16_t durationMs) {
    stop();
    const uint32_t startedAt = millis();
    const float leftStart = L_ENC_FWD * LeftEncoder.getRotation();
    const float rightStart = R_ENC_FWD * RightEncoder.getRotation();
    float rateSum = 0.0f;
    float rateSquaredSum = 0.0f;
    uint16_t rateSamples = 0;
    uint8_t invalidSamples = 0;
    bool motionDetected = false;

    while (millis() - startedAt < durationMs) {
        const bool valid = imu.update();
        if (!valid && invalidSamples < UINT8_MAX) ++invalidSamples;

        const float leftDelta = L_ENC_FWD * LeftEncoder.getRotation() - leftStart;
        const float rightDelta = R_ENC_FWD * RightEncoder.getRotation() - rightStart;
        if (fabs(leftDelta) > BIAS_MAX_ENCODER_DELTA_RAD
            || fabs(rightDelta) > BIAS_MAX_ENCODER_DELTA_RAD) {
            motionDetected = true;
        }

        if (valid) {
            const float rawRate = imu.getRawYawRate();
            if (millis() - startedAt >= BIAS_SAMPLE_GUARD_MS) {
                rateSum += rawRate;
                rateSquaredSum += rawRate * rawRate;
                ++rateSamples;
            }
        }
        delay(LOOP_DELAY_MS);
    }

    if (invalidSamples > BIAS_MAX_INVALID_SAMPLES
        || rateSamples < BIAS_MIN_SAMPLES
        || motionDetected) {
        return false;
    }

    const float mean = rateSum / rateSamples;
    float variance = rateSquaredSum / rateSamples - mean * mean;
    if (variance < 0.0f) variance = 0.0f;
    if (variance > BIAS_MAX_VARIANCE) return false;

    imu.setYawRateBias(mean);
    return true;
}

void Movement::stop() {
    // Stopping must never depend on I2C; a wedged sensor bus must not prevent
    // the PWM outputs from being cleared.
    LeftMotor.setPWM(0);
    RightMotor.setPWM(0);
}

}  // namespace mtrn3100
