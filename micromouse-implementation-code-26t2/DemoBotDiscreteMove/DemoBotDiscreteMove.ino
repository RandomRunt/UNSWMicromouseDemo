#include <Wire.h>
#include <avr/pgmspace.h>

#include "DualEncoder.hpp"
#include "Imu.hpp"
#include "Lidar.hpp"
#include "Motor.hpp"

// DemoBot wiring. A0-A2 are the lidar XSHUT pins, not analogue readings.
constexpr uint8_t LEFT_MOTOR_PWM = 11;
constexpr uint8_t LEFT_MOTOR_DIR = 12;
constexpr uint8_t RIGHT_MOTOR_PWM = 9;
constexpr uint8_t RIGHT_MOTOR_DIR = 10;
constexpr uint8_t LEFT_ENCODER_A = 2;
constexpr uint8_t LEFT_ENCODER_B = 7;
constexpr uint8_t RIGHT_ENCODER_A = 3;
constexpr uint8_t RIGHT_ENCODER_B = 8;
constexpr uint8_t LEFT_LIDAR_XSHUT = A0;
constexpr uint8_t FRONT_LIDAR_XSHUT = A1;
constexpr uint8_t RIGHT_LIDAR_XSHUT = A2;

// Hardware/maze tuning retained from DemoBot-Task4.1V2.
constexpr float WHEEL_RADIUS_MM = 17.1f; // was 16.0f
constexpr float CELL_DISTANCE_MM = 180.0f;
constexpr float SIDE_WALL_TARGET_MM = 56.0f;
constexpr float SIDE_WALL_MAX_MM = 80.0f;
constexpr float FRONT_TARGET_MM = 54.0f;

// BestDiscreteMove-style forward controller.
constexpr int MIN_MOVE_PWM = 125;
constexpr int CRUISE_PWM = 175;
constexpr float DECEL_PWM_PER_MM = 0.9f;
constexpr int PWM_RISE_STEP = 8;
constexpr int PWM_FALL_STEP = 16;
constexpr float HEADING_KP = 8.0f;
constexpr float LIDAR_HEADING_KP = 0.25f;
constexpr float LIDAR_HEADING_ALPHA = 0.25f;
constexpr float MAX_LIDAR_HEADING_DEG = 8.0f;
constexpr float CENTRE_DEADBAND_MM = 5.0f;
constexpr float MAX_STEERING_PWM = 50.0f;
constexpr float DISTANCE_TOLERANCE_MM = 5.0f;

// Front-wall arrival is enabled only near the expected encoder endpoint.
constexpr float FRONT_ENABLE_REMAINING_MM = 30.0f;
constexpr float FRONT_MATCH_TOLERANCE_MM = 20.0f;
constexpr float FRONT_TOLERANCE_MM = 3.0f;
constexpr float FRONT_EMERGENCY_MM = 45.0f;
constexpr float FRONT_MAX_VALID_MM = 100.0f;
constexpr float FRONT_MAX_EXTRA_TRAVEL_MM = 20.0f;
constexpr uint8_t FRONT_CONFIRM_READINGS = 2;

// BestDiscreteMove-style settled IMU turns.
constexpr float TURN_KP = 6.0f;
constexpr float TURN_KD = 0.3f;
constexpr int TURN_MIN_PWM = 20;
constexpr int TURN_MAX_PWM = 120;
constexpr float TURN_TOLERANCE_DEG = 1.0f;
constexpr float TURN_STILL_DPS = 2.0f;
constexpr uint16_t TURN_STABLE_MS = 100;

constexpr float BIAS_MAX_VARIANCE = 0.25f;
constexpr float BIAS_MAX_ENCODER_DELTA_RAD = 0.02f;
constexpr uint16_t BIAS_SAMPLE_GUARD_MS = 50;
constexpr uint16_t BIAS_MIN_SAMPLES = 20;
constexpr uint8_t BIAS_MAX_INVALID_SAMPLES = 5;

constexpr uint32_t MOVE_TIMEOUT_MS = 9000;
constexpr uint32_t TURN_TIMEOUT_MS = 7000;
constexpr uint8_t MAX_CONSECUTIVE_IMU_MISSES = 30;
constexpr uint16_t LIDAR_PERIOD_MS = 60;
constexpr uint16_t LIDAR_SLOT_MS = LIDAR_PERIOD_MS / 3;
constexpr uint16_t LIDAR_MAX_AGE_MS = 150;
constexpr uint8_t LOOP_DELAY_MS = 5;

// Task 4.2 uses only encoder distance and IMU heading. Lidar control is kept
// disabled so cylindrical obstacles cannot be mistaken for maze walls.
constexpr float TASK42_MAX_LEG_MM = 1600.0f;
constexpr float TASK42_WHEEL_RADIUS_MM = WHEEL_RADIUS_MM;
constexpr uint16_t TASK42_MAX_TURN_TENTHS = 1800;
constexpr uint16_t TASK42_MAX_DISTANCE_TENTHS = 16000;
constexpr uint8_t TASK42_MAX_MOVES = 64;
constexpr int TASK42_CRUISE_PWM = 150;
constexpr int TASK42_APPROACH_PWM = 140;
constexpr int TASK42_MAX_PWM = 170;
constexpr int TASK42_MAX_STEERING_PWM = 20;
constexpr float TASK42_APPROACH_DISTANCE_MM = 35.0f;
constexpr float TASK42_STOP_LEAD_MM = 1.0f;
constexpr uint16_t TASK42_ARRIVAL_SETTLE_MS = 180;
constexpr uint16_t TASK42_ENTRY_SETTLE_MS = 400;
constexpr uint16_t TASK42_RECALIBRATION_SETTLE_MS = 250;
constexpr uint8_t TASK42_RECALIBRATION_ATTEMPTS = 3;

// Normal commands are f/l/r. A Task 4.2 tuple is (clockwise turn degrees,
// forward millimetres); negative angles turn left. Commas separate modes.
// const char ROUTE[] PROGMEM = "fr,[(58.1,287.4),(-3.9,116.7),(-64.3,377.6),],fl";
// const char ROUTE[] PROGMEM = "frfflflfflfflflfrfrfrffrffrfrfrlflflfflflfrfrfrffrfrflfrf";
// const char ROUTE[] PROGMEM = "flflfrfrflflfrfflfrflflfflfrfrflflfrfrffrfffff"; // Left side of maze
// const char ROUTE[] PROGMEM = "frfrflflfrfrflffrflfrfrffrflflfrfrflflfflfffff"; // Right side of maze

// Robotics@UNSW showcase
const char ROUTE[] PROGMEM = "frflfrflfrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrffrfrflflfrfrflffrflfrfrffrflflfrfrflffrf"; // infiniteish loop
// const char ROUTE[] PROGMEM = "frflfrflfrfrflflfrfrflffrflfrfrffrflflfrfrflffrf"; // 1 loop of the maze

struct Task42Move {
    int16_t turnTenths;
    uint16_t distanceTenths;
};

Motor leftMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR);
Motor rightMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR);
DualEncoder encoders(
    LEFT_ENCODER_A,
    LEFT_ENCODER_B,
    RIGHT_ENCODER_A,
    RIGHT_ENCODER_B);
Lidar leftLidar(LEFT_LIDAR_XSHUT, 0x54);
Lidar frontLidar(FRONT_LIDAR_XSHUT, 0x56);
Lidar rightLidar(RIGHT_LIDAR_XSHUT, 0x58);
Imu imu;
uint32_t lastLidarPollMs = 0;
uint8_t nextLidar = 0;
float committedHeadingDeg = 0.0f;
bool ready = false;
bool routeRun = false;

char routeAt(uint16_t index) {
    if (index >= sizeof(ROUTE)) return '\0';
    return static_cast<char>(pgm_read_byte(ROUTE + index));
}

float wrapAngle180(float angle) {
    angle = fmod(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

int slewPWM(int current, int target) {
    if (target > current) return min(target, current + PWM_RISE_STEP);
    return max(target, current - PWM_FALL_STEP);
}

void drive(int leftPWM, int rightPWM) {
    // DemoBot's two motors are mounted with opposite electrical polarity.
    leftMotor.setPWM(-leftPWM);
    rightMotor.setPWM(rightPWM);
}

void stopMotors() {
    drive(0, 0);
}

void serviceLidar() {
    const uint32_t now = millis();
    if (now - lastLidarPollMs < LIDAR_SLOT_MS) return;
    lastLidarPollMs = now;

    if (nextLidar == 0) {
        frontLidar.updateDistance();
    } else if (nextLidar == 1) {
        leftLidar.updateDistance();
    } else {
        rightLidar.updateDistance();
    }
    nextLidar = (nextLidar + 1U) % 3U;
}

void settle(uint16_t durationMs) {
    stopMotors();
    const uint32_t startedAt = millis();
    while (millis() - startedAt < durationMs) {
        imu.update();
        serviceLidar();
        delay(LOOP_DELAY_MS);
    }
}

bool settleImuOnly(uint16_t durationMs) {
    stopMotors();
    bool receivedSample = false;
    uint8_t consecutiveMisses = 0;
    const uint32_t startedAt = millis();
    while (millis() - startedAt < durationMs) {
        if (imu.update()) {
            receivedSample = true;
            consecutiveMisses = 0;
        } else if (++consecutiveMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            return false;
        }
        delay(LOOP_DELAY_MS);
    }
    return receivedSample;
}

bool calibrateYawRateBias(uint16_t durationMs) {
    stopMotors();
    const uint32_t startedAt = millis();
    const float leftStart = encoders.leftRotation();
    const float rightStart = encoders.rightRotation();
    float rateSum = 0.0f;
    float rateSquaredSum = 0.0f;
    uint16_t samples = 0;
    uint8_t invalidSamples = 0;
    bool motionDetected = false;

    while (millis() - startedAt < durationMs) {
        const bool valid = imu.update();
        if (!valid && invalidSamples < UINT8_MAX) ++invalidSamples;
        if (fabs(encoders.leftRotation() - leftStart)
                > BIAS_MAX_ENCODER_DELTA_RAD
            || fabs(encoders.rightRotation() - rightStart)
                > BIAS_MAX_ENCODER_DELTA_RAD) {
            motionDetected = true;
        }

        if (valid && millis() - startedAt >= BIAS_SAMPLE_GUARD_MS) {
            const float rate = imu.getRawYawRate();
            rateSum += rate;
            rateSquaredSum += rate * rate;
            ++samples;
        }
        delay(LOOP_DELAY_MS);
    }

    if (invalidSamples > BIAS_MAX_INVALID_SAMPLES
        || samples < BIAS_MIN_SAMPLES
        || motionDetected) {
        return false;
    }
    const float mean = rateSum / samples;
    float variance = rateSquaredSum / samples - mean * mean;
    if (variance < 0.0f) variance = 0.0f;
    if (variance > BIAS_MAX_VARIANCE) return false;
    imu.setYawRateBias(mean);
    return true;
}

bool initialiseHardware() {
    stopMotors();
    Wire.begin();
    Wire.setClock(100000UL);
    Wire.setWireTimeout(5000UL, true);
    Wire.clearWireTimeoutFlag();

    pinMode(LEFT_LIDAR_XSHUT, OUTPUT);
    pinMode(FRONT_LIDAR_XSHUT, OUTPUT);
    pinMode(RIGHT_LIDAR_XSHUT, OUTPUT);
    digitalWrite(LEFT_LIDAR_XSHUT, LOW);
    digitalWrite(FRONT_LIDAR_XSHUT, LOW);
    digitalWrite(RIGHT_LIDAR_XSHUT, LOW);
    delay(100);

    if (!imu.initialise()) return false;
    if (!leftLidar.initialise()
        || !leftLidar.startContinuous(LIDAR_PERIOD_MS)) return false;
    delay(LIDAR_SLOT_MS);
    if (!frontLidar.initialise()
        || !frontLidar.startContinuous(LIDAR_PERIOD_MS)) return false;
    delay(LIDAR_SLOT_MS);
    if (!rightLidar.initialise()
        || !rightLidar.startContinuous(LIDAR_PERIOD_MS)) return false;

    lastLidarPollMs = millis();
    if (!calibrateYawRateBias(1000) || !imu.update()) return false;
    committedHeadingDeg = imu.getYaw();
    return true;
}

bool moveForward(float distanceMm) {
    if (distanceMm <= 0.0f) return false;

    const float leftStart = -encoders.leftRotation();
    const float rightStart = encoders.rightRotation();
    const float nominalHeading = committedHeadingDeg;
    const uint32_t startedAt = millis();
    uint32_t lastWallUpdateMs = 0;
    uint16_t processedFrontSequence = frontLidar.getSequence();
    float lidarHeadingOffset = 0.0f;
    int appliedLeftPWM = 0;
    int appliedRightPWM = 0;
    uint8_t closeFrontReadings = 0;
    uint8_t consecutiveImuMisses = 0;
    bool frontApproachActive = false;

    while (millis() - startedAt < MOVE_TIMEOUT_MS) {
        if (imu.update()) {
            consecutiveImuMisses = 0;
        } else if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            stopMotors();
            return false;
        }

        const float leftTravel = -encoders.leftRotation() - leftStart;
        const float rightTravel = encoders.rightRotation() - rightStart;
        const float travelledMm = (leftTravel + rightTravel)
                                * 0.5f * WHEEL_RADIUS_MM;
        const float remainingMm = distanceMm - travelledMm;

        const float frontDistance = frontLidar.getDistance();
        const bool frontUsable = frontLidar.hasFreshReading(LIDAR_MAX_AGE_MS)
                              && frontDistance <= FRONT_MAX_VALID_MM;
        if (remainingMm <= FRONT_ENABLE_REMAINING_MM
            && frontUsable
            && frontLidar.getSequence() != processedFrontSequence) {
            processedFrontSequence = frontLidar.getSequence();
            const float expected = FRONT_TARGET_MM + max(remainingMm, 0.0f);
            if (!frontApproachActive
                && fabs(frontDistance - expected)
                    <= FRONT_MATCH_TOLERANCE_MM) {
                frontApproachActive = true;
            }

            if (frontApproachActive
                && frontDistance <= FRONT_TARGET_MM + FRONT_TOLERANCE_MM) {
                if (frontDistance <= FRONT_EMERGENCY_MM) {
                    closeFrontReadings = FRONT_CONFIRM_READINGS;
                } else if (closeFrontReadings < FRONT_CONFIRM_READINGS) {
                    ++closeFrontReadings;
                }
            } else if (frontApproachActive) {
                closeFrontReadings = 0;
            }
        }

        if (closeFrontReadings >= FRONT_CONFIRM_READINGS) {
            stopMotors();
            return true;
        }
        if (closeFrontReadings > 0) {
            stopMotors();
            serviceLidar();
            delay(LOOP_DELAY_MS);
            continue;
        }
        if (frontApproachActive && !frontUsable) frontApproachActive = false;

        if ((!frontApproachActive
             && remainingMm <= DISTANCE_TOLERANCE_MM)
            || (frontApproachActive
                && travelledMm
                    >= distanceMm + FRONT_MAX_EXTRA_TRAVEL_MM)) {
            stopMotors();
            return !frontApproachActive;
        }

        int basePWM = constrain(
            MIN_MOVE_PWM
                + static_cast<int>(max(remainingMm, 0.0f) * DECEL_PWM_PER_MM),
            MIN_MOVE_PWM,
            CRUISE_PWM);
        if (remainingMm <= 10.0f) basePWM = 50;

        const uint32_t now = millis();
        if (now - lastWallUpdateMs >= LIDAR_PERIOD_MS) {
            lastWallUpdateMs = now;
            const float leftDistance = leftLidar.getDistance();
            const float rightDistance = rightLidar.getDistance();
            const bool leftWall = leftLidar.hasFreshReading(LIDAR_MAX_AGE_MS)
                               && leftDistance <= SIDE_WALL_MAX_MM;
            const bool rightWall = rightLidar.hasFreshReading(LIDAR_MAX_AGE_MS)
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
                -LIDAR_HEADING_KP * centreError,
                -MAX_LIDAR_HEADING_DEG,
                MAX_LIDAR_HEADING_DEG);
            lidarHeadingOffset += LIDAR_HEADING_ALPHA
                                * (requestedOffset - lidarHeadingOffset);
        }

        const float desiredHeading = nominalHeading + lidarHeadingOffset;
        const float headingError = wrapAngle180(
            imu.getYaw() - desiredHeading);
        const int steering = static_cast<int>(constrain(
            HEADING_KP * headingError,
            -MAX_STEERING_PWM,
            MAX_STEERING_PWM));
        const int targetLeftPWM = constrain(basePWM + steering, 0, 255);
        const int targetRightPWM = constrain(basePWM - steering, 0, 255);

        appliedLeftPWM = slewPWM(appliedLeftPWM, targetLeftPWM);
        appliedRightPWM = slewPWM(appliedRightPWM, targetRightPWM);
        drive(appliedLeftPWM, appliedRightPWM);
        serviceLidar();
        delay(LOOP_DELAY_MS);
    }

    stopMotors();
    return false;
}

bool moveDeadReckoned(float distanceMm) {
    if (!isfinite(distanceMm)
        || distanceMm <= 0.0f
        || distanceMm > TASK42_MAX_LEG_MM) {
        stopMotors();
        return false;
    }

    const float leftStart = -encoders.leftRotation();
    const float rightStart = encoders.rightRotation();
    const float nominalHeading = committedHeadingDeg;
    const uint32_t startedAt = millis();
    int appliedLeftPWM = 0;
    int appliedRightPWM = 0;
    uint8_t consecutiveImuMisses = 0;
    bool arrived = false;

    while (millis() - startedAt < MOVE_TIMEOUT_MS) {
        if (imu.update()) {
            consecutiveImuMisses = 0;
        } else if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) {
            break;
        }

        const float leftTravel = -encoders.leftRotation() - leftStart;
        const float rightTravel = encoders.rightRotation() - rightStart;
        const float travelledMm = (leftTravel + rightTravel)
                                * 0.5f * TASK42_WHEEL_RADIUS_MM;
        const float remainingMm = distanceMm - travelledMm;
        if (remainingMm <= TASK42_STOP_LEAD_MM) {
            arrived = true;
            break;
        }

        const int basePWM = remainingMm <= TASK42_APPROACH_DISTANCE_MM
                          ? TASK42_APPROACH_PWM
                          : TASK42_CRUISE_PWM;
        const float headingError = wrapAngle180(
            imu.getYaw() - nominalHeading);
        const int steering = constrain(
            static_cast<int>(HEADING_KP * headingError),
            -TASK42_MAX_STEERING_PWM,
            TASK42_MAX_STEERING_PWM);

        // Keep both wheels above the reliable motor floor. Correct heading by
        // accelerating only the outside wheel rather than slowing one wheel.
        int targetLeftPWM = basePWM;
        int targetRightPWM = basePWM;
        if (steering > 0) {
            targetLeftPWM += steering;
        } else {
            targetRightPWM -= steering;
        }
        targetLeftPWM = min(targetLeftPWM, TASK42_MAX_PWM);
        targetRightPWM = min(targetRightPWM, TASK42_MAX_PWM);

        appliedLeftPWM = slewPWM(appliedLeftPWM, targetLeftPWM);
        appliedRightPWM = slewPWM(appliedRightPWM, targetRightPWM);
        drive(appliedLeftPWM, appliedRightPWM);
        // Deliberately no lidar transactions during Task 4.2 movement.
        delay(LOOP_DELAY_MS);
    }

    stopMotors();
    return settleImuOnly(TASK42_ARRIVAL_SETTLE_MS) && arrived;
}

bool turnTo(float targetHeadingDeg, bool serviceLidars) {
    if (!imu.update()) {
        stopMotors();
        return false;
    }
    float previousError = wrapAngle180(targetHeadingDeg - imu.getYaw());
    uint32_t previousTimeUs = micros();
    uint32_t stableSinceMs = 0;
    const uint32_t startedAt = millis();
    uint8_t consecutiveImuMisses = 0;

    while (millis() - startedAt < TURN_TIMEOUT_MS) {
        if (!imu.update()) {
            stopMotors();
            stableSinceMs = 0;
            if (++consecutiveImuMisses >= MAX_CONSECUTIVE_IMU_MISSES) break;
            delay(LOOP_DELAY_MS);
            continue;
        }
        consecutiveImuMisses = 0;
        const uint32_t nowUs = micros();
        const float dt = static_cast<float>(nowUs - previousTimeUs) / 1000000.0f;
        previousTimeUs = nowUs;
        const float error = wrapAngle180(targetHeadingDeg - imu.getYaw());
        const float derivative = dt > 0.0f ? (error - previousError) / dt : 0.0f;
        previousError = error;

        const bool inBand = fabs(error) < TURN_TOLERANCE_DEG;
        int spinPWM = 0;
        if (!inBand) {
            const float output = TURN_KP * error + TURN_KD * derivative;
            const int magnitude = constrain(
                static_cast<int>(fabs(output)), TURN_MIN_PWM, TURN_MAX_PWM);
            spinPWM = output < 0.0f ? -magnitude : magnitude;
        }
        drive(-spinPWM, spinPWM);

        if (inBand && fabs(imu.getYawRate()) < TURN_STILL_DPS) {
            if (stableSinceMs == 0) stableSinceMs = millis();
            if (millis() - stableSinceMs >= TURN_STABLE_MS) {
                stopMotors();
                committedHeadingDeg = targetHeadingDeg;
                if (serviceLidars) {
                    settle(60);
                    return true;
                }
                return settleImuOnly(250);
            }
        } else {
            stableSinceMs = 0;
        }

        if (serviceLidars) serviceLidar();
        delay(LOOP_DELAY_MS);
    }

    stopMotors();
    return false;
}

bool parseTenths(
    uint16_t& index,
    uint16_t maximumMagnitude,
    bool allowNegative,
    int16_t& value) {
    bool negative = false;
    char current = routeAt(index);
    if (current == '-' || current == '+') {
        negative = current == '-';
        if (negative && !allowNegative) return false;
        current = routeAt(++index);
    }
    if (current < '0' || current > '9') return false;

    const uint16_t maximumWhole = maximumMagnitude / 10U;
    uint16_t whole = 0;
    do {
        const uint8_t digit = static_cast<uint8_t>(current - '0');
        if (whole > static_cast<uint16_t>((maximumWhole - digit) / 10U)) {
            return false;
        }
        whole = static_cast<uint16_t>(whole * 10U + digit);
        current = routeAt(++index);
    } while (current >= '0' && current <= '9');

    uint8_t fraction = 0;
    if (current == '.') {
        current = routeAt(++index);
        if (current < '0' || current > '9') return false;
        fraction = static_cast<uint8_t>(current - '0');
        current = routeAt(++index);
        if (current >= '0' && current <= '9') return false;
    }

    const uint16_t magnitude = static_cast<uint16_t>(whole * 10U + fraction);
    if (magnitude > maximumMagnitude) return false;
    value = negative ? -static_cast<int16_t>(magnitude)
                     : static_cast<int16_t>(magnitude);
    return true;
}

bool parseTask42Move(uint16_t& index, Task42Move& move) {
    if (routeAt(index) != '(') return false;
    ++index;
    if (!parseTenths(
            index, TASK42_MAX_TURN_TENTHS, true, move.turnTenths)
        || routeAt(index) != ',') {
        return false;
    }
    ++index;

    int16_t distanceTenths = 0;
    if (!parseTenths(
            index,
            TASK42_MAX_DISTANCE_TENTHS,
            false,
            distanceTenths)
        || routeAt(index) != ')') {
        return false;
    }
    ++index;
    move.distanceTenths = static_cast<uint16_t>(distanceTenths);
    return true;
}

bool validateTask42Block(uint16_t& index) {
    if (routeAt(index) != '[') return false;
    ++index;
    uint8_t moveCount = 0;
    bool hasTravel = false;

    while (true) {
        if (moveCount >= TASK42_MAX_MOVES) return false;
        Task42Move move;
        if (!parseTask42Move(index, move)) return false;
        ++moveCount;
        if (move.distanceTenths > 0) hasTravel = true;

        const char delimiter = routeAt(index);
        if (delimiter == ']') {
            ++index;
            return hasTravel;
        }
        if (delimiter != ',') return false;
        ++index;
        // Accept the computer-vision exporter's trailing comma before ']'.
        if (routeAt(index) == ']') {
            ++index;
            return hasTravel;
        }
    }
}

bool routeIsValid() {
    uint16_t index = 0;
    uint8_t task42Blocks = 0;
    bool hasCommands = false;
    char previousToken = '\0';  // 'n' normal, 'b' block, ',' separator.
    bool commaFollowsBlock = false;

    while (true) {
        const char command = tolower(routeAt(index));
        if (command == '\0') {
            return hasCommands
                && (previousToken != ',' || commaFollowsBlock);
        }
        if (command == ',') {
            if (previousToken == ',') return false;
            const char next = routeAt(index + 1U);
            const bool beforeBlock = next == '['
                && (previousToken == '\0' || previousToken == 'n');
            const bool afterBlock = previousToken == 'b'
                && (next == '\0'
                    || tolower(next) == 'f'
                    || tolower(next) == 'l'
                    || tolower(next) == 'r');
            if (!beforeBlock && !afterBlock) return false;
            commaFollowsBlock = afterBlock;
            previousToken = ',';
            ++index;
            continue;
        }
        if (command == 'f' || command == 'l' || command == 'r') {
            if (previousToken == ',' && !commaFollowsBlock) return false;
            hasCommands = true;
            previousToken = 'n';
            ++index;
            continue;
        }
        if (command == '[') {
            if (previousToken == ',' && commaFollowsBlock) return false;
            if (++task42Blocks > 1 || !validateTask42Block(index)) return false;
            hasCommands = true;
            previousToken = 'b';
            continue;
        }
        return false;
    }
}

bool recalibrateForTask42() {
    stopMotors();
    if (!settleImuOnly(TASK42_ENTRY_SETTLE_MS)) return false;
    for (uint8_t attempt = 0;
         attempt < TASK42_RECALIBRATION_ATTEMPTS;
         ++attempt) {
        if (calibrateYawRateBias(1000) && imu.update()) {
            // Task 4.2 headings are relative to the stationary entry pose.
            committedHeadingDeg = imu.getYaw();
            return true;
        }
        if (attempt + 1U < TASK42_RECALIBRATION_ATTEMPTS
            && !settleImuOnly(TASK42_RECALIBRATION_SETTLE_MS)) {
            return false;
        }
    }
    return false;
}

bool executeTask42Block(uint16_t& index) {
    if (routeAt(index) != '[' || !recalibrateForTask42()) return false;
    ++index;

    while (true) {
        Task42Move move;
        if (!parseTask42Move(index, move)) return false;

        // Camera coordinates are clockwise-positive and tuple turns are
        // relative to the heading left by the preceding tuple.
        const float clockwiseAngleDeg =
            static_cast<float>(move.turnTenths) * 0.1f;
        if (move.turnTenths != 0
            && !turnTo(
                committedHeadingDeg - clockwiseAngleDeg,
                false)) {
            return false;
        }
        if (move.distanceTenths > 0
            && !moveDeadReckoned(
                static_cast<float>(move.distanceTenths) * 0.1f)) {
            return false;
        }

        const char delimiter = routeAt(index);
        if (delimiter == ']') {
            ++index;
            return true;
        }
        if (delimiter != ',') return false;
        ++index;
        if (routeAt(index) == ']') {
            ++index;
            return true;
        }
    }
}

bool runRoute() {
    if (!routeIsValid()) return false;
    uint16_t index = 0;
    while (routeAt(index) != '\0') {
        const char command = tolower(routeAt(index));
        if (command == 'f') {
            uint8_t cells = 0;
            do {
                ++cells;
                ++index;
            } while (routeAt(index) != '\0'
                     && tolower(routeAt(index)) == 'f');
            if (!moveForward(static_cast<float>(cells) * CELL_DISTANCE_MM)) {
                return false;
            }
            settle(100);
        } else if (command == 'l') {
            ++index;
            if (!turnTo(committedHeadingDeg + 90.0f, true)) return false;
        } else if (command == 'r') {
            ++index;
            if (!turnTo(committedHeadingDeg - 90.0f, true)) return false;
        } else if (command == ',') {
            ++index;
        } else if (command == '[') {
            if (!executeTask42Block(index)) return false;
        } else {
            return false;
        }
    }
    return true;
}

void setup() {
    Serial.begin(9600);
    ready = initialiseHardware();
    if (!ready) {
        stopMotors();
        Serial.println(F("Hardware initialisation failed"));
    }
}

void loop() {
    if (!ready || routeRun) return;
    routeRun = true;
    const bool success = runRoute();
    stopMotors();
    Serial.println(success ? F("Route complete") : F("Route failed"));
}
