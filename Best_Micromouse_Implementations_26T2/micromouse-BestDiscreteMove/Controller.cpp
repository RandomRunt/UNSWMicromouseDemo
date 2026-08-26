// AI-ASSISTED FILE: Task 4.2/4.3 mode integration generated with OpenAI Codex
// (2026-08-11 to 2026-08-12). AI-authored sections are identified by comments.
#include "Controller.hpp"

#define MICROMOUSE_MODE_COMMAND_STRING 1
#define MICROMOUSE_MODE_AUTONOMOUS_MAPPING 2

// Change only this line to switch competition programs. The inactive program
// is removed by the preprocessor, making the Task 4.3 build visibly free of a
// hard-coded maze route.
#ifndef MICROMOUSE_ACTIVE_MODE
#define MICROMOUSE_ACTIVE_MODE MICROMOUSE_MODE_COMMAND_STRING
#endif

#if MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_COMMAND_STRING
#include <avr/pgmspace.h>
#elif MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_AUTONOMOUS_MAPPING
#include "AutonomousMapping.hpp"
#else
#error "Select a supported MICROMOUSE_ACTIVE_MODE"
#endif

namespace {

#if MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_COMMAND_STRING
// Section 4.1.2/4.2 route. Keeping it in flash saves one byte of scarce SRAM
// per character. Inside a Task 4.2 block, every tuple is
// (signed clockwise turn in degrees, forward distance in millimetres).
// This syntax is copied directly from computer_vision/continous_planning.
const char COMMAND_STRING[] PROGMEM =
    "frflfffflflfrflf,[(58.1,287.4),(-3.9,116.7),(-64.3,377.6),(53.1,161.1),(73.7,378.7),(-26.7,0.0),],frflfrfrflff";

constexpr float CELL_MM = 180.0f;
constexpr uint16_t TASK42_MAX_TURN_TENTHS = 1800;
constexpr uint16_t TASK42_MAX_DISTANCE_TENTHS = 16000;
constexpr uint8_t TASK42_MAX_MOVES = 64;
// Normal f/l/r commands settle for only 100 ms. Give residual wheel coast time
// to finish before the entry bias calibration starts monitoring the encoders;
// otherwise a fraction of a millimetre of late travel rejects the whole block.
constexpr uint16_t TASK42_ENTRY_SETTLE_MS = 400;
constexpr uint16_t TASK42_RECALIBRATION_SETTLE_MS = 250;
constexpr uint8_t TASK42_RECALIBRATION_ATTEMPTS = 3;

struct Task42Move {
    int16_t turnTenths;
    uint16_t distanceTenths;
};
#endif

// Set to false to disable all OLED I2C traffic. Command mode shows F/L/R lidar
// diagnostics; Task 4.3 shows the map, completion percentage, and phase.
constexpr bool ENABLE_DISPLAY = true;

#if MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_AUTONOMOUS_MAPPING
// Task 4.3 permits only the supplied start pose and goal coordinates to be
// hard-coded. Cartesian (0,0) is south-west; x grows East and y grows North.
// The twelve L-shaped corner cells are non-playable. Replace these values
// with the marking-day coordinates; both start and goal must be mappable.
constexpr mtrn3100::Task43Config TASK_43_CONFIG = {
    {3, 7},
    mtrn3100::Direction::South,
    {4, 7},
};
#endif

#if MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_COMMAND_STRING
char commandAt(uint16_t index) {
    if (index >= sizeof(COMMAND_STRING)) return '\0';
    return static_cast<char>(pgm_read_byte(COMMAND_STRING + index));
}

// Parse an integer or one-decimal-place number without String, sscanf, strtod,
// heap allocation, or an SRAM copy of the flash-resident route.
bool parseTenths(
    uint16_t& index,
    uint16_t maximumMagnitude,
    bool allowNegative,
    int16_t& value) {
    bool negative = false;
    char current = commandAt(index);
    if (current == '-' || current == '+') {
        negative = current == '-';
        if (negative && !allowNegative) return false;
        current = commandAt(++index);
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
        current = commandAt(++index);
    } while (current >= '0' && current <= '9');

    uint8_t fraction = 0;
    if (current == '.') {
        current = commandAt(++index);
        if (current < '0' || current > '9') return false;
        fraction = static_cast<uint8_t>(current - '0');
        current = commandAt(++index);
        // The vision exporter rounds to one decimal place. Rejecting further
        // digits prevents an apparently valid value from being truncated.
        if (current >= '0' && current <= '9') return false;
    }

    const uint16_t magnitude = static_cast<uint16_t>(whole * 10U + fraction);
    if (magnitude > maximumMagnitude) return false;
    value = negative ? -static_cast<int16_t>(magnitude)
                     : static_cast<int16_t>(magnitude);
    return true;
}

bool parseTask42Move(uint16_t& index, Task42Move& move) {
    if (commandAt(index) != '(') return false;
    ++index;

    if (!parseTenths(
            index, TASK42_MAX_TURN_TENTHS, true, move.turnTenths)
        || commandAt(index) != ',') {
        return false;
    }
    ++index;

    int16_t distanceTenths = 0;
    if (!parseTenths(
            index,
            TASK42_MAX_DISTANCE_TENTHS,
            false,
            distanceTenths)
        || commandAt(index) != ')') {
        return false;
    }
    ++index;
    move.distanceTenths = static_cast<uint16_t>(distanceTenths);
    return true;
}

bool validateTask42Block(uint16_t& index) {
    if (commandAt(index) != '[') return false;
    ++index;

    uint8_t moveCount = 0;
    bool hasTravel = false;
    while (true) {
        if (moveCount >= TASK42_MAX_MOVES) return false;
        Task42Move move;
        if (!parseTask42Move(index, move)) return false;
        ++moveCount;
        if (move.distanceTenths > 0) hasTravel = true;

        const char delimiter = commandAt(index);
        if (delimiter == ']') {
            ++index;
            return hasTravel;
        }
        if (delimiter != ',') return false;
        ++index;
        // computer_vision emits a trailing comma before ']'. Also accept a
        // block without the trailing comma for convenient hand-written tests.
        if (commandAt(index) == ']') {
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
        const char command = commandAt(index);
        if (command == '\0') {
            return hasCommands
                && (previousToken != ',' || commaFollowsBlock);
        }
        if (command == ',') {
            if (previousToken == ',') return false;
            const char next = commandAt(index + 1U);
            const bool beforeBlock = next == '['
                && (previousToken == '\0' || previousToken == 'n');
            const bool afterBlock = previousToken == 'b'
                && (next == '\0' || next == 'f' || next == 'l' || next == 'r');
            if (!beforeBlock && !afterBlock) return false;
            commaFollowsBlock = afterBlock;
            previousToken = ',';
            ++index;
            continue;
        }
        if (command == 'f' || command == 'l' || command == 'r') {
            if (previousToken == ',' && !commaFollowsBlock) {
                return false;
            }
            hasCommands = true;
            previousToken = 'n';
            ++index;
            continue;
        }
        if (command == '[') {
            if (previousToken == ',' && commaFollowsBlock) {
                return false;
            }
            if (++task42Blocks > 1
                || !validateTask42Block(index)) {
                return false;
            }
            hasCommands = true;
            previousToken = 'b';
            continue;
        }
        return false;
    }
}

bool executeTask42Block(
    mtrn3100::Movement& movement,
    uint16_t& index) {
    if (commandAt(index) != '[') return false;

    // Keep this pause lidar-free: at the obstacle-course boundary only the
    // drivetrain must become still before gyro recalibration. A failed attempt
    // itself supplies another one-second stationary window, but retain a short
    // explicit pause before retrying so vibration can decay as well.
    movement.stop();
    delay(TASK42_ENTRY_SETTLE_MS);
    for (uint8_t attempt = 0;
         attempt < TASK42_RECALIBRATION_ATTEMPTS;
         ++attempt) {
        if (movement.recalibrateAtCurrentHeading()) {
            break;
        }
        if (attempt + 1U < TASK42_RECALIBRATION_ATTEMPTS) {
            movement.stop();
            delay(TASK42_RECALIBRATION_SETTLE_MS);
        }
    }
    // If all attempts fail, retain the last bias established during startup and
    // execute the generated route instead of abandoning it at the transition.
    ++index;

    while (true) {
        Task42Move move;
        if (!parseTask42Move(index, move)) return false;

        // The computer-vision frame is clockwise-positive: positive is a
        // right turn and negative is a left turn. Every tuple is relative to
        // the robot's heading after the preceding tuple.
        if (move.turnTenths > 0) {
            if (!movement.rotateRightDeadReckoned(
                    static_cast<float>(move.turnTenths) * 0.1f)) {
                movement.adoptCurrentHeading();
            }
        } else if (move.turnTenths < 0) {
            if (!movement.rotateLeftDeadReckoned(
                    static_cast<float>(-move.turnTenths) * 0.1f)) {
                movement.adoptCurrentHeading();
            }
        }

        // A zero-distance tuple is intentional: its turn aligns the robot for
        // the ordinary f/l/r commands following the bracketed course.
        if (move.distanceTenths > 0) {
            // A bounded runtime failure consumes this tuple. Continuing gives
            // the remaining generated route a valid attempt instead of ending
            // the whole run beside the obstacle course.
            (void)movement.moveDeadReckoned(
                static_cast<float>(move.distanceTenths) * 0.1f);
        }

        const char delimiter = commandAt(index);
        if (delimiter == ']') {
            ++index;
            return true;
        }
        if (delimiter != ',') return false;
        ++index;
        if (commandAt(index) == ']') {
            ++index;
            return true;
        }
    }
}
#endif

}  // namespace

void Controller::initialise() {
#if MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_AUTONOMOUS_MAPPING
    constexpr bool SHOW_LIDAR_DIAGNOSTICS = false;
    // A full initialization includes the gyro warm-up. Do not repeat it
    // forever when required hardware is unavailable.
    ready = move.initialise(
        ENABLE_DISPLAY,
        SHOW_LIDAR_DIAGNOSTICS);
#else
    constexpr bool SHOW_LIDAR_DIAGNOSTICS = ENABLE_DISPLAY;
    ready = move.initialise(
        ENABLE_DISPLAY,
        SHOW_LIDAR_DIAGNOSTICS);
#endif
}

void Controller::run() {
    if (!ready) {
        move.stop();
        ready = false;
        return;
    }

#if MICROMOUSE_ACTIVE_MODE == MICROMOUSE_MODE_AUTONOMOUS_MAPPING
    {
        // Static storage keeps the fixed mapping state off the Nano's small
        // call stack while the page renderer is active.
        static mtrn3100::AutonomousMapping mapping(move, TASK_43_CONFIG);
        mapping.run();
        move.stop();
        ready = false;
        return;
    }
#else
    if (!routeIsValid()) {
        move.stop();
        ready = false;
        return;
    }

    uint16_t index = 0;
    while (true) {
        const char command = commandAt(index);
        if (command == '\0') break;
        bool settleAfterCommand = false;

        if (command == ',') {
            ++index;
            continue;
        } else if (command == 'f') {
            uint8_t cellCount = 0;
            do {
                ++cellCount;
                ++index;
            } while (cellCount < UINT8_MAX && commandAt(index) == 'f');
            (void)move.moveForward(static_cast<float>(cellCount) * CELL_MM);
            settleAfterCommand = true;
        } else if (command == 'l') {
            ++index;
            if (!move.rotateLeftTask41(90.0f)) {
                move.adoptCurrentHeading();
            }
        } else if (command == 'r') {
            ++index;
            if (!move.rotateRightTask41(90.0f)) {
                move.adoptCurrentHeading();
            }
        } else if (command == '[') {
            if (!executeTask42Block(move, index)) {
                move.stop();
                ready = false;
                return;
            }
            // Task 4.2 turns and legs already perform stationary
            // settling. Do not immediately add a lidar/OLED service interval
            // while the robot is still beside the obstacle course.
            continue;
        } else {
            move.stop();
            ready = false;
            return;
        }

        // Forward coast still gets the proven 100 ms pause. Fast Task 4.1
        // turns already include their own 60 ms stopped lidar-service window,
        // so the old additional 100 ms here was entirely redundant.
        if (settleAfterCommand) move.settle(100);
    }

    move.stop();
    ready = false;
#endif
}
