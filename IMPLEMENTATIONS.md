# Micromouse Implementations Guide

This repository contains discrete-movement and continuous-path controllers. Each Arduino sketch is a separate build; open the `.ino` file inside the implementation folder and upload only that sketch.

## At a glance

| Implementation | Purpose | Route input |
|---|---|---|
| `DemoBotPinDetails4.1` | Original compact DemoBot controller with PID turns, encoder distance control and LiDAR wall following | Compile-time `f`, `l`, `r` string |
| `DemoBotDiscreteMove` | Barebones DemoBot controller using the more robust discrete movement, custom IMU/LiDAR drivers and Task 4.2 support | Compile-time `f`, `l`, `r` string with one optional tuple block |
| `DemoBotDiscreteMoveBluetooth` | Discrete DemoBot controller with one-way HC-06 route and sensor telemetry | Compile-time `f`, `l`, `r` string with one optional tuple block |
| `DemoBotPPEKF` | Continuous pure-pursuit path following with encoder/IMU EKF estimation and optional map-based front-LiDAR correction | Compile-time `f`, `l`, `r` string converted to waypoints |
| `micromouse-BestDiscreteMove` | Full reference implementation for Task 4.1/4.2 command routes or Task 4.3 autonomous mapping | Command string or autonomous start/goal configuration |
| `F12A_T03-Micromouse-TaperedPPEKF` | Original continuous tapered pure-pursuit and EKF reference | Explicit waypoint array; no movement string |

## DemoBotPinDetails4.1

This is the original simple state-machine implementation. It combines consecutive forward commands, uses encoder distance and IMU heading feedback, follows side walls with the LiDAR sensors, and stops/settles when a front wall is detected.

Important files:

- `DemoBotPinDetails4.1/DemoBotPinDetails4.1.ino` — pins, geometry, LiDAR thresholds, movement distance, speed limits, PID gains and route.
- `DemoBotPinDetails4.1/PIDController.hpp` — PID calculation.
- `DemoBotPinDetails4.1/Motor.hpp` — motor direction and PWM behaviour.
- `DemoBotPinDetails4.1/DualEncoder.hpp` — encoder pins, counts and direction handling.

Edit `moveSetChar` in `DemoBotPinDetails4.1.ino`:

```cpp
char* moveSetChar = "ffrfl";
```

- `f` — forward by `distanceF` (currently 175 mm); adjacent `f` commands are combined.
- `l` — 90-degree stationary left turn.
- `r` — 90-degree stationary right turn.

Use only these command letters. The code accepts upper- or lowercase letters but does not validate other characters before starting.

## DemoBotDiscreteMove

This is the recommended barebones discrete controller for the DemoBot hardware. It retains the DemoBot motor, encoder and LiDAR pin arrangement while using safer motor reversal, non-blocking LiDAR reads, a lightweight gyro driver, controlled acceleration/deceleration, IMU turn settling and end-of-cell wall alignment.

Important files:

- `DemoBotDiscreteMove/DemoBotDiscreteMove.ino` — all pins, geometry, wall targets, movement/turn tuning, Task 4.2 settings and `ROUTE`.
- `DemoBotDiscreteMove/Imu.hpp` and `Imu.cpp` — MPU6050 sampling, validation and bias calibration.
- `DemoBotDiscreteMove/Lidar.hpp` and `Lidar.cpp` — VL6180X addressing and non-blocking ranging.
- `DemoBotDiscreteMove/Motor.hpp` — motor polarity and safe PWM switching.
- `DemoBotDiscreteMove/DualEncoder.hpp` — encoder resolution and count handling.

### Task 4.1 route format

Edit the active `ROUTE` declaration in `DemoBotDiscreteMove.ino`:

```cpp
const char ROUTE[] PROGMEM = "ffrfl";
```

- `f` — one 180 mm cell.
- `l` — 90-degree stationary left turn.
- `r` — 90-degree stationary right turn.

The route is lowercase, contains no spaces and runs once after startup.

### Bluetooth telemetry variant

`DemoBotDiscreteMoveBluetooth` uses the same route formats and movement logic, then adds an HC-06 software-serial link at 9600 baud. It transmits status, route events, targets, heading, encoder rotations, and left/front/right LiDAR readings. It does not accept Bluetooth control commands. See `DemoBotDiscreteMoveBluetooth/README.md` for wiring and the message format.

### Task 4.2 route format

A route may contain one Task 4.2 block, optionally surrounded by normal commands:

```cpp
const char ROUTE[] PROGMEM =
    "fr,[(58.1,287.4),(-3.9,116.7),(-64.3,377.6),],fl";
```

Each tuple is `(relative turn degrees, forward distance millimetres)`:

- positive angle — clockwise/right turn;
- negative angle — anticlockwise/left turn;
- distance — forward travel after that turn;
- integer or one decimal place only;
- angle range: -180.0 to 180.0 degrees;
- distance range: 0.0 to 1600.0 mm;
- maximum 64 tuples and one tuple block per route.

Tuples are comma-separated. A trailing comma before `]` is optional. When normal commands appear beside the block, retain the separating commas shown above. LiDAR readings are ignored during the Task 4.2 block, although the sensors remain powered; normal wall control resumes after `]`.

## DemoBotPPEKF

This implementation converts a cell-command route into metric waypoints, estimates continuous `(x, y, heading)` pose with an EKF, and uses tapered pure pursuit plus wheel-velocity control. Corners are followed as a continuous curve rather than separate stop-and-turn actions.

Important files:

- `DemoBotPPEKF/Config.hpp` — pins, motor/encoder signs, wheel geometry, speeds, controller gains, sensor timing and feature switches.
- `DemoBotPPEKF/Route.hpp` — movement string, initial heading, route capacity and LiDAR-correction segments.
- `DemoBotPPEKF/MazeMap.hpp` — known maze geometry used by LiDAR/EKF correction.
- `DemoBotPPEKF/PurePursuit.hpp` — lookahead and path-following behaviour.
- `DemoBotPPEKF/VelocityController.hpp` and `StateEstimator.hpp` — wheel control and EKF logic.
- `DemoBotPPEKF/README.md` — implementation-specific setup and safety checks.

Edit `ROUTE_COMMANDS` in `Route.hpp`:

```cpp
const char ROUTE_COMMANDS[] PROGMEM = "ffrfl";
```

`f`, `l` and `r` describe a 180 mm grid path. Here, turns change the direction of the next generated waypoint; they are not stationary turns. Uppercase commands and whitespace are accepted. The route must contain at least one `f` and no more than 47 forward moves. Task 4.2 tuples are not supported. Keep `MazeMap.hpp` and the front-wall correction segment list consistent with the route.

## Best reference implementations

### micromouse-BestDiscreteMove

This is the full-featured discrete reference. Command mode supports Task 4.1 cell moves and a Task 4.2 tuple block. Autonomous mode maps an unknown maze, plans paths and navigates between configured cells.

Important files:

- `Controller.cpp` — active mode, `COMMAND_STRING`, Task 4.2 parser, display switch and Task 4.3 start/goal.
- `Movement.cpp` — wiring, motor polarity, wheel calibration, speeds, wall control and turn tuning.
- `RobotGeometry.hpp` — cell and chassis/sensor geometry.
- `AutonomousMapping.cpp`, `Maze.cpp` and `MazePlanner.cpp` — Task 4.3 mapping and planning.
- `Imu.cpp`, `Lidar.cpp` and `Encoder.cpp` — sensor and odometry drivers.

In command-string mode, edit `COMMAND_STRING` in `Controller.cpp`. Its `f`/`l`/`r` and Task 4.2 tuple syntax matches `DemoBotDiscreteMove`. To use Task 4.3 instead, change `MICROMOUSE_ACTIVE_MODE` and update `TASK_43_CONFIG`; autonomous mode does not use a route string.

### F12A_T03-Micromouse-TaperedPPEKF

This is the original continuous-path reference used as the basis for `DemoBotPPEKF`. It fuses encoders and IMU data, optionally corrects pose from a known wall map, and follows a pre-generated waypoint path with tapered pure pursuit.

Important files:

- `F12A_T03-Micromouse.ino` — pins, geometry, speeds, gains, feature switches and sensor mounting offsets.
- `waypoints_planned.hpp` — initial heading, waypoint coordinates and course segment metadata.
- `walls_planned.hpp` — maze wall geometry used for pose correction.
- `PurePursuit.hpp`, `StateEstimator.hpp` and `VelocityPID.hpp` — navigation and control behaviour.

This implementation has no movement string. Replace the `WAYPOINTS` array in `waypoints_planned.hpp` with `{x, y}` coordinates in metres, update `WAYPOINT_LEN`, and keep the initial heading and wall data consistent. The `high_level` folder contains generated route/map outputs that can be copied into the active planned headers.

## Hardware warning

Do not assume the LiDAR XSHUT order is interchangeable:

- DemoBot implementations: A0 = left, A1 = front, A2 = right.
- Best reference implementations: A0 = left, A1 = right, A2 = front.

All implementations must still be calibrated on the physical robot. In particular, verify wheel radius, wheelbase where used, encoder polarity, motor polarity, wall targets, stopping distance and turn gains before a full-speed maze run.
