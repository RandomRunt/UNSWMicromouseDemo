# DemoBot PPEKF

This sketch adapts `F12A_T03-Micromouse-TaperedPPEKF` to the hardware and
compact coding conventions of `DemoBotPinDetails4.1`.

The navigation chain is:

1. signed encoder increments and tilt-compensated gyro yaw feed a 3-state
   `(x, y, heading)` EKF;
2. selected front-ToF observations are gated against the known maze map and
   correct EKF position;
3. tapered pure pursuit converts the estimated pose and waypoint route into
   left/right wheel velocity targets;
4. filtered wheel-velocity PID plus feed-forward converts those targets into
   B-compatible motor PWM commands.

## Hardware retained from B

- left motor PWM/direction: `11/12`
- right motor PWM/direction: `9/10`
- left encoder A/B: `2/7`
- right encoder A/B: `3/8`
- left/front/right LiDAR XSHUT: `A0/A1/A2`
- left/front/right LiDAR addresses: `0x54/0x56/0x58`
- wheel radius: `16 mm`
- wheelbase: `90 mm`
- encoder resolution: `700 counts/revolution`
- left motor and encoder polarities are reversed exactly as in B

## Before running the motors

The checked-in `Route.hpp` and `MazeMap.hpp` are A's example route and known
maze. Replace both when the physical course changes. Confirm the following on
blocks or with the motors disabled first:

- forward encoder signs produce increasing left and right distances;
- positive wheel commands drive both wheels forward;
- positive IMU heading corresponds to a counter-clockwise/left turn;
- `FRONT_LIDAR_MOUNT_M` in `Config.hpp` matches the actual sensor position;
- pure-pursuit and velocity-PID gains suit B's drivetrain.

Set `MOTORS_ENABLED` to `false` in `Config.hpp` for estimator-only bench tests.
Map-based ToF correction can independently be disabled with
`FRONT_LIDAR_EKF_ENABLED`.

## Route command string

Edit `ROUTE_COMMANDS` in `Route.hpp` using the same basic command style as
`DemoBotDiscreteMove`:

- `f` advances the planned position by one 180 mm maze cell and adds a waypoint.
- `l` rotates the planned direction 90 degrees left.
- `r` rotates the planned direction 90 degrees right.

At startup, `buildWaypointRoute()` converts the flash-resident string into the
metric waypoint array used by pure pursuit. Uppercase commands and whitespace
are accepted. An invalid command, a route with no `f`, or more than 47 forward
moves fails safely and leaves the motors stopped.

The checked-in string selects the right-side Robotics@UNSW showcase route; a
left-side route is retained as a commented alternative beside it. Unlike the
discrete mover, `l` and `r` do not execute stationary turns: they set the
direction of the next waypoint, and pure pursuit rounds the resulting corner.
Tuple commands such as `[(58.1,287.4), ...]` are not supported.

After changing the string, make sure `MazeMap.hpp` matches the physical course
and update `FRONT_WALL_CORRECTION_SEGMENTS`, or temporarily set
`FRONT_LIDAR_EKF_ENABLED` to `false`. A mismatched route/map could associate a
LiDAR reading with the wrong wall and pull the EKF pose in the wrong direction.
