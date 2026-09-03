# DemoBot Discrete Move Bluetooth

This Arduino sketch is the discrete DemoBot controller with one-way HC-06 telemetry for monitoring a demonstration. It retains the normal `f`/`l`/`r` route parser, optional Task 4.2 tuple block, encoder and IMU motion control, and LiDAR wall handling from `DemoBotDiscreteMove`.

The Bluetooth connection does not remotely control the robot. After successful hardware initialization, the checked-in compile-time route starts automatically, runs once, and reports progress over the link.

## HC-06 wiring

| HC-06 connection | Arduino Uno connection |
|---|---|
| VCC | Supply appropriate for the module breakout |
| GND | GND |
| TXD | D4 (`BLUETOOTH_RX`) |
| RXD | D5 (`BLUETOOTH_TX`) through a suitable voltage divider |

The sketch uses `SoftwareSerial` at 9600 baud. Confirm the supply and logic-level requirements of the specific HC-06 breakout before powering it.

## Telemetry format

Most messages are one comma-separated line:

```text
event=<name>,target=<value>,heading=<degrees>,encL=<rotations>,encR=<rotations>,lidarL=<mm>,lidarF=<mm>,lidarR=<mm>
```

Reported events include hardware readiness, route start/completion/failure, and the start and completion of forward legs, left/right turns, and Task 4.2 movements. A hardware initialization failure is sent as `event=hardware-failed`.

The `target` field is context-dependent: it is a distance in millimetres for forward events and a heading in degrees for turn events. LiDAR fields contain the most recently stored readings.

## Route configuration

Edit the active `ROUTE` declaration in `DemoBotDiscreteMoveBluetooth.ino`. Normal commands use one 180 mm cell per `f` and stationary 90-degree turns for `l` and `r`. Adjacent forward commands are combined.

One camera-derived Task 4.2 tuple block can also be included:

```cpp
const char ROUTE[] PROGMEM =
    "fr,[(58.1,287.4),(-3.9,116.7),(-64.3,377.6),],fl";
```

Each tuple is `(clockwise turn degrees, forward distance millimetres)`; negative angles turn left. Tuple legs use encoder distance and IMU heading without LiDAR wall correction.

## Before running

1. Raise or restrain the robot and verify both motor and encoder polarities.
2. Confirm the MPU6050 and all three VL6180X sensors initialize correctly.
3. Verify the HC-06 receives readable 9600-baud messages before a floor run.
4. Check that the active route matches the intended course.
5. Provide a physical way to stop or disconnect motor power; Bluetooth is monitoring-only.

For the controller design and route parser details, see the [root project README](../README.md#implementation-1-discrete-movement).
