# guppy

ROS 2 package for the Guppy ROV onboard driver. This package bridges ROS topics
on the single-board computer to the MCU serial protocol.

## Role In The System

`guppy` owns SBC to MCU communication.

- Reads command topics from ROS 2.
- Converts commands into the Teensy MCU packet format.
- Reads MCU telemetry from serial.
- Publishes telemetry back into ROS 2.

The MCU firmware source is maintained separately:

<https://github.com/panitan103/TEENSY_40_YOUYEETOO.git>

Current firmware hardware notes:

- MCU target: Teensy 4.0 using PlatformIO / Arduino.
- Connected and streaming/control-capable hardware:
  - PCA9685 PWM driver.
  - BNO055 IMU.
  - MS5837 pressure/depth sensor.
- No servo is currently attached to the MCU.
- PCA9685 channels are currently used for ESC/thruster outputs.

## Nodes

### `guppy_driver_control`

Main serial driver node.

Launch:

```bash
ros2 launch guppy guppy_driver_control.launch.py
```

Useful launch parameters:

- `port`: serial device, default `/dev/ttyS0`.
- `baud_rate`: default `115200`.
- `rtscts`: hardware flow control, default `true`.
- `debug_serial`: packet logging, default `false`.
- `cmd_vel_max_output`: normalized `/guppy/cmd_vel` scale, default `200`.
- `cmd_timeout_sec`: command timeout before neutral command, default `0.5`.
- `enable_legacy_topics`: default `true`.
- `enable_cmd_vel`: default `true`.

### `guppy_transmitted_example`

Small example publisher for the legacy command topic.

```bash
ros2 run guppy guppy_transmitted_example
```

## Topics

Command inputs:

- `Guppy_Transmitted`: legacy `guppy_interface/msg/GuppyDriverTransmitted`.
- `/guppy/cmd_raw`: conventional raw `GuppyDriverTransmitted`.
- `/guppy/cmd_vel`: `geometry_msgs/msg/Twist`, normalized `-1.0..1.0`.

Telemetry outputs:

- `Guppy_Received`: legacy `guppy_interface/msg/GuppyDriverReceived`.
- `/guppy/telemetry`: conventional telemetry alias.

## `/guppy/cmd_vel` Mapping

The driver maps AUV-style velocity axes into the MCU command message:

- `linear.x` -> `surge`
- `linear.y` -> `sway`
- `linear.z` -> `heave`
- `angular.z` -> `yaw`

Inputs are clipped to `-1.0..1.0`, scaled by `cmd_vel_max_output`, then encoded
as signed 16-bit values in the MCU command packet.

## Serial Protocol

ROS/SBC to MCU command packet:

- Header: `AB CD`
- Fields: `surge`, `sway`, `yaw`, `heave` as signed 16-bit big-endian values.
- Aux fields: `s1`, `s2`, `o1`, `o2`, `o3`, `o4` as unsigned bytes.
- Checksum: low byte of the sum of all previous bytes.

MCU to ROS/SBC telemetry line:

- Header: `DC BA`
- ASCII hex bytes separated by spaces.
- Fields: `roll`, `pitch`, `yaw`, `depth`, `water`, `volt`, `amp`.
- Checksum: low byte of the sum of all previous packet bytes.

## Development

Build and test:

```bash
colcon build --packages-select guppy_interface guppy
colcon test --packages-select guppy
colcon test-result --verbose
```

This package has its own git repository. Commit every coherent change locally;
push is manual.
