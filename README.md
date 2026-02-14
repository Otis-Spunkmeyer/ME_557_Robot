# ME557 Robot

This repository contains both ME557 versions:

- Online version (ROS2 + MoveIt): `src/`
- Offline version (Arduino playback): `arduino/`

## Online Version

The online stack uses ROS2/MoveIt packages for planning and control:

- `src/me557_pen_description`
- `src/me557_pen_moveit_config`

## Offline Version

The offline stack runs trajectory playback on Arduino without ROS at runtime:

- `arduino/me557_pen_arduino_ws/me557_pen_arduino_ws.ino`
- `arduino/me557_pen_arduino_ws/ace_trajectory_data.h`
- `arduino/README.md`

## Tools

Helper scripts for exporting and calibrating trajectories are in `tools/`.
