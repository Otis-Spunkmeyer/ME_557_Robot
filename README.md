# ME557 Robot

This repository is split into two separate top-level structures:

- Online version (ROS2 + MoveIt): `online/`
- Offline version (Arduino playback): `offline/`

## Online Version

The online stack uses ROS2/MoveIt packages for planning and control:

- `online/src/me557_pen_description`
- `online/src/me557_pen_moveit_config`
- `online/cad/`

## Offline Version

The offline stack runs trajectory playback on Arduino without ROS at runtime:

- `offline/arduino/me557_pen_arduino_ws/me557_pen_arduino_ws.ino`
- `offline/arduino/me557_pen_arduino_ws/ace_trajectory_data.h`
- `offline/arduino/README.md`
- `offline/tools/`

## Quick Start

Build online workspace:

```bash
cd online
source /opt/ros/humble/setup.bash
colcon build
```

Generate offline trajectory header:

```bash
offline/tools/export_offline_ace.sh
```

This export path uses MoveIt planning (`write_ace`) and applies configured
PlanningScene collision objects before planning.
