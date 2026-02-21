# ME557 Online (ROS2 + MoveIt)

This folder is the online workspace.

Key paths:

- `src/me557_pen_description`
- `src/me557_pen_moveit_config`
- `cad/`

Build and run:

```bash
cd online
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch me557_pen_moveit_config demo.launch.py
```

Run `write_ace` online with the same PlanningScene collision setup used by offline export:

```bash
ros2 launch me557_pen_moveit_config write_ace_online.launch.py use_rviz:=false
```
