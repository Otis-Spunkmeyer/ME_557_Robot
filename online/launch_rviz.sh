#!/bin/bash
# Terminal 1: Build and launch RViz
# Only shows board_writer messages + ERRORs from other nodes

source /opt/ros/humble/setup.bash
colcon build --packages-select me557_pen_description me557_pen_moveit_config --event-handlers console_direct- 2>&1 | grep -E "(error|warning|finished|failed|Summary)" --color=never
echo ""
echo "Build done. Launching..."
echo ""
source install/setup.bash
ros2 launch me557_pen_moveit_config write_on_board.launch.py use_rviz:=true 2>&1 \
  | grep --line-buffered -E "(board_writer|ERROR|error: |FATAL)" --color=never
