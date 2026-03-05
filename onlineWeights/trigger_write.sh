#!/bin/bash
# Terminal 2: Trigger board_writer repeatedly.
# Writes:
#   onlineWeights/robotMovements/trajectory_data.h
# (fixed-speed 555 trajectory, including pause flags)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Loop to allow repeated calls
while true; do
  echo "Press ENTER to run the movement (Ctrl+C to quit)..."
  read

  echo "[trigger_write] Triggering movement plan..."
  ros2 service call /board_writer/run_plan std_srvs/srv/Trigger "{}"
  echo "[trigger_write] trajectory_data.h updated."
done
