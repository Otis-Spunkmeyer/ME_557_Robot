#!/bin/bash
# Terminal 2: Replay the A letter (run this repeatedly)
# Also exports both trajectory headers after each run:
#   online/robotMovements/trajectory_data.h  (original fixed-speed, code 555)
#   online/robotMovements/moveItData.h       (velocity-aware dynamic, code 777)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Loop to allow repeated calls
while true; do
  echo "Press ENTER to run the movement (Ctrl+C to quit)..."
  read

  echo "[trigger_write] Starting dynamic trajectory capture (code 777)..."
  python3 "$SCRIPT_DIR/tools/export_moveit_dynamic.py" \
    --output "$SCRIPT_DIR/robotMovements/moveItData.h" \
    --post-capture-sec 5 \
    --timeout-sec 120 &
  CAPTURE_PID=$!

  echo "[trigger_write] Triggering movement plan..."
  ros2 service call /board_writer/run_plan std_srvs/srv/Trigger "{}"

  echo "[trigger_write] Waiting for dynamic capture to finish..."
  wait $CAPTURE_PID
  CAPTURE_RC=$?

  if [ $CAPTURE_RC -eq 0 ]; then
    echo "[trigger_write] moveItData.h written successfully."
  else
    echo "[trigger_write] WARNING: dynamic capture failed (exit $CAPTURE_RC). moveItData.h may be stale."
  fi
done
