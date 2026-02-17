#!/usr/bin/env bash
set -euo pipefail

offline_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd "$offline_root/.." && pwd)"
online_root="${ME557_ONLINE_WS:-$repo_root/online}"

output="arduino/me557_pen_arduino_ws/ace_trajectory_data.h"
writer_log="/tmp/write_ace_offline.log"
moveit_log="/tmp/moveit_demo.log"

wait_sec=7
timeout_sec=120
use_rviz="false"

relative_to_current="false"
relative_scale="1.0"
semantic_yaw_deg="90.0"

workspace_enforce_x_bounds="false"
workspace_x_min="-0.1778"
workspace_x_max="0.1778"
workspace_z_min="0.0"
workspace_z_max="10.0"

scene_setup_enabled="true"
scene_board_enabled="true"
scene_board_id="ace_board"
scene_board_center_x="0.415"
scene_board_center_y="0.0"
scene_board_center_z="0.2286"
scene_board_size_x="0.01"
scene_board_size_y="0.45"
scene_board_size_z="0.40"
scene_table_enabled="false"
scene_table_id="ace_table"
scene_table_center_x="0.0"
scene_table_center_y="-0.34"
scene_table_center_z="0.10"
scene_table_size_x="0.60"
scene_table_size_y="0.60"
scene_table_size_z="0.05"

ros_domain_id="${ROS_DOMAIN_ID:-57}"
ros_localhost_only="${ROS_LOCALHOST_ONLY:-1}"

launch_pid=""

count_points() {
  local file="$1"
  awk '/kAceMoveitTrajectory\[] = \{/ {inarr=1; next} /\};/ && inarr {inarr=0; exit} inarr && /\{/ {c++} END {print c+0}' "$file"
}

usage() {
  cat <<'EOF'
Usage: offline/tools/export_offline_ace.sh [options]

Generates Arduino offline trajectory header using write_ace plan-only capture.

Options:
  --output PATH                     Output header path
                                    (default: arduino/me557_pen_arduino_ws/ace_trajectory_data.h)
  --writer-log PATH                 write_ace log file (default: /tmp/write_ace_offline.log)
  --moveit-log PATH                 MoveIt launch log file (default: /tmp/moveit_demo.log)
  --wait-sec N                      Seconds to wait after launch (default: 7)
  --timeout-sec N                   Timeout for write_ace run (default: 120)
  --use-rviz true|false             Launch RViz (default: false)

  --relative-to-current true|false  relative_to_current_pose (default: false)
  --relative-scale FLOAT            relative_scale (default: 1.0)
  --semantic-yaw-deg FLOAT          semantic_yaw_deg (default: 90.0)

  --workspace-enforce-x-bounds true|false  Clamp tip X to bounds (default: false)
  --workspace-x-min FLOAT           workspace_x_min (default: -0.1778)
  --workspace-x-max FLOAT           workspace_x_max (default: 0.1778)
  --workspace-z-min FLOAT           workspace_z_min (default: 0.0)
  --workspace-z-max FLOAT           workspace_z_max (default: 10.0)

  --scene-setup-enabled true|false  Apply PlanningScene collision objects (default: true)
  --scene-board-enabled true|false  Enable board collision object (default: true)
  --scene-board-id NAME             Board object id (default: ace_board)
  --scene-board-center-x FLOAT      Board center x in scene frame (default: 0.415)
  --scene-board-center-y FLOAT      Board center y in scene frame (default: 0.0)
  --scene-board-center-z FLOAT      Board center z in scene frame (default: 0.2286)
  --scene-board-size-x FLOAT        Board size x (default: 0.01)
  --scene-board-size-y FLOAT        Board size y (default: 0.45)
  --scene-board-size-z FLOAT        Board size z (default: 0.40)
  --scene-table-enabled true|false  Enable table collision object (default: false)
  --scene-table-id NAME             Table object id (default: ace_table)
  --scene-table-center-x FLOAT      Table center x in scene frame (default: 0.0)
  --scene-table-center-y FLOAT      Table center y in scene frame (default: -0.34)
  --scene-table-center-z FLOAT      Table center z in scene frame (default: 0.10)
  --scene-table-size-x FLOAT        Table size x (default: 0.60)
  --scene-table-size-y FLOAT        Table size y (default: 0.60)
  --scene-table-size-z FLOAT        Table size z (default: 0.05)

  --ros-domain-id N                 ROS_DOMAIN_ID (default: env or 57)
  --ros-localhost-only 0|1          ROS_LOCALHOST_ONLY (default: env or 1)
  --online-root PATH                Online ROS workspace root
                                    (default: ../online from this script)
  -h, --help                        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output) output="$2"; shift 2 ;;
    --writer-log) writer_log="$2"; shift 2 ;;
    --moveit-log) moveit_log="$2"; shift 2 ;;
    --wait-sec) wait_sec="$2"; shift 2 ;;
    --timeout-sec) timeout_sec="$2"; shift 2 ;;
    --use-rviz) use_rviz="$2"; shift 2 ;;
    --relative-to-current) relative_to_current="$2"; shift 2 ;;
    --relative-scale) relative_scale="$2"; shift 2 ;;
    --semantic-yaw-deg) semantic_yaw_deg="$2"; shift 2 ;;
    --workspace-enforce-x-bounds) workspace_enforce_x_bounds="$2"; shift 2 ;;
    --workspace-x-min) workspace_x_min="$2"; shift 2 ;;
    --workspace-x-max) workspace_x_max="$2"; shift 2 ;;
    --workspace-z-min) workspace_z_min="$2"; shift 2 ;;
    --workspace-z-max) workspace_z_max="$2"; shift 2 ;;
    --scene-setup-enabled) scene_setup_enabled="$2"; shift 2 ;;
    --scene-board-enabled) scene_board_enabled="$2"; shift 2 ;;
    --scene-board-id) scene_board_id="$2"; shift 2 ;;
    --scene-board-center-x) scene_board_center_x="$2"; shift 2 ;;
    --scene-board-center-y) scene_board_center_y="$2"; shift 2 ;;
    --scene-board-center-z) scene_board_center_z="$2"; shift 2 ;;
    --scene-board-size-x) scene_board_size_x="$2"; shift 2 ;;
    --scene-board-size-y) scene_board_size_y="$2"; shift 2 ;;
    --scene-board-size-z) scene_board_size_z="$2"; shift 2 ;;
    --scene-table-enabled) scene_table_enabled="$2"; shift 2 ;;
    --scene-table-id) scene_table_id="$2"; shift 2 ;;
    --scene-table-center-x) scene_table_center_x="$2"; shift 2 ;;
    --scene-table-center-y) scene_table_center_y="$2"; shift 2 ;;
    --scene-table-center-z) scene_table_center_z="$2"; shift 2 ;;
    --scene-table-size-x) scene_table_size_x="$2"; shift 2 ;;
    --scene-table-size-y) scene_table_size_y="$2"; shift 2 ;;
    --scene-table-size-z) scene_table_size_z="$2"; shift 2 ;;
    --ros-domain-id) ros_domain_id="$2"; shift 2 ;;
    --ros-localhost-only) ros_localhost_only="$2"; shift 2 ;;
    --online-root) online_root="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" >/dev/null 2>&1; then
    kill "$launch_pid" >/dev/null 2>&1 || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$online_root" != /* ]]; then
  online_root="$repo_root/$online_root"
fi

cd "$offline_root"
mkdir -p /tmp/ros_home /tmp/ros_logs
export ROS_HOME=/tmp/ros_home
export ROS_LOG_DIR=/tmp/ros_logs

set +u
source /opt/ros/humble/setup.bash
if [[ ! -f "$online_root/install/setup.bash" ]]; then
  echo "Missing $online_root/install/setup.bash" >&2
  echo "Build online workspace first: cd $online_root && colcon build" >&2
  exit 1
fi
source "$online_root/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$ros_domain_id"
export ROS_LOCALHOST_ONLY="$ros_localhost_only"

echo "Launching MoveIt (use_rviz=$use_rviz)..."
ros2 launch me557_pen_moveit_config demo.launch.py "use_rviz:=$use_rviz" >"$moveit_log" 2>&1 &
launch_pid=$!
sleep "$wait_sec"

echo "Running write_ace offline export..."
set +e
timeout "${timeout_sec}s" ros2 run me557_pen_description write_ace --ros-args \
  -p plan_only_capture:=true \
  -p relative_to_current_pose:="$relative_to_current" \
  -p relative_scale:="$relative_scale" \
  -p semantic_yaw_deg:="$semantic_yaw_deg" \
  -p workspace_enforce_x_bounds:="$workspace_enforce_x_bounds" \
  -p workspace_x_min:="$workspace_x_min" \
  -p workspace_x_max:="$workspace_x_max" \
  -p workspace_z_min:="$workspace_z_min" \
  -p workspace_z_max:="$workspace_z_max" \
  -p scene_setup_enabled:="$scene_setup_enabled" \
  -p scene_board_enabled:="$scene_board_enabled" \
  -p scene_board_id:="$scene_board_id" \
  -p scene_board_center_x:="$scene_board_center_x" \
  -p scene_board_center_y:="$scene_board_center_y" \
  -p scene_board_center_z:="$scene_board_center_z" \
  -p scene_board_size_x:="$scene_board_size_x" \
  -p scene_board_size_y:="$scene_board_size_y" \
  -p scene_board_size_z:="$scene_board_size_z" \
  -p scene_table_enabled:="$scene_table_enabled" \
  -p scene_table_id:="$scene_table_id" \
  -p scene_table_center_x:="$scene_table_center_x" \
  -p scene_table_center_y:="$scene_table_center_y" \
  -p scene_table_center_z:="$scene_table_center_z" \
  -p scene_table_size_x:="$scene_table_size_x" \
  -p scene_table_size_y:="$scene_table_size_y" \
  -p scene_table_size_z:="$scene_table_size_z" \
  -p trajectory_header_output:="$output" \
  >"$writer_log" 2>&1
rc=$?
set -e

summary_line="$(rg -n "ACE_SUMMARY attempted=" "$writer_log" -S | tail -n 1 || true)"
if [[ -z "$summary_line" ]]; then
  echo "No ACE_SUMMARY found in $writer_log" >&2
  tail -n 80 "$writer_log" >&2 || true
  exit 1
fi

if [[ "$summary_line" =~ attempted=([0-9]+)[[:space:]]+executed=([0-9]+)[[:space:]]+failed=([0-9]+)[[:space:]]+skipped=([0-9]+) ]]; then
  attempted="${BASH_REMATCH[1]}"
  executed="${BASH_REMATCH[2]}"
  failed="${BASH_REMATCH[3]}"
  skipped="${BASH_REMATCH[4]}"
else
  echo "Failed to parse ACE_SUMMARY line: $summary_line" >&2
  exit 1
fi

if [[ "$summary_line" =~ bounded=([0-9]+) ]]; then
  bounded="${BASH_REMATCH[1]}"
else
  bounded="n/a"
fi

if [[ "$output" = /* ]]; then
  output_file="$output"
else
  output_file="$offline_root/$output"
fi

points=0
if [[ -f "$output_file" ]]; then
  points="$(count_points "$output_file")"
fi

echo "ACE_SUMMARY attempted=$attempted executed=$executed failed=$failed skipped=$skipped bounded=$bounded"
echo "Output: $output_file (points=$points)"
echo "Logs: writer=$writer_log moveit=$moveit_log"

if [[ "$rc" -eq 124 ]]; then
  echo "write_ace timed out after ${timeout_sec}s" >&2
  exit 1
fi

if [[ "$rc" -ne 0 ]]; then
  echo "write_ace exited with code $rc" >&2
  exit 1
fi

if [[ "$failed" -ne 0 || "$executed" -lt "$attempted" || "$points" -lt 20 ]]; then
  echo "Offline trajectory export incomplete." >&2
  exit 1
fi

echo "Offline trajectory export successful."
