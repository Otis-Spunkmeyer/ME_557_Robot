#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

output="arduino/me557_pen_arduino_ws/ace_trajectory_data.h"
writer_log="/tmp/write_ace_offline.log"
moveit_log="/tmp/moveit_demo.log"

wait_sec=7
timeout_sec=120
use_rviz="false"

relative_to_current="true"
relative_scale="0.02"

workspace_enforce_x_bounds="false"
workspace_x_min="-0.1778"
workspace_x_max="0.1778"
workspace_z_min="0.0"
workspace_z_max="10.0"

ros_domain_id="${ROS_DOMAIN_ID:-57}"
ros_localhost_only="${ROS_LOCALHOST_ONLY:-1}"

launch_pid=""

count_points() {
  local file="$1"
  awk '/kAceMoveitTrajectory\[] = \{/ {inarr=1; next} /\};/ && inarr {inarr=0; exit} inarr && /\{/ {c++} END {print c+0}' "$file"
}

usage() {
  cat <<'EOF'
Usage: tools/export_offline_ace.sh [options]

Generates Arduino offline trajectory header using write_ace plan-only capture.

Options:
  --output PATH                     Output header path
                                    (default: arduino/me557_pen_arduino_ws/ace_trajectory_data.h)
  --writer-log PATH                 write_ace log file (default: /tmp/write_ace_offline.log)
  --moveit-log PATH                 MoveIt launch log file (default: /tmp/moveit_demo.log)
  --wait-sec N                      Seconds to wait after launch (default: 7)
  --timeout-sec N                   Timeout for write_ace run (default: 120)
  --use-rviz true|false             Launch RViz (default: false)

  --relative-to-current true|false  relative_to_current_pose (default: true)
  --relative-scale FLOAT            relative_scale (default: 0.02)

  --workspace-enforce-x-bounds true|false  Clamp tip X to bounds (default: false)
  --workspace-x-min FLOAT           workspace_x_min (default: -0.1778)
  --workspace-x-max FLOAT           workspace_x_max (default: 0.1778)
  --workspace-z-min FLOAT           workspace_z_min (default: 0.0)
  --workspace-z-max FLOAT           workspace_z_max (default: 10.0)

  --ros-domain-id N                 ROS_DOMAIN_ID (default: env or 57)
  --ros-localhost-only 0|1          ROS_LOCALHOST_ONLY (default: env or 1)
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
    --workspace-enforce-x-bounds) workspace_enforce_x_bounds="$2"; shift 2 ;;
    --workspace-x-min) workspace_x_min="$2"; shift 2 ;;
    --workspace-x-max) workspace_x_max="$2"; shift 2 ;;
    --workspace-z-min) workspace_z_min="$2"; shift 2 ;;
    --workspace-z-max) workspace_z_max="$2"; shift 2 ;;
    --ros-domain-id) ros_domain_id="$2"; shift 2 ;;
    --ros-localhost-only) ros_localhost_only="$2"; shift 2 ;;
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

cd "$workspace_root"
mkdir -p /tmp/ros_home /tmp/ros_logs
export ROS_HOME=/tmp/ros_home
export ROS_LOG_DIR=/tmp/ros_logs

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
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
  -p workspace_enforce_x_bounds:="$workspace_enforce_x_bounds" \
  -p workspace_x_min:="$workspace_x_min" \
  -p workspace_x_max:="$workspace_x_max" \
  -p workspace_z_min:="$workspace_z_min" \
  -p workspace_z_max:="$workspace_z_max" \
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
  output_file="$workspace_root/$output"
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
