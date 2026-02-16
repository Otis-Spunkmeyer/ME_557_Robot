# ME557 Arduino Offline Playback

This folder is the offline ME557 firmware and trajectory data.

Repository layout:

- `online/`: ROS2 + MoveIt workspace.
- `offline/`: Arduino firmware plus offline export/calibration tools.

## Files

- `me557_pen_arduino_ws/me557_pen_arduino_ws.ino`: main firmware.
- `me557_pen_arduino_ws/ace_trajectory_data.h`: exported MoveIt trajectory points.
- `../tools/export_moveit_joint_trajectory.py`: captures trajectory from MoveIt and writes header.
- `../tools/compute_moveit_calibration.py`: computes `MOVEIT_HOME_RAD` and `MOVEIT_DIR_SIGN`.
- `../tools/generate_offline_ace_ik_trajectory.py`: legacy IK-only debug exporter (no collision-aware planning).

## Export ACE trajectory

From repo root (`ME557_pen_arduino_ws`) with online workspace already built:

```bash
source /opt/ros/humble/setup.bash
source online/install/setup.bash
ROS_DOMAIN_ID=57 ROS_LOCALHOST_ONLY=1 \
python3 offline/tools/export_moveit_joint_trajectory.py \
  --run-write-ace \
  --output offline/arduino/me557_pen_arduino_ws/ace_trajectory_data.h
```

One-command offline export (launches MoveIt, exports, validates summary, then stops):

```bash
offline/tools/export_offline_ace.sh
```

`export_offline_ace.sh` runs `write_ace` in plan-only mode, captures planned
time-parameterized joint motion, and applies configured PlanningScene collision
objects (board/table) before planning.

Useful options:

```bash
# Strict 14" x 14" tip bounds on XZ
offline/tools/export_offline_ace.sh \
  --workspace-enforce-x-bounds true \
  --workspace-x-min -0.1778 \
  --workspace-x-max 0.1778 \
  --workspace-z-min 0.0 \
  --workspace-z-max 0.3556

# Floor-only Z constraint, unbounded X for tip requests
offline/tools/export_offline_ace.sh \
  --workspace-enforce-x-bounds false \
  --workspace-z-min 0.0 \
  --workspace-z-max 10.0

# Override board collision object used by planning scene setup
offline/tools/export_offline_ace.sh \
  --scene-board-center-y -0.420 \
  --scene-board-center-z 0.230 \
  --scene-board-size-x 0.50 \
  --scene-board-size-z 0.42
```

## Arduino commands

- `555`: play exported trajectory.
- `777`: per-joint ROM sweep.
- `888`: full ROM showcase.
- `901`: print current logical/physical angles and estimated MoveIt radians.
- `902`: per-joint sign/home calibration helper.

## Calibration example

1. Send `901` and record logical angles for IDs `[1,2,4,5,6]`.
2. Send `902` for each MoveIt joint index `1..5`, choose sign direction.
3. Collect synchronized values from one pose:

- MoveIt rad (5 joints), example:
```text
0.00698 5.55596 -0.66950 0.18387 0.00524
```
- Logical deg from `901`, example:
```text
180.4 179.2 181.1 179.7 180.3
```
- Signs from `902`, example:
```text
1 1 -1 1 1
```

4. Compute constants:

```bash
python3 offline/tools/compute_moveit_calibration.py \
  --moveit-rad 0.00698 5.55596 -0.66950 0.18387 0.00524 \
  --logical-deg 180.4 179.2 181.1 179.7 180.3 \
  --sign 1 1 -1 1 1
```

5. Paste output into `me557_pen_arduino_ws/me557_pen_arduino_ws.ino` and reflash.

## Notes

- `901` and `902` print values only; they do not save to EEPROM or source.
- Keep MoveIt and hardware values from the same pose.
- If one joint is inverted, flip only that joint sign and recompute home.
