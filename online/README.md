# ME557 Online — ROS 2 + MoveIt + Arduino

This folder contains everything needed to plan, export, and execute robot writing movements **live** on the hardware. The general flow is:

1. Launch the ROS 2 / MoveIt stack (`launch_rviz.sh`)
2. Trigger a planning + simulation run (`trigger_write.sh`) — this also auto-exports the Arduino trajectory headers
3. Flash the Arduino sketch and send a serial code to run the arm

---

## Folder Structure

```
online/
├── launch_rviz.sh                 # Terminal 1: build + launch RViz / MoveIt
├── trigger_write.sh               # Terminal 2: trigger a plan run + export headers
├── tools/
│   └── export_moveit_dynamic.py  # ROS 2 node: captures MoveIt trajectory → moveItData.h
├── robotMovements/
│   ├── robotMovements.ino        # Arduino sketch — flash this to the robot
│   ├── trajectory_data.h         # Auto-generated: fixed-speed trajectory (code 555)
│   └── moveItData.h              # Auto-generated: velocity-aware trajectory (code 777)
├── src/
│   ├── me557_pen_description/    # URDF, meshes, ROS 2 nodes (write_ace / board_writer)
│   └── me557_pen_moveit_config/  # MoveIt config, launch files
├── letterCoordinates.json        # Letter path coordinates used by board_writer
├── letterCoordinateMaker.py      # Tool to generate/edit letterCoordinates.json
└── cad/                          # SolidWorks assembly (reference only)
```

---

## Prerequisites

- Ubuntu 22.04 with ROS 2 Humble installed
- `source /opt/ros/humble/setup.bash` (add to `~/.bashrc` to avoid repeating)
- Arduino IDE or `arduino-cli` with the `Dynamixel2Arduino` library installed
- Arduino board connected via USB (typically `/dev/ttyUSB0` or `/dev/ttyACM0`)
- Dynamixel servos connected and powered

---

## Full Workflow

### Step 1 — Build and launch RViz / MoveIt

Open **Terminal 1** from the `online/` directory:

```bash
cd online
./launch_rviz.sh
```

This builds the two ROS 2 packages, launches MoveIt, and starts the `board_writer` node which plans and simulates the pen writing. RViz will open showing the robot and the planned path. Wait until you see `board_writer` ready messages before continuing.

> Only `board_writer` output and ERRORs are shown — other node noise is filtered out.

---

### Step 2 — Trigger a run and export trajectory headers

Open **Terminal 2** from the `online/` directory:

```bash
cd online
./trigger_write.sh
```

Press **ENTER** when prompted. This does three things in one step:

1. Starts `export_moveit_dynamic.py` in the background, subscribed to the MoveIt trajectory topic
2. Calls the `board_writer` ROS 2 service to trigger planning and simulation
3. Waits for all trajectory segments (5 s silence window), then writes `robotMovements/moveItData.h`

When it finishes you will see:
```
[trigger_write] moveItData.h written successfully.
```

> `trajectory_data.h` (used by code 555) is written by the `board_writer` node directly during the run. Both files are updated every time you press ENTER.

Press ENTER again in Terminal 2 to re-run and re-export as many times as needed.

---

### Step 3 — Flash the Arduino

Open `robotMovements/robotMovements.ino` in Arduino IDE and flash to the board. The sketch already `#include`s both `trajectory_data.h` and `moveItData.h` from the same folder — they are updated automatically by the workflow above, so just reflash after each trigger run.

**Serial port baud rate:** 1000000 (set this in Arduino IDE's serial monitor too).

---

### Step 4 — Run the arm

Open the Arduino Serial Monitor at **1000000 baud**. On startup the arm homes (all joints → 180°) and prints the available codes. Type a code and press Enter to send it:

| Code | Action |
|------|--------|
| `100` | Home all motors to 180° |
| `555` | Play `trajectory_data.h` — fixed speed, streaming keyframes |
| `777` | Play `moveItData.h` — **velocity-aware**, uses MoveIt's planned timing and per-joint velocities *(recommended)* |
| `1`–`6` | Manual control — lock a single servo, send target angles interactively (send `999` to exit) |

**Recommended: use `777`** for best smoothness. Use `555` as a fallback if `moveItData.h` is empty or stale.

When code `777` starts, a diagnostic summary prints before any movement:
```
Waypoints loaded : 247
Pause boundaries : 6
Planned duration : 18 s
First dt_ms      : 120
Last  dt_ms      : 85
--- STARTING ---
```
If `Waypoints loaded` is very small (< 50) or `Planned duration` is under 5 s, the export likely only caught the first segment — re-run `trigger_write.sh` and reflash.

---

## Tuning Parameters

All motion parameters live at the top of `robotMovements.ino` and are labelled clearly.

### Global speed / acceleration

| Constant | Default | Effect |
|---|---|---|
| `TRAJECTORY_SPEED` | 8 | Speed for code 555. Lower = smoother but slower |
| `ACCEL_SMOOTH` | 15 | Velocity ramp steepness for MX-64 servos. Lower = gentler start/stop |

### Servo 1 — base rotation (MX-64, extra bearing load)

| Constant | Default | Effect |
|---|---|---|
| `SERVO1_P_GAIN` | 400 | Position correction aggression. Lower = less overshoot |
| `SERVO1_D_GAIN` | 400 | Damping. Higher = kills oscillation faster |
| `SERVO1_I_GAIN` | 0 | Keep at 0 — integral wind-up worsens oscillation |

### Servo 4 — elbow (AX-12, worst moment arm / gravity load)

| Constant | Default | Effect |
|---|---|---|
| `SERVO4_AX12_SLOPE` | 64 | Compliance slope (2–128). Higher = softer spring, less snap-back |
| `SERVO4_AX12_MARGIN` | 3 | Dead-zone (raw units). Higher = stops fighting micro-errors under load |
| `SERVO4_TRAJECTORY_SPEED` | 15 | Independent speed — higher than global to maintain torque authority when extended |

### Trajectory timing (code 555 only)

| Constant | Default | Effect |
|---|---|---|
| `KEYFRAME_INTERVAL_MS` | 50 | Time between streamed waypoints. Must be less than how long each move takes |
| `SETTLE_MS` | 40 | Extra settle time after stop at pause boundaries |
| `RETRACT_PAUSE_MS` | 500 | Dwell at pen-lift / pen-plant boundaries |

---

## Re-exporting the Dynamic Trajectory Manually

If RViz is already running and you just want to re-capture without re-triggering:

```bash
cd online
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 tools/export_moveit_dynamic.py
```

Then trigger a plan from RViz or via the service call — the script will capture it and write the file.

**Available options:**

| Option | Default | Description |
|---|---|---|
| `--output PATH` | `robotMovements/moveItData.h` | Override output path |
| `--pause-dt-ms-threshold N` | 400 | Waypoints with `dt_ms >= N` flagged as pen-lift/plant boundaries |
| `--post-capture-sec N` | 3 | Silence window after last segment before writing |
| `--timeout-sec N` | 120 | Give up if nothing received after N seconds |

---

## Troubleshooting

**Arm homes immediately after starting code 777**
`moveItData.h` is the empty stub — the export did not run or failed. Check:
- Re-run `trigger_write.sh` and watch for `moveItData.h written successfully`
- Check serial monitor: `Waypoints loaded` should be > 50 and `Planned duration` > 5 s
- Reflash the Arduino after the export completes

**Jittery / lurching between waypoints (code 555)**
- Increase `KEYFRAME_INTERVAL_MS` so servos have more time to complete each move before the next arrives
- Or decrease `TRAJECTORY_SPEED` so each move takes longer than the interval

**Code 777 still lurching**
- The timing comes from MoveIt's `dt_ms` — if individual segments are very short the servos can't keep up; try raising `SERVO4_TRAJECTORY_SPEED` slightly or reducing `ACCEL_SMOOTH`

**Servo 1 oscillating (base rotation)**
- Lower `SERVO1_P_GAIN` (try 300) to reduce correction aggression
- Raise `SERVO1_D_GAIN` (try 500) to add more damping

**Servo 4 slow or bouncing when arm is extended**
- Raise `SERVO4_TRAJECTORY_SPEED` (try 18) so it has torque authority against gravity
- Raise `SERVO4_AX12_MARGIN` (try 4) to widen the dead-zone and stop micro-hunting
