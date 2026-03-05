# ME557 OnlineWeights — 555 Trajectory Workflow

This workspace is now **555-only** (fixed-speed playback from `trajectory_data.h`).

## Workflow

1. Launch ROS 2 / MoveIt stack:

```bash
cd /home/rosubu/ME_557_Robot/onlineWeights
./launch_rviz.sh
```

2. Trigger planning and export trajectory header:

```bash
cd /home/rosubu/ME_557_Robot/onlineWeights
./trigger_write.sh
```

Each run calls `/board_writer/run_plan` and updates:

- `robotMovements/trajectory_data.h`

3. Flash Arduino sketch:

- `robotMovements/robotMovements.ino`

4. Run from Serial Monitor at `1000000` baud:

- `555` → play `trajectory_data.h`

## Notes

- `pause` flags in `trajectory_data.h` are honored by firmware:
  - `pause=1`: full stop + contact dwell
  - `pause=2`: full stop (corner settle), no dwell
  - `pause=3`: full stop + user pause dwell
- Current firmware uses synchronized Dynamixel start (`REG_WRITE + ACTION`) for better multi-joint coordination.

## Key files

- `robotMovements/robotMovements.ino`
- `robotMovements/trajectory_data.h`
- `src/me557_pen_description/me557_pen_description/write_on_board.py`
- `src/me557_pen_moveit_config/config/write_on_board.yaml`
