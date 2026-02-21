#!/usr/bin/env python3
"""Capture a live MoveIt trajectory and write online/robotMovements/moveItData.h.

This tool subscribes to the MoveIt planned-path topic, captures joint positions,
velocities, and timing (dt_ms), then writes a C header for Arduino code 777
(playMoveitDynamicTrajectory).

Usage — with ROS 2 and the robot stack running:

  # Source the workspace first:
  source online/install/setup.bash

  # Capture the next trajectory that MoveIt plans and writes:
  python3 online/tools/export_moveit_dynamic.py

  # Override output path:
  python3 online/tools/export_moveit_dynamic.py \
    --output online/robotMovements/moveItData.h

  # Tighten pause detection (flag waypoints >= 300 ms as pen-lift boundaries):
  python3 online/tools/export_moveit_dynamic.py --pause-dt-ms-threshold 300

How pause detection works:
  MoveIt naturally slows down (large dt_ms) at pen-lift and pen-plant moments.
  Any waypoint with dt_ms >= pause_dt_ms_threshold is tagged pause=1 in the
  header.  The Arduino firmware uses this to call waitForMotionComplete() at
  those points and then dwell for RETRACT_PAUSE_MS before continuing.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

EXPECTED_JOINTS = [
    "Motor1_joint",
    "Motor2_L",
    "Motor4_elb",
    "Motor5_wr",
    "Joint_EE",
]

DEFAULT_OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "robotMovements"
    / "moveItData.h"
)


@dataclass
class Sample:
    dt_ms: int
    j1_rad: float
    j2l_rad: float
    j4_rad: float
    j5_rad: float
    j6_rad: float
    v1_rads: float = 0.0   # planned joint velocity (rad/s) — 0 if not available
    v2l_rads: float = 0.0
    v4_rads: float = 0.0
    v5_rads: float = 0.0
    v6_rads: float = 0.0


def _same_pose(a: Sample, b: Sample, eps: float = 1e-6) -> bool:
    return (
        abs(a.j1_rad  - b.j1_rad)  <= eps
        and abs(a.j2l_rad - b.j2l_rad) <= eps
        and abs(a.j4_rad  - b.j4_rad)  <= eps
        and abs(a.j5_rad  - b.j5_rad)  <= eps
        and abs(a.j6_rad  - b.j6_rad)  <= eps
    )


def compress_holds(samples: list[Sample]) -> list[Sample]:
    """Merge consecutive identical-pose waypoints, summing their dt_ms."""
    if not samples:
        return []
    out: list[Sample] = [samples[0]]
    for s in samples[1:]:
        last = out[-1]
        if _same_pose(last, s):
            out[-1] = Sample(
                dt_ms=min(65535, last.dt_ms + s.dt_ms),
                j1_rad=last.j1_rad, j2l_rad=last.j2l_rad,
                j4_rad=last.j4_rad, j5_rad=last.j5_rad, j6_rad=last.j6_rad,
                v1_rads=last.v1_rads, v2l_rads=last.v2l_rads,
                v4_rads=last.v4_rads, v5_rads=last.v5_rads, v6_rads=last.v6_rads,
            )
        else:
            out.append(s)
    return out


class DynamicCapture(Node):
    def __init__(self, traj_topic: str, display_topic: str) -> None:
        super().__init__("moveit_dynamic_capture")
        self.samples: list[Sample] = []          # accumulated across ALL segments
        self._last_key: tuple[float, ...] | None = None
        self._last_received_wall: float | None = None  # wall time of last message
        self._traj_topic_seen = False            # prefer traj_topic over display

        self.create_subscription(JointTrajectory, traj_topic, self._traj_cb, 10)
        self.create_subscription(DisplayTrajectory, display_topic, self._display_cb, 10)
        self.get_logger().info(
            f"Waiting for trajectories on '{traj_topic}' or '{display_topic}' ..."
        )

    def _append_segment(self, new_samples: list[Sample], source: str) -> None:
        """Append a new trajectory segment to the accumulated sample list."""
        if not new_samples:
            return
        merged = compress_holds(self.samples + new_samples)
        added = len(merged) - len(self.samples)
        self.samples = merged
        self._last_received_wall = time.monotonic()
        self.get_logger().info(
            f"[{source}] +{added} waypoints (total={len(self.samples)})"
        )

    # ------------------------------------------------------------------
    # trajectory_topic callback — preferred: has proper time_from_start
    # ------------------------------------------------------------------
    def _traj_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        idx = {name: i for i, name in enumerate(msg.joint_names)}
        if not all(name in idx for name in EXPECTED_JOINTS):
            self.get_logger().warn(
                f"Skipping trajectory — unexpected joints: {msg.joint_names}"
            )
            return

        self._traj_topic_seen = True
        new_samples: list[Sample] = []
        prev_t_ms = 0
        for pt in msg.points:
            t_ms = int(pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1e6)
            dt_ms = max(50, t_ms - prev_t_ms) if t_ms > prev_t_ms else 50
            prev_t_ms = max(prev_t_ms, t_ms)

            vals = [float(pt.positions[idx[name]]) for name in EXPECTED_JOINTS]
            key = tuple(round(v, 6) for v in vals)
            if key == self._last_key:
                continue
            self._last_key = key

            vels = list(pt.velocities) if pt.velocities else []
            v = [float(vels[idx[name]]) if vels else 0.0 for name in EXPECTED_JOINTS]

            new_samples.append(Sample(
                dt_ms=dt_ms,
                j1_rad=vals[0], j2l_rad=vals[1], j4_rad=vals[2],
                j5_rad=vals[3], j6_rad=vals[4],
                v1_rads=v[0], v2l_rads=v[1], v4_rads=v[2],
                v5_rads=v[3], v6_rads=v[4],
            ))

        self._append_segment(new_samples, "trajectory_topic")

    # ------------------------------------------------------------------
    # display_planned_path fallback — only used if traj_topic never fires
    # ------------------------------------------------------------------
    def _display_cb(self, msg: DisplayTrajectory) -> None:
        if self._traj_topic_seen:
            return  # traj_topic is richer, don't mix sources
        for robot_traj in msg.trajectory:
            traj = robot_traj.joint_trajectory
            if not traj.joint_names or not traj.points:
                continue
            idx = {name: i for i, name in enumerate(traj.joint_names)}
            if not all(name in idx for name in EXPECTED_JOINTS):
                continue

            new_samples: list[Sample] = []
            prev_t_ms = 0
            for pt in traj.points:
                t_ms = int(pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1e6)
                dt_ms = max(50, t_ms - prev_t_ms) if t_ms > prev_t_ms else 50
                prev_t_ms = max(prev_t_ms, t_ms)

                vals = [float(pt.positions[idx[name]]) for name in EXPECTED_JOINTS]
                vels = list(pt.velocities) if pt.velocities else []
                v = [float(vels[idx[name]]) if vels else 0.0 for name in EXPECTED_JOINTS]

                new_samples.append(Sample(
                    dt_ms=dt_ms,
                    j1_rad=vals[0], j2l_rad=vals[1], j4_rad=vals[2],
                    j5_rad=vals[3], j6_rad=vals[4],
                    v1_rads=v[0], v2l_rads=v[1], v4_rads=v[2],
                    v5_rads=v[3], v6_rads=v[4],
                ))

            self._append_segment(new_samples, "display_planned_path")


def write_header(samples: list[Sample], output: Path, pause_dt_ms: int) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n")
        f.write("#include <stddef.h>\n\n")
        f.write("// Auto-generated by online/tools/export_moveit_dynamic.py\n")
        f.write("// Format: MoveIt position + velocity + timing per waypoint.\n")
        f.write("// Used by Arduino code 777 (playMoveitDynamicTrajectory).\n\n")
        f.write("typedef struct {\n")
        f.write("  uint16_t dt_ms;     // MoveIt time budget (ms) for this move\n")
        f.write("  float    j1_rad;\n")
        f.write("  float    j2l_rad;\n")
        f.write("  float    j4_rad;\n")
        f.write("  float    j5_rad;\n")
        f.write("  float    j6_rad;\n")
        f.write("  float    v1_rads;   // planned joint velocity (rad/s)\n")
        f.write("  float    v2l_rads;\n")
        f.write("  float    v4_rads;\n")
        f.write("  float    v5_rads;\n")
        f.write("  float    v6_rads;\n")
        f.write("  uint8_t  pause;     // 1 = pen-lift/plant boundary\n")
        f.write("} MoveitDynamicSample;\n\n")

        f.write("static const MoveitDynamicSample kDynTrajectory[] = {\n")
        for s in samples:
            pause = 1 if s.dt_ms >= pause_dt_ms else 0
            f.write(
                "  { %d, %.8ff, %.8ff, %.8ff, %.8ff, %.8ff,"
                " %.6ff, %.6ff, %.6ff, %.6ff, %.6ff, %d },\n"
                % (
                    s.dt_ms,
                    s.j1_rad, s.j2l_rad, s.j4_rad, s.j5_rad, s.j6_rad,
                    s.v1_rads, s.v2l_rads, s.v4_rads, s.v5_rads, s.v6_rads,
                    pause,
                )
            )
        f.write("};\n\n")
        f.write(
            "static const size_t kDynTrajectoryCount =\n"
            "    sizeof(kDynTrajectory) / sizeof(kDynTrajectory[0]);\n"
        )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--output", default=str(DEFAULT_OUTPUT),
        help="Path to write moveItData.h (default: online/robotMovements/moveItData.h)",
    )
    p.add_argument(
        "--topic", default="/write_arm_controller/joint_trajectory",
        help="Primary trajectory topic",
    )
    p.add_argument(
        "--display-topic", default="/display_planned_path",
        help="Fallback DisplayTrajectory topic",
    )
    p.add_argument(
        "--pause-dt-ms-threshold", type=int, default=400,
        help="Waypoints with dt_ms >= this are flagged pause=1 (pen-lift/plant). "
             "Default: 400 ms.",
    )
    p.add_argument(
        "--timeout-sec", type=float, default=120.0,
        help="Give up waiting after this many seconds with no data. Default: 120.",
    )
    p.add_argument(
        "--post-capture-sec", type=float, default=3.0,
        help="Seconds of silence after the last trajectory message before writing. "
             "Increase if write_ace publishes many segments with long gaps. Default: 3.",
    )
    args = p.parse_args()

    rclpy.init()
    node = DynamicCapture(args.topic, args.display_topic)

    deadline = time.monotonic() + args.timeout_sec
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)

            # Once we have received at least one trajectory, wait for
            # post_capture_sec of silence — write_ace may publish multiple
            # segments and we want to accumulate all of them.
            if node._last_received_wall is not None:
                silence = time.monotonic() - node._last_received_wall
                if silence >= args.post_capture_sec:
                    node.get_logger().info(
                        f"No new segments for {args.post_capture_sec:.1f}s — capture complete."
                    )
                    break

            if time.monotonic() > deadline:
                if node.samples:
                    node.get_logger().warn(
                        f"Timeout reached — writing {len(node.samples)} waypoints captured so far."
                    )
                    break
                node.get_logger().error(
                    f"Timed out after {args.timeout_sec:.0f}s — no trajectory received."
                )
                node.destroy_node()
                rclpy.shutdown()
                return 1
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")

    if not node.samples:
        node.get_logger().error("No samples captured.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    output = Path(args.output)
    if not output.is_absolute():
        output = Path.cwd() / output

    write_header(node.samples, output, args.pause_dt_ms_threshold)
    node.get_logger().info(
        f"Wrote {len(node.samples)} waypoints to {output} "
        f"(pause_threshold={args.pause_dt_ms_threshold} ms)"
    )

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
