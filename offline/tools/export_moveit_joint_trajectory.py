#!/usr/bin/env python3
"""Capture one MoveIt run and write an Arduino trajectory header.

Example:
  python3 offline/tools/export_moveit_joint_trajectory.py \
    --run-write-ace \
    --output offline/arduino/me557_pen_arduino_ws/ace_trajectory_data.h
"""

from __future__ import annotations

import argparse
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

EXPECTED_JOINTS = [
    "Motor1_joint",
    "Motor2_L",
    "Motor4_elb",
    "Motor5_wr",
    "Joint_EE",
]
ACE_SUMMARY_RE = re.compile(
    r"ACE_SUMMARY\s+attempted=(\d+)\s+executed=(\d+)\s+failed=(\d+)\s+skipped=(\d+)"
)


@dataclass
class Sample:
    dt_ms: int
    j1_rad: float
    j2l_rad: float
    j4_rad: float
    j5_rad: float
    j6_rad: float


def _same_pose(a: Sample, b: Sample, eps: float) -> bool:
    return (
        abs(a.j1_rad - b.j1_rad) <= eps
        and abs(a.j2l_rad - b.j2l_rad) <= eps
        and abs(a.j4_rad - b.j4_rad) <= eps
        and abs(a.j5_rad - b.j5_rad) <= eps
        and abs(a.j6_rad - b.j6_rad) <= eps
    )


def compress_holds(samples: list[Sample], eps: float) -> list[Sample]:
    if not samples:
        return []
    out: list[Sample] = [samples[0]]
    for s in samples[1:]:
        last = out[-1]
        if _same_pose(last, s, eps):
            merged_dt = min(65535, int(last.dt_ms) + int(s.dt_ms))
            out[-1] = Sample(
                dt_ms=merged_dt,
                j1_rad=last.j1_rad,
                j2l_rad=last.j2l_rad,
                j4_rad=last.j4_rad,
                j5_rad=last.j5_rad,
                j6_rad=last.j6_rad,
            )
        else:
            out.append(s)
    return out


def unique_pose_count(samples: list[Sample], eps: float) -> int:
    seen: list[Sample] = []
    for s in samples:
        if not any(_same_pose(s, t, eps) for t in seen):
            seen.append(s)
    return len(seen)


def parse_ace_summary(log_text: str) -> dict[str, int] | None:
    m = ACE_SUMMARY_RE.search(log_text)
    if not m:
        return None
    attempted, executed, failed, skipped = (int(x) for x in m.groups())
    return {
        "attempted": attempted,
        "executed": executed,
        "failed": failed,
        "skipped": skipped,
    }


def consume_writer_output(
    writer_proc: subprocess.Popen[str] | None,
    writer_log: Path,
    logger,
) -> tuple[str, dict[str, int] | None]:
    if writer_proc is None or writer_proc.stdout is None:
        return "", None

    writer_output_text = writer_proc.stdout.read() or ""
    if not writer_output_text:
        return "", None

    writer_summary = parse_ace_summary(writer_output_text)
    try:
        writer_log.write_text(writer_output_text, encoding="utf-8")
    except Exception:
        pass
    for line in writer_output_text.splitlines():
        logger.info(f"[write_ace] {line}")
    return writer_output_text, writer_summary


class TrajectoryCapture(Node):
    def __init__(self, topic: str, controller_state_topic: str, display_topic: str):
        super().__init__("moveit_trajectory_capture")
        self.samples: list[Sample] = []
        self.display_samples: list[Sample] = []
        self.controller_state_samples: list[Sample] = []
        self.joint_state_samples: list[Sample] = []
        self._last_key: tuple[float, ...] | None = None
        self._last_display_key: tuple[float, ...] | None = None
        self._last_ctrl_key: tuple[float, ...] | None = None
        self._last_js_key: tuple[float, ...] | None = None
        self._last_sample_wall: float | None = None
        self._last_display_wall: float | None = None
        self._last_ctrl_wall: float | None = None
        self._last_js_wall: float | None = None
        self._saw_traj_topic = False
        self.create_subscription(JointTrajectory, topic, self._cb, 10)
        self.create_subscription(DisplayTrajectory, display_topic, self._display_cb, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            controller_state_topic,
            self._controller_state_cb,
            50,
        )
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 50)

    def _cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        self._saw_traj_topic = True

        idx = {name: i for i, name in enumerate(msg.joint_names)}
        if not all(name in idx for name in EXPECTED_JOINTS):
            self.get_logger().warn(f"Skipping trajectory with joints: {msg.joint_names}")
            return

        prev_t_ms = 0
        for pt in msg.points:
            t_ms = int(pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1e6)
            dt_ms = max(120, t_ms - prev_t_ms if t_ms > prev_t_ms else 0)
            prev_t_ms = max(prev_t_ms, t_ms)

            vals = [float(pt.positions[idx[name]]) for name in EXPECTED_JOINTS]
            key = tuple(round(v, 6) for v in vals)
            if key == self._last_key:
                continue
            self._last_key = key

            self.samples.append(
                Sample(
                    dt_ms=dt_ms,
                    j1_rad=vals[0],
                    j2l_rad=vals[1],
                    j4_rad=vals[2],
                    j5_rad=vals[3],
                    j6_rad=vals[4],
                )
            )

        self._last_sample_wall = time.monotonic()

    def _controller_state_cb(self, msg: JointTrajectoryControllerState) -> None:
        if not msg.joint_names or not msg.actual.positions:
            return
        idx = {name: i for i, name in enumerate(msg.joint_names)}
        if not all(name in idx for name in EXPECTED_JOINTS):
            return

        vals = [float(msg.actual.positions[idx[name]]) for name in EXPECTED_JOINTS]
        key = tuple(round(v, 6) for v in vals)
        if key == self._last_ctrl_key:
            return

        now = time.monotonic()
        if self._last_ctrl_wall is None:
            dt_ms = 120
        else:
            dt_ms = int((now - self._last_ctrl_wall) * 1000.0)
            dt_ms = max(120, min(2000, dt_ms))

        self._last_ctrl_key = key
        self._last_ctrl_wall = now
        self.controller_state_samples.append(
            Sample(
                dt_ms=dt_ms,
                j1_rad=vals[0],
                j2l_rad=vals[1],
                j4_rad=vals[2],
                j5_rad=vals[3],
                j6_rad=vals[4],
            )
        )

    def _display_cb(self, msg: DisplayTrajectory) -> None:
        if not msg.trajectory:
            return

        for robot_traj in msg.trajectory:
            traj = robot_traj.joint_trajectory
            if not traj.joint_names or not traj.points:
                continue
            idx = {name: i for i, name in enumerate(traj.joint_names)}
            if not all(name in idx for name in EXPECTED_JOINTS):
                continue

            prev_t_ms = 0
            for pt in traj.points:
                t_ms = int(pt.time_from_start.sec * 1000 + pt.time_from_start.nanosec / 1e6)
                dt_ms = max(120, t_ms - prev_t_ms if t_ms > prev_t_ms else 0)
                prev_t_ms = max(prev_t_ms, t_ms)

                vals = [float(pt.positions[idx[name]]) for name in EXPECTED_JOINTS]

                self.display_samples.append(
                    Sample(
                        dt_ms=dt_ms,
                        j1_rad=vals[0],
                        j2l_rad=vals[1],
                        j4_rad=vals[2],
                        j5_rad=vals[3],
                        j6_rad=vals[4],
                    )
                )
                self._last_display_wall = time.monotonic()

    def _joint_state_cb(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        idx = {name: i for i, name in enumerate(msg.name)}
        if not all(name in idx for name in EXPECTED_JOINTS):
            return

        vals = [float(msg.position[idx[name]]) for name in EXPECTED_JOINTS]
        key = tuple(round(v, 6) for v in vals)
        if key == self._last_js_key:
            return

        now = time.monotonic()
        if self._last_js_wall is None:
            dt_ms = 120
        else:
            dt_ms = int((now - self._last_js_wall) * 1000.0)
            dt_ms = max(120, min(2000, dt_ms))

        self._last_js_key = key
        self._last_js_wall = now
        self.joint_state_samples.append(
            Sample(
                dt_ms=dt_ms,
                j1_rad=vals[0],
                j2l_rad=vals[1],
                j4_rad=vals[2],
                j5_rad=vals[3],
                j6_rad=vals[4],
            )
        )


def write_header(samples: list[Sample], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    with output.open("w", encoding="utf-8") as f:
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n")
        f.write("#include <stddef.h>\n\n")
        f.write("typedef struct {\n")
        f.write("  uint16_t dt_ms;\n")
        f.write("  float j1_rad;\n")
        f.write("  float j2l_rad;\n")
        f.write("  float j4_rad;\n")
        f.write("  float j5_rad;\n")
        f.write("  float j6_rad;\n")
        f.write("} MoveitJointSampleRad;\n\n")

        f.write("static const MoveitJointSampleRad kAceMoveitTrajectory[] = {\n")
        for s in samples:
            f.write(
                "  { %d, %.8ff, %.8ff, %.8ff, %.8ff, %.8ff },\n"
                % (s.dt_ms, s.j1_rad, s.j2l_rad, s.j4_rad, s.j5_rad, s.j6_rad)
            )
        f.write("};\n\n")
        f.write(
            "static const size_t kAceMoveitTrajectoryCount = "
            "sizeof(kAceMoveitTrajectory) / sizeof(kAceMoveitTrajectory[0]);\n"
        )


def main() -> int:
    default_output = (
        Path(__file__).resolve().parents[1]
        / "arduino"
        / "me557_pen_arduino_ws"
        / "ace_trajectory_data.h"
    )
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/write_arm_controller/joint_trajectory")
    p.add_argument("--controller-state-topic", default="/write_arm_controller/controller_state")
    p.add_argument("--display-topic", default="/display_planned_path")
    p.add_argument("--output", default=str(default_output))
    p.add_argument("--writer-log", default="/tmp/write_ace_last.log")
    p.add_argument("--run-write-ace", action="store_true")
    p.add_argument("--post-wait-sec", type=float, default=1.0)
    p.add_argument("--timeout-sec", type=float, default=600.0)
    p.add_argument("--compress-eps", type=float, default=1e-6)
    p.add_argument("--no-compress-holds", action="store_false", dest="compress_holds")
    p.add_argument("--require-complete-ace", action="store_true")
    p.add_argument("--min-unique-points", type=int, default=5)
    p.add_argument("--writer-fraction-threshold", type=float, default=None)
    p.add_argument("--writer-min-points", type=int, default=None)
    p.add_argument("--writer-relative-scale", type=float, default=None)
    p.add_argument("--writer-cartesian", choices=["true", "false"], default=None)
    p.add_argument("--writer-fallback-noncartesian", choices=["true", "false"], default=None)
    p.add_argument("--writer-align-orientation", choices=["true", "false"], default=None)
    p.add_argument("--writer-tolerance-position", type=float, default=None)
    p.add_argument("--writer-min-joint-span", type=float, default=None)
    p.set_defaults(compress_holds=True)
    args = p.parse_args()

    writer_proc: subprocess.Popen[str] | None = None
    writer_exit_code: int | None = None
    writer_summary: dict[str, int] | None = None
    writer_output_text: str = ""
    writer_log_path = Path(args.writer_log)

    rclpy.init()
    node = TrajectoryCapture(args.topic, args.controller_state_topic, args.display_topic)

    try:
        if args.run_write_ace:
            writer_cmd = ["ros2", "run", "me557_pen_description", "write_ace"]
            ros_args: list[str] = []
            if args.writer_fraction_threshold is not None:
                ros_args += [
                    "-p",
                    f"cartesian_fraction_threshold:={args.writer_fraction_threshold}",
                ]
            if args.writer_min_points is not None:
                ros_args += ["-p", f"min_trajectory_points:={args.writer_min_points}"]
            if args.writer_relative_scale is not None:
                ros_args += ["-p", f"relative_scale:={args.writer_relative_scale}"]
            if args.writer_cartesian is not None:
                ros_args += ["-p", f"cartesian:={args.writer_cartesian}"]
            if args.writer_fallback_noncartesian is not None:
                ros_args += [
                    "-p",
                    f"fallback_to_noncartesian:={args.writer_fallback_noncartesian}",
                ]
            if args.writer_align_orientation is not None:
                ros_args += [
                    "-p",
                    f"align_orientation_to_current_pose:={args.writer_align_orientation}",
                ]
            if args.writer_tolerance_position is not None:
                ros_args += ["-p", f"tolerance_position:={args.writer_tolerance_position}"]
            if args.writer_min_joint_span is not None:
                ros_args += ["-p", f"min_joint_span_rad:={args.writer_min_joint_span}"]
            if ros_args:
                writer_cmd += ["--ros-args"] + ros_args
            writer_proc = subprocess.Popen(
                writer_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            node.get_logger().info("Started write_ace capture.")

        deadline = time.monotonic() + args.timeout_sec
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)

            if writer_proc is None:
                continue

            if writer_proc.poll() is not None:
                writer_exit_code = writer_proc.returncode
                writer_output_text, writer_summary = consume_writer_output(
                    writer_proc,
                    writer_log_path,
                    node.get_logger(),
                )
                post_wait_deadline = time.monotonic() + max(0.0, args.post_wait_sec)
                while time.monotonic() < post_wait_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                break

            if time.monotonic() > deadline:
                node.get_logger().warn(
                    f"Timed out at {args.timeout_sec:.1f}s, stopping writer."
                )
                writer_proc.terminate()
                break

    except KeyboardInterrupt:
        node.get_logger().info("Capture stopped.")
    finally:
        if writer_proc is not None and writer_proc.poll() is None:
            writer_proc.send_signal(signal.SIGINT)
            try:
                writer_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                writer_proc.kill()
        if not writer_output_text:
            writer_output_text, writer_summary = consume_writer_output(
                writer_proc,
                writer_log_path,
                node.get_logger(),
            )

        output = Path(args.output)
        if not output.is_absolute():
            output = Path.cwd() / output

        candidates = [
            ("trajectory_topic", node.samples),
            ("display_planned_path", node.display_samples),
            ("controller_state", node.controller_state_samples),
            ("joint_states", node.joint_state_samples),
        ]
        scored = []
        for name, data in candidates:
            raw_n = len(data)
            if raw_n == 0:
                scored.append((name, data, data, 0, 0))
                continue
            compressed = compress_holds(data, args.compress_eps)
            uniq_n = unique_pose_count(compressed, args.compress_eps)
            scored.append((name, data, compressed, raw_n, uniq_n))

        selected_name, selected_raw, selected_compressed, raw_n, uniq_n = max(
            scored, key=lambda t: (t[4], t[3])
        )
        node.get_logger().info(
            "Selected source "
            f"{selected_name}: raw={raw_n} unique_after_compress={uniq_n}"
        )

        selected_samples = selected_raw
        raw_count = len(selected_raw)
        if args.compress_holds:
            selected_samples = selected_compressed
            node.get_logger().info(
                f"Compressed hold samples: {raw_count} -> {len(selected_samples)} "
                f"(eps={args.compress_eps:g})"
            )
        unique_after = unique_pose_count(selected_samples, args.compress_eps)
        node.get_logger().info(
            f"Unique trajectory points after compression: {unique_after}"
        )

        if args.run_write_ace and writer_exit_code not in (None, 0):
            node.get_logger().error(
                f"write_ace exited with code {writer_exit_code}; proceeding with partial capture."
            )
            node.get_logger().error(f"write_ace log saved to {args.writer_log}")
        if writer_summary is not None:
            node.get_logger().info(
                "Parsed write_ace summary: "
                f"attempted={writer_summary['attempted']} "
                f"executed={writer_summary['executed']} "
                f"failed={writer_summary['failed']} "
                f"skipped={writer_summary['skipped']}"
            )

        incomplete_ace = False
        if args.require_complete_ace:
            if not args.run_write_ace:
                node.get_logger().error("--require-complete-ace requires --run-write-ace.")
                incomplete_ace = True
            elif writer_summary is None:
                node.get_logger().error(
                    "No ACE_SUMMARY found in write_ace output; cannot verify completeness."
                )
                incomplete_ace = True
            elif writer_summary["failed"] > 0:
                node.get_logger().error(
                    "ACE run incomplete: failed segments > 0. "
                    f"(failed={writer_summary['failed']})"
                )
                incomplete_ace = True
            elif unique_after < args.min_unique_points:
                node.get_logger().error(
                    "ACE run incomplete: unique motion points below threshold "
                    f"({unique_after} < {args.min_unique_points})."
                )
                incomplete_ace = True

        if not selected_samples:
            node.get_logger().error("No samples captured; header not written.")
            rc = 1
        elif incomplete_ace:
            node.get_logger().error(
                "Completeness gate failed; header not written. "
                "Re-run without --require-complete-ace to keep partial data."
            )
            rc = 1
        else:
            write_header(selected_samples, output)
            node.get_logger().info(f"Wrote {len(selected_samples)} samples to {output}")
            rc = 0 if writer_exit_code in (None, 0) else 2

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return rc


if __name__ == "__main__":
    sys.exit(main())
