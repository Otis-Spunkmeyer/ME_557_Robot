#!/usr/bin/env python3
"""Legacy debug tool: generate ACE joint samples via IK only (no motion planning)."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

EXPECTED_JOINTS = [
    "Motor1_joint",
    "Motor2_L",
    "Motor4_elb",
    "Motor5_wr",
    "Joint_EE",
]


def inch(v: float) -> float:
    return v * 0.0254


@dataclass
class Sample:
    dt_ms: int
    j1_rad: float
    j2l_rad: float
    j4_rad: float
    j5_rad: float
    j6_rad: float


def write_header(samples: list[Sample], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as f:
        f.write("#pragma once\n#include <stdint.h>\n#include <stddef.h>\n\n")
        f.write("typedef struct {\n")
        f.write("  uint16_t dt_ms;\n")
        f.write("  float j1_rad;\n  float j2l_rad;\n  float j4_rad;\n  float j5_rad;\n  float j6_rad;\n")
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


class AceIkExporter(Node):
    def __init__(self, group: str, base_link: str, tip_link: str):
        super().__init__("ace_ik_exporter")
        self.group = group
        self.base_link = base_link
        self.tip_link = tip_link
        self._last_joint_state: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

    def _js_cb(self, msg: JointState) -> None:
        if msg.name and msg.position:
            self._last_joint_state = msg

    def wait_ready(self, timeout_sec: float = 5.0) -> None:
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("MoveIt /compute_ik service not available.")
        deadline = self.get_clock().now() + RclpyDuration(seconds=timeout_sec)
        while self._last_joint_state is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._last_joint_state is None:
            raise RuntimeError("/joint_states not available.")

    def current_tip_pose(self, timeout_sec: float = 3.0):
        deadline = self.get_clock().now() + RclpyDuration(seconds=timeout_sec)
        while self.get_clock().now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_link,
                    self.tip_link,
                    rclpy.time.Time(),
                    timeout=RclpyDuration(seconds=0.2),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                return (float(t.x), float(t.y), float(t.z)), (
                    float(q.x),
                    float(q.y),
                    float(q.z),
                    float(q.w),
                )
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f"TF {self.base_link}->{self.tip_link} unavailable.")

    def solve_ik(
        self,
        xyz,
        quat_xyzw,
        seed: JointState,
        timeout_sec: float = 1.5,
        ik_link_name: str | None = None,
    ) -> tuple[list[float] | None, int]:
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group
        req.ik_request.ik_link_name = ik_link_name if ik_link_name else self.tip_link
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base_link
        req.ik_request.pose_stamped.pose.position.x = float(xyz[0])
        req.ik_request.pose_stamped.pose.position.y = float(xyz[1])
        req.ik_request.pose_stamped.pose.position.z = float(xyz[2])
        req.ik_request.pose_stamped.pose.orientation.x = float(quat_xyzw[0])
        req.ik_request.pose_stamped.pose.orientation.y = float(quat_xyzw[1])
        req.ik_request.pose_stamped.pose.orientation.z = float(quat_xyzw[2])
        req.ik_request.pose_stamped.pose.orientation.w = float(quat_xyzw[3])
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.timeout = Duration(
            sec=int(timeout_sec), nanosec=int((timeout_sec % 1.0) * 1e9)
        )
        req.ik_request.avoid_collisions = False

        fut = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec + 0.5)
        if not fut.done() or fut.result() is None:
            return None, -1
        resp = fut.result()
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, int(resp.error_code.val)

        idx = {n: i for i, n in enumerate(resp.solution.joint_state.name)}
        if not all(n in idx for n in EXPECTED_JOINTS):
            return None, -2
        return [float(resp.solution.joint_state.position[idx[n]]) for n in EXPECTED_JOINTS], int(
            resp.error_code.val
        )

    def solve_ik_multi(
        self,
        xyz,
        quats: list[tuple[float, float, float, float]],
        seed: JointState,
        timeout_sec: float,
        ik_links: list[str],
    ) -> tuple[list[float] | None, int, tuple[float, float, float, float] | None, str | None]:
        last_err = -1
        for link in ik_links:
            for q in quats:
                sol, err = self.solve_ik(
                    xyz, q, seed, timeout_sec=timeout_sec, ik_link_name=link
                )
                if sol is not None:
                    return sol, int(err), q, link
                last_err = int(err)
        return None, last_err, None, None


def orientation_candidates(base_q: tuple[float, float, float, float]) -> list[tuple[float, float, float, float]]:
    # A small bank of orientation candidates to avoid brittle single-orientation IK failures.
    return [
        base_q,
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.70710678, 0.70710678),
        (0.0, 0.0, -0.70710678, 0.70710678),
        (0.0, 0.0, 1.0, 0.0),
        (0.70710678, 0.0, 0.0, 0.70710678),
        (-0.70710678, 0.0, 0.0, 0.70710678),
        (0.0, 0.70710678, 0.0, 0.70710678),
        (0.0, -0.70710678, 0.0, 0.70710678),
    ]


def build_ace_semantic_plan():
    forward_draw = inch(16.1)
    forward_ret = inch(14.5)
    z_bot = inch(8.0)
    z_top = z_bot + inch(2.0)
    z_mid = (z_top + z_bot) * 0.5
    z_top_margin = z_top - inch(0.1)
    z_upper_mid = z_bot + inch(1.6)
    z_lower_mid = z_bot + inch(0.4)
    z_bot_margin = z_bot + inch(0.1)
    return [
        (forward_ret, -0.08255, z_bot),
        (forward_draw, -0.08255, z_bot),
        (forward_draw, -0.08255, z_bot),
        (forward_draw, -0.06350, z_top),
        (forward_draw, -0.04445, z_bot),
        (forward_draw, -0.07303, z_mid),
        (forward_draw, -0.05398, z_mid),
        (forward_ret, -0.05398, z_mid),
        (forward_ret, 0.01524, z_top_margin),
        (forward_draw, 0.01524, z_top_margin),
        (forward_draw, 0.01524, z_top_margin),
        (forward_draw, 0.00762, z_top),
        (forward_draw, -0.01016, z_top),
        (forward_draw, -0.01905, z_upper_mid),
        (forward_draw, -0.01905, z_lower_mid),
        (forward_draw, -0.01016, z_bot),
        (forward_draw, 0.00762, z_bot),
        (forward_draw, 0.01524, z_bot_margin),
        (forward_ret, 0.01524, z_bot_margin),
        (forward_ret, 0.04445, z_top),
        (forward_draw, 0.04445, z_top),
        (forward_draw, 0.04445, z_top),
        (forward_draw, 0.04445, z_bot),
        (forward_draw, 0.04445, z_top),
        (forward_draw, 0.08255, z_top),
        (forward_draw, 0.04445, z_mid),
        (forward_draw, 0.07493, z_mid),
        (forward_draw, 0.04445, z_bot),
        (forward_draw, 0.08255, z_bot),
        (forward_ret, 0.08255, z_bot),
    ]


def main() -> int:
    ap = argparse.ArgumentParser()
    default_output = (
        Path(__file__).resolve().parents[1]
        / "arduino"
        / "me557_pen_arduino_ws"
        / "ace_trajectory_data.h"
    )
    ap.add_argument("--output", default=str(default_output))
    ap.add_argument("--group", default="Write")
    ap.add_argument("--base-link", default="base_link")
    ap.add_argument("--tip-link", default="Pen_tip")
    ap.add_argument("--ik-links", nargs="+", default=None)
    ap.add_argument("--scales", nargs="+", type=float, default=[0.2, 0.1, 0.05, 0.02, 0.01])
    ap.add_argument(
        "--xy-signs",
        nargs="+",
        default=["-1,-1", "-1,1", "1,-1", "1,1"],
        help="Candidate sign mappings as sx,sy for (lateral,forward) deltas.",
    )
    ap.add_argument("--dt-ms", type=int, default=120)
    ap.add_argument("--max-step-rad", type=float, default=0.02)
    args = ap.parse_args()

    rclpy.init()
    node = AceIkExporter(args.group, args.base_link, args.tip_link)
    try:
        node.get_logger().warn(
            "This exporter is IK-only and does not produce collision-aware motion plans. "
            "Use write_ace/export_offline_ace.sh for planning-safe trajectories."
        )
        node.wait_ready()
        anchor_xyz, anchor_q = node.current_tip_pose()
        quat_bank = orientation_candidates(anchor_q)
        ik_links = args.ik_links if args.ik_links else [args.tip_link, "Link6", "Pen_tip"]
        # preserve order but unique
        seen_links = set()
        ik_links = [l for l in ik_links if not (l in seen_links or seen_links.add(l))]
        semantic = build_ace_semantic_plan()
        anchor_sem = semantic[0]

        seed = node._last_joint_state
        assert seed is not None

        sign_candidates: list[tuple[int, int]] = []
        for token in args.xy_signs:
            a, b = token.split(",")
            sx = -1 if int(a) < 0 else 1
            sy = -1 if int(b) < 0 else 1
            sign_candidates.append((sx, sy))

        solved: list[list[float]] | None = None
        chosen_scale = None
        chosen_signs = None
        first_fail_details = None
        for sx, sy in sign_candidates:
            for s in args.scales:
                seq: list[list[float]] = []
                seed_js = seed
                ok = True
                for i, (fwd, lat, z) in enumerate(semantic):
                    x = anchor_xyz[0] + s * (sx * (lat - anchor_sem[1]))
                    y = anchor_xyz[1] + s * (sy * (fwd - anchor_sem[0]))
                    zz = anchor_xyz[2] + s * (z - anchor_sem[2])
                    j, err, used_q, used_link = node.solve_ik_multi(
                        (x, y, zz),
                        quat_bank,
                        seed_js,
                        timeout_sec=1.5,
                        ik_links=ik_links,
                    )
                    if j is None:
                        ok = False
                        if first_fail_details is None:
                            first_fail_details = (s, i, x, y, zz, err, ik_links, sx, sy)
                        break
                    if i == 0 and used_q is not None:
                        node.get_logger().info(
                            "IK branch selected: "
                            f"link={used_link} "
                            f"quat=[{used_q[0]:.3f},{used_q[1]:.3f},{used_q[2]:.3f},{used_q[3]:.3f}] "
                            f"signs=({sx},{sy}) scale={s}"
                        )
                    seq.append(j)
                    seed_js = JointState(name=EXPECTED_JOINTS, position=j)
                if ok:
                    solved = seq
                    chosen_scale = s
                    chosen_signs = (sx, sy)
                    break
            if solved is not None:
                break

        if not solved:
            if first_fail_details is not None:
                s, i, x, y, zz, err, tried_links, sx, sy = first_fail_details
                node.get_logger().error(
                    "IK failed for all tested scales. "
                    f"First failure: scale={s}, waypoint={i}, "
                    f"target=({x:.4f},{y:.4f},{zz:.4f}), error_code={err}, "
                    f"ik_links={tried_links}, signs=({sx},{sy})"
                )
            else:
                node.get_logger().error("IK failed for all tested scales.")
            return 1

        samples: list[Sample] = []
        prev = solved[0]
        samples.append(Sample(args.dt_ms, *prev))
        for curr in solved[1:]:
            max_diff = max(abs(curr[i] - prev[i]) for i in range(5))
            steps = max(1, int(math.ceil(max_diff / max(1e-6, args.max_step_rad))))
            for k in range(1, steps + 1):
                a = k / steps
                v = [prev[i] + a * (curr[i] - prev[i]) for i in range(5)]
                samples.append(Sample(args.dt_ms, *v))
            prev = curr

        # collapse exact duplicates if any
        out: list[Sample] = [samples[0]]
        for s in samples[1:]:
            p = out[-1]
            if (
                abs(p.j1_rad - s.j1_rad) < 1e-8
                and abs(p.j2l_rad - s.j2l_rad) < 1e-8
                and abs(p.j4_rad - s.j4_rad) < 1e-8
                and abs(p.j5_rad - s.j5_rad) < 1e-8
                and abs(p.j6_rad - s.j6_rad) < 1e-8
            ):
                out[-1] = Sample(min(65535, p.dt_ms + s.dt_ms), p.j1_rad, p.j2l_rad, p.j4_rad, p.j5_rad, p.j6_rad)
            else:
                out.append(s)

        out_path = Path(args.output)
        if not out_path.is_absolute():
            out_path = Path.cwd() / out_path
        write_header(out, out_path)
        node.get_logger().info(
            "Generated offline IK ACE trajectory: "
            f"scale={chosen_scale}, signs={chosen_signs}, samples={len(out)}, output={out_path}"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
