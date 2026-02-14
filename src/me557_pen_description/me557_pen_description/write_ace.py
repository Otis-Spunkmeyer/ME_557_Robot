#!/usr/bin/env python3

import math
import time
from threading import Thread
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

try:
    from pymoveit2 import MoveIt2
except ImportError as exc:
    MoveIt2 = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


def inch(value_in_inches: float) -> float:
    return value_in_inches * 0.0254


class AceWriter(Node):
    def __init__(self):
        super().__init__("ace_writer")

        if MoveIt2 is None:
            raise RuntimeError(
                "pymoveit2 is not installed. Install with: sudo apt-get install ros-humble-pymoveit2"
            ) from _IMPORT_ERROR

        # Settable parameters for robot and MoveIt setup.
        self.declare_parameter(
            "joint_names",
            ["Motor1_joint", "Motor2_L", "Motor4_elb", "Motor5_wr", "Joint_EE"],
        )
        self.declare_parameter("base_link_name", "base_link")
        self.declare_parameter("end_effector_name", "Pen_tip")
        self.declare_parameter("group_name", "Write")
        self.declare_parameter("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("execute_trajectory_action_name", "/execute_trajectory")
        self.declare_parameter("cartesian", True)
        self.declare_parameter("cartesian_max_step", 0.0025)
        self.declare_parameter("cartesian_fraction_threshold", 0.95)
        self.declare_parameter("min_trajectory_points", 2)
        self.declare_parameter("tolerance_position", 0.005)
        self.declare_parameter("tolerance_orientation", 3.14159)
        self.declare_parameter("weight_position", 1.0)
        self.declare_parameter("weight_orientation", 0.0)
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)
        self.declare_parameter("workspace_x_min", -inch(7.0))
        self.declare_parameter("workspace_x_max", inch(7.0))
        self.declare_parameter("workspace_enforce_x_bounds", False)
        self.declare_parameter("workspace_z_min", 0.0)
        self.declare_parameter("workspace_z_max", inch(14.0))
        self.declare_parameter("relative_to_current_pose", True)
        self.declare_parameter("relative_scale", 0.2)
        self.declare_parameter("skip_distance_epsilon", 0.0005)
        self.declare_parameter("fallback_to_noncartesian", True)
        self.declare_parameter("align_orientation_to_current_pose", True)
        self.declare_parameter("min_joint_span_rad", 1e-4)
        self.declare_parameter("plan_only_capture", False)
        self.declare_parameter(
            "trajectory_header_output",
            "arduino/me557_pen_arduino_ws/ace_trajectory_data.h",
        )

        joint_names = list(self.get_parameter("joint_names").value)
        base_link_name = str(self.get_parameter("base_link_name").value)
        end_effector_name = str(self.get_parameter("end_effector_name").value)
        group_name = str(self.get_parameter("group_name").value)
        quat_xyzw = list(self.get_parameter("quat_xyzw").value)
        self.execute_trajectory_action_name = str(
            self.get_parameter("execute_trajectory_action_name").value
        )
        self.use_cartesian = bool(self.get_parameter("cartesian").value)
        self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
        self.cartesian_fraction_threshold = float(
            self.get_parameter("cartesian_fraction_threshold").value
        )
        self.min_trajectory_points = int(self.get_parameter("min_trajectory_points").value)
        self.tolerance_position = float(self.get_parameter("tolerance_position").value)
        self.tolerance_orientation = float(
            self.get_parameter("tolerance_orientation").value
        )
        self.weight_position = float(self.get_parameter("weight_position").value)
        self.weight_orientation = float(self.get_parameter("weight_orientation").value)
        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)
        self.workspace_x_min = float(self.get_parameter("workspace_x_min").value)
        self.workspace_x_max = float(self.get_parameter("workspace_x_max").value)
        self.workspace_enforce_x_bounds = bool(
            self.get_parameter("workspace_enforce_x_bounds").value
        )
        self.workspace_z_min = float(self.get_parameter("workspace_z_min").value)
        self.workspace_z_max = float(self.get_parameter("workspace_z_max").value)
        self.relative_to_current_pose = bool(
            self.get_parameter("relative_to_current_pose").value
        )
        self.relative_scale = float(self.get_parameter("relative_scale").value)
        self.skip_distance_epsilon = float(
            self.get_parameter("skip_distance_epsilon").value
        )
        self.fallback_to_noncartesian = bool(
            self.get_parameter("fallback_to_noncartesian").value
        )
        self.align_orientation_to_current_pose = bool(
            self.get_parameter("align_orientation_to_current_pose").value
        )
        self.min_joint_span_rad = float(self.get_parameter("min_joint_span_rad").value)
        self.plan_only_capture = bool(self.get_parameter("plan_only_capture").value)
        self.trajectory_header_output = str(
            self.get_parameter("trajectory_header_output").value
        )
        self.base_link_name = base_link_name
        self.end_effector_name = end_effector_name
        self._expected_joints = [
            "Motor1_joint",
            "Motor2_L",
            "Motor4_elb",
            "Motor5_wr",
            "Joint_EE",
        ]
        self._joint_index_cache = {}
        self._captured_samples = []

        if len(quat_xyzw) != 4:
            raise ValueError("quat_xyzw must contain exactly 4 values [x, y, z, w]")
        self.quat_xyzw = [float(v) for v in quat_xyzw]
        if (
            self.plan_only_capture
            and self.use_cartesian
            and self.weight_orientation <= 0.0
        ):
            self.get_logger().warn(
                "plan_only_capture + cartesian + weight_orientation<=0 is unstable; "
                "using cartesian=False."
            )
            self.use_cartesian = False
        self.get_logger().info(
            "Planning mode "
            f"cartesian={self.use_cartesian} "
            f"max_step={self.cartesian_max_step:.4f} "
            f"fraction_threshold={self.cartesian_fraction_threshold:.3f} "
            f"min_points={self.min_trajectory_points} "
            f"tol_pos={self.tolerance_position:.4f} "
            f"tol_ori={self.tolerance_orientation:.3f} "
            f"w_pos={self.weight_position:.2f} "
            f"w_ori={self.weight_orientation:.2f} "
            f"offset=({self.offset_x:.4f},{self.offset_y:.4f},{self.offset_z:.4f}) "
            f"x_bounds_enforced={self.workspace_enforce_x_bounds} "
            f"x_bounds=[{self.workspace_x_min:.4f},{self.workspace_x_max:.4f}] "
            f"z_bounds=[{self.workspace_z_min:.4f},{self.workspace_z_max:.4f}] "
            f"relative={self.relative_to_current_pose} "
            f"relative_scale={self.relative_scale:.3f} "
            f"skip_eps={self.skip_distance_epsilon:.4f} "
            f"fallback_noncart={self.fallback_to_noncartesian} "
            f"align_ori={self.align_orientation_to_current_pose} "
            f"min_joint_span={self.min_joint_span_rad:g} "
            f"plan_only={self.plan_only_capture}"
        )

        self.callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name=base_link_name,
            end_effector_name=end_effector_name,
            group_name=group_name,
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # Run a background executor so action/service callbacks are processed.
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()
        if not self.plan_only_capture:
            try:
                self._ensure_single_execute_trajectory_server()
            except Exception:
                # If startup validation fails, cleanly stop background spinning
                # before re-raising so process exits without abort noise.
                try:
                    self._executor.shutdown(timeout_sec=1.0)
                except Exception:
                    pass
                try:
                    self.destroy_node()
                except Exception:
                    pass
                raise

        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(
            Marker, "/ace_writer/trajectory_marker", marker_qos
        )

        # Path dimensions.
        # Semantic frame for path points:
        # - forward: distance in front of robot (mapped to world -y)
        # - lateral: left/right across the writing surface (mapped to world x)
        # - z: up/down (mapped to world z)
        self.forward_draw = inch(16.1)
        self.forward_ret = inch(14.5)

        self.height = inch(2.0)
        self.z_bot = inch(8.0)
        self.z_mid = self.z_bot + self.height / 2.0
        self.z_top = self.z_bot + self.height
        self.z_top_margin = self.z_top - inch(0.1)
        self.z_upper_mid = self.z_bot + inch(1.6)
        self.z_lower_mid = self.z_bot + inch(0.4)
        self.z_bot_margin = self.z_bot + inch(0.1)
        self._semantic_plan = self._build_semantic_plan()

    def _ensure_single_execute_trajectory_server(self, timeout_sec: float = 5.0) -> None:
        action_name = self.execute_trajectory_action_name.rstrip("/")
        status_topic = f"{action_name}/_action/status"
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            pubs = self.get_publishers_info_by_topic(status_topic)
            servers = {
                f"{endpoint.node_namespace.rstrip('/')}/{endpoint.node_name}".replace("//", "/")
                for endpoint in pubs
            }
            known_servers = {
                s
                for s in servers
                if "_NODE_NAMESPACE_UNKNOWN_" not in s and "_NODE_NAME_UNKNOWN_" not in s
            }
            if len(known_servers) == 1:
                self.get_logger().info(
                    f"execute_trajectory server: {next(iter(known_servers))}"
                )
                return
            if len(servers) == 1:
                self.get_logger().info(
                    f"execute_trajectory server: {next(iter(servers))}"
                )
                return
            if len(servers) > 1:
                raise RuntimeError(
                    "Multiple execute_trajectory action servers detected. "
                    f"Expected 1, found {len(servers)} on '{action_name}': {sorted(servers)}. "
                    "Stop duplicate MoveIt launches and retry."
                )
            time.sleep(0.2)

        raise RuntimeError(
            "No execute_trajectory action server detected on "
            f"'{action_name}' within {timeout_sec:.1f}s. Is MoveIt demo.launch.py running?"
        )

    def _world_from_semantic(
        self,
        forward: float,
        lateral: float,
        z: float,
        anchor_semantic=None,
        anchor_world=None,
    ):
        if anchor_semantic is not None and anchor_world is not None:
            anchor_f, anchor_l, anchor_z = anchor_semantic
            ax, ay, az = anchor_world
            s = self.relative_scale
            x = ax + s * (-(lateral - anchor_l)) + self.offset_x
            y = ay + s * (-(forward - anchor_f)) + self.offset_y
            return x, y, az + s * (z - anchor_z) + self.offset_z

        # Flip lateral so text is readable from the front.
        x = -lateral + self.offset_x
        y = -forward + self.offset_y
        return x, y, z + self.offset_z

    def _apply_workspace_bounds(self, x: float, y: float, z: float):
        if self.workspace_enforce_x_bounds:
            x2 = min(self.workspace_x_max, max(self.workspace_x_min, x))
        else:
            x2 = x
        z2 = min(self.workspace_z_max, max(self.workspace_z_min, z))
        clipped = (abs(x2 - x) > 1e-12) or (abs(z2 - z) > 1e-12)
        return x2, y, z2, clipped

    def _lookup_current_ee_pose(self, timeout_sec: float = 3.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_link_name,
                    self.end_effector_name,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.2),
                )
                t = tf.transform.translation
                q = tf.transform.rotation
                return (
                    float(t.x),
                    float(t.y),
                    float(t.z),
                    [float(q.x), float(q.y), float(q.z), float(q.w)],
                )
            except TransformException:
                time.sleep(0.05)
        return None

    def _plan_pose(self, x: float, y: float, z: float, cartesian: bool):
        # With orientation disabled, use a position-only request.
        # Some cartesian plan calls raise instead of returning None.
        try:
            if self.weight_orientation <= 0.0:
                traj = self.moveit2.plan(
                    position=[x, y, z],
                    tolerance_position=self.tolerance_position,
                    weight_position=self.weight_position,
                    cartesian=cartesian,
                    max_step=self.cartesian_max_step,
                    cartesian_fraction_threshold=self.cartesian_fraction_threshold,
                )
            else:
                traj = self.moveit2.plan(
                    position=[x, y, z],
                    quat_xyzw=self.quat_xyzw,
                    tolerance_position=self.tolerance_position,
                    tolerance_orientation=self.tolerance_orientation,
                    weight_position=self.weight_position,
                    weight_orientation=self.weight_orientation,
                    cartesian=cartesian,
                    max_step=self.cartesian_max_step,
                    cartesian_fraction_threshold=self.cartesian_fraction_threshold,
                )
            return traj
        except Exception as exc:
            self.get_logger().warn(
                "Planner exception for "
                f"cartesian={cartesian} at x={x:.4f}, y={y:.4f}, z={z:.4f}: "
                f"{type(exc).__name__}: {exc}"
            )
            return None

    def _expected_joint_indices(self, joint_names):
        key = tuple(joint_names)
        if key in self._joint_index_cache:
            return self._joint_index_cache[key]

        index = []
        for name in self._expected_joints:
            try:
                index.append(joint_names.index(name))
            except ValueError:
                self._joint_index_cache[key] = None
                return None
        self._joint_index_cache[key] = index
        return index

    def _plan_and_execute_pose(self, x: float, y: float, z: float) -> None:
        traj = self._plan_pose(x, y, z, cartesian=self.use_cartesian)
        planned_cartesian = self.use_cartesian
        if (
            traj is None
            and self.use_cartesian
            and self.fallback_to_noncartesian
        ):
            self.get_logger().warn(
                "Cartesian plan failed; retrying this segment with non-cartesian planning."
            )
            traj = self._plan_pose(x, y, z, cartesian=False)
            planned_cartesian = False
        if traj is None:
            raise RuntimeError("Planning failed: no trajectory returned.")
        if len(traj.points) < self.min_trajectory_points:
            raise RuntimeError(
                "Planning returned a degenerate trajectory "
                f"({len(traj.points)} points, expected >= {self.min_trajectory_points})."
            )
        if traj.points:
            first = traj.points[0].positions
            last = traj.points[-1].positions
            if first and last and len(first) == len(last):
                max_span = max(abs(last[i] - first[i]) for i in range(len(first)))
                if max_span < self.min_joint_span_rad:
                    raise RuntimeError(
                        "Planning returned near-zero joint motion "
                        f"(max_span={max_span:.3e} rad < {self.min_joint_span_rad:.3e})."
                    )
        if planned_cartesian != self.use_cartesian:
            self.get_logger().info(
                f"segment planned with cartesian={planned_cartesian}"
            )
        self._append_planned_samples(traj)
        if self.plan_only_capture:
            return
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()

    def _append_planned_samples(self, traj) -> None:
        names = list(traj.joint_names)
        index = self._expected_joint_indices(names)
        if index is None:
            return

        for i, pt in enumerate(traj.points):
            vals = [float(pt.positions[j]) for j in index]
            if i == 0:
                dt_ms = 120 if not self._captured_samples else 60
            else:
                t0 = traj.points[i - 1].time_from_start
                t1 = pt.time_from_start
                dt_ms = int(
                    max(
                        20,
                        min(
                            2000,
                            round(
                                ((t1.sec - t0.sec) * 1e9 + (t1.nanosec - t0.nanosec))
                                / 1e6
                            ),
                        ),
                    )
                )
            self._captured_samples.append((dt_ms, vals))

    def _write_header(self) -> None:
        out = Path(self.trajectory_header_output)
        if not out.is_absolute():
            out = Path.cwd() / out
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as f:
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
            for dt_ms, vals in self._captured_samples:
                f.write(
                    "  { %d, %.8ff, %.8ff, %.8ff, %.8ff, %.8ff },\n"
                    % (dt_ms, vals[0], vals[1], vals[2], vals[3], vals[4])
                )
            f.write("};\n\n")
            f.write(
                "static const size_t kAceMoveitTrajectoryCount = "
                "sizeof(kAceMoveitTrajectory) / sizeof(kAceMoveitTrajectory[0]);\n"
            )
        self.get_logger().info(
            f"Wrote {len(self._captured_samples)} joint samples to {out}"
        )

    def _execute_segment_with_backoff(self, prev_xyz, target_xyz, mode: str):
        # Try full segment first, then progressively smaller fractions to keep motion feasible.
        scales = (1.0, 0.5, 0.25, 0.1, 0.05, 0.02)
        if prev_xyz is None:
            px, py, pz = target_xyz
        else:
            px, py, pz = prev_xyz
        tx, ty, tz = target_xyz
        last_err = None
        for s in scales:
            x = px + s * (tx - px)
            y = py + s * (ty - py)
            z = pz + s * (tz - pz)
            try:
                if s < 1.0:
                    self.get_logger().warn(
                        f"{mode}: retrying with shorter segment scale={s:.2f} "
                        f"to x={x:.4f} y={y:.4f} z={z:.4f}"
                    )
                self._plan_and_execute_pose(x, y, z)
                return (x, y, z), True
            except RuntimeError as exc:
                last_err = exc
                continue
        if last_err is not None:
            self.get_logger().error(
                f"{mode}: all retries failed for target x={tx:.4f} y={ty:.4f} z={tz:.4f}; "
                "keeping previous pose and continuing."
            )
            return prev_xyz, False
        self.get_logger().error(
            f"{mode}: segment failed at x={tx:.4f} y={ty:.4f} z={tz:.4f}; "
            "keeping previous pose and continuing."
        )
        return prev_xyz, False

    def _build_semantic_plan(self):
        return [
            # A
            ("travel", (self.forward_ret, -0.08255, self.z_bot)),
            ("travel", (self.forward_draw, -0.08255, self.z_bot)),
            ("draw", (self.forward_draw, -0.08255, self.z_bot)),
            ("draw", (self.forward_draw, -0.06350, self.z_top)),
            ("draw", (self.forward_draw, -0.04445, self.z_bot)),
            ("travel", (self.forward_draw, -0.07303, self.z_mid)),
            ("draw", (self.forward_draw, -0.05398, self.z_mid)),
            ("travel", (self.forward_ret, -0.05398, self.z_mid)),
            # C
            ("travel", (self.forward_ret, 0.01524, self.z_top_margin)),
            ("travel", (self.forward_draw, 0.01524, self.z_top_margin)),
            ("draw", (self.forward_draw, 0.01524, self.z_top_margin)),
            ("draw", (self.forward_draw, 0.00762, self.z_top)),
            ("draw", (self.forward_draw, -0.01016, self.z_top)),
            ("draw", (self.forward_draw, -0.01905, self.z_upper_mid)),
            ("draw", (self.forward_draw, -0.01905, self.z_lower_mid)),
            ("draw", (self.forward_draw, -0.01016, self.z_bot)),
            ("draw", (self.forward_draw, 0.00762, self.z_bot)),
            ("draw", (self.forward_draw, 0.01524, self.z_bot_margin)),
            ("travel", (self.forward_ret, 0.01524, self.z_bot_margin)),
            # E
            ("travel", (self.forward_ret, 0.04445, self.z_top)),
            ("travel", (self.forward_draw, 0.04445, self.z_top)),
            ("draw", (self.forward_draw, 0.04445, self.z_top)),
            ("draw", (self.forward_draw, 0.04445, self.z_bot)),
            ("travel", (self.forward_draw, 0.04445, self.z_top)),
            ("draw", (self.forward_draw, 0.08255, self.z_top)),
            ("travel", (self.forward_draw, 0.04445, self.z_mid)),
            ("draw", (self.forward_draw, 0.07493, self.z_mid)),
            ("travel", (self.forward_draw, 0.04445, self.z_bot)),
            ("draw", (self.forward_draw, 0.08255, self.z_bot)),
            ("travel", (self.forward_ret, 0.08255, self.z_bot)),
        ]

    def _resolve_world_targets(self, plan, anchor_semantic=None, anchor_world=None):
        resolved = []
        for mode, (forward, lateral, z) in plan:
            x, y, z2 = self._world_from_semantic(
                forward,
                lateral,
                z,
                anchor_semantic=anchor_semantic,
                anchor_world=anchor_world,
            )
            x, y, z2, clipped = self._apply_workspace_bounds(x, y, z2)
            resolved.append((mode, forward, lateral, x, y, z2, clipped))
        return resolved

    def run_plan(self) -> None:
        plan = self._semantic_plan

        anchor_semantic = None
        anchor_world = None
        if self.relative_to_current_pose:
            anchor_semantic = plan[0][1]
            current_pose = self._lookup_current_ee_pose(timeout_sec=3.0)
            if current_pose is None:
                self.get_logger().warn(
                    "Failed to read current end-effector transform; falling back to absolute path."
                )
                anchor_semantic = None
            else:
                anchor_world = current_pose[:3]
                if self.align_orientation_to_current_pose:
                    self.quat_xyzw = current_pose[3]
                self.get_logger().info(
                    "Using relative ACE origin at current end-effector pose: "
                    f"x={anchor_world[0]:.4f} y={anchor_world[1]:.4f} z={anchor_world[2]:.4f}"
                )
                if self.align_orientation_to_current_pose:
                    self.get_logger().info(
                        "Using current end-effector orientation for planning: "
                        f"[{self.quat_xyzw[0]:.4f}, {self.quat_xyzw[1]:.4f}, "
                        f"{self.quat_xyzw[2]:.4f}, {self.quat_xyzw[3]:.4f}]"
                    )

        world_targets = self._resolve_world_targets(
            plan, anchor_semantic=anchor_semantic, anchor_world=anchor_world
        )
        self._publish_trace_marker(world_targets)

        prev_xyz = anchor_world if anchor_world is not None else None
        attempted_segments = 0
        skipped_segments = 0
        executed_segments = 0
        failed_segments = 0
        bounded_segments = 0
        for mode, forward, lateral, x, y, z, clipped in world_targets:
            if clipped:
                bounded_segments += 1
            if prev_xyz is not None:
                if math.dist((x, y, z), prev_xyz) < self.skip_distance_epsilon:
                    self.get_logger().info(
                        f"skip: negligible segment at x={x:.4f} y={y:.4f} z={z:.4f}"
                    )
                    skipped_segments += 1
                    continue
            self.get_logger().info(
                f"{mode}: forward={forward:.4f} lateral={lateral:.4f} -> x={x:.4f} y={y:.4f} z={z:.4f}"
            )
            attempted_segments += 1
            prev_xyz, ok = self._execute_segment_with_backoff(
                prev_xyz=prev_xyz, target_xyz=(x, y, z), mode=mode
            )
            if ok:
                executed_segments += 1
            else:
                failed_segments += 1

        self.get_logger().info(
            "ACE_SUMMARY "
            f"attempted={attempted_segments} "
            f"executed={executed_segments} "
            f"failed={failed_segments} "
            f"skipped={skipped_segments} "
            f"bounded={bounded_segments}"
        )
        if executed_segments > 0 and self._captured_samples:
            self._write_header()

    def _publish_trace_marker(self, world_targets) -> None:
        travel_marker = Marker()
        travel_marker.header.frame_id = "base_link"
        travel_marker.header.stamp = self.get_clock().now().to_msg()
        travel_marker.ns = "ace_trace"
        travel_marker.id = 0
        travel_marker.type = Marker.LINE_LIST
        travel_marker.action = Marker.ADD
        travel_marker.scale.x = 0.003
        travel_marker.color.r = 1.0
        travel_marker.color.g = 0.8
        travel_marker.color.b = 0.1
        travel_marker.color.a = 1.0

        draw_marker = Marker()
        draw_marker.header.frame_id = "base_link"
        draw_marker.header.stamp = travel_marker.header.stamp
        draw_marker.ns = "ace_trace"
        draw_marker.id = 1
        draw_marker.type = Marker.LINE_LIST
        draw_marker.action = Marker.ADD
        draw_marker.scale.x = 0.004
        draw_marker.color.r = 0.1
        draw_marker.color.g = 0.9
        draw_marker.color.b = 1.0
        draw_marker.color.a = 1.0

        prev_pt = None
        for mode, _, _, x, y, z, _ in world_targets:
            curr_pt = Point(x=x, y=y, z=z)
            if prev_pt is not None:
                if mode == "draw":
                    draw_marker.points.append(prev_pt)
                    draw_marker.points.append(curr_pt)
                else:
                    travel_marker.points.append(prev_pt)
                    travel_marker.points.append(curr_pt)
            prev_pt = curr_pt

        self.marker_pub.publish(travel_marker)
        self.marker_pub.publish(draw_marker)


def main() -> None:
    rclpy.init()
    node = AceWriter()
    try:
        node.run_plan()
    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user.")
    finally:
        try:
            node._executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
