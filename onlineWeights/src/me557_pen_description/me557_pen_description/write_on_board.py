#!/usr/bin/env python3

import json
import math
import time
from threading import Thread
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
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
        super().__init__("board_writer")

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
        self.declare_parameter("semantic_yaw_deg", 0.0)
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
        self.declare_parameter("max_joint_span_rad", 1.2)
        self.declare_parameter("plan_only_capture", False)
        self.declare_parameter(
            "trajectory_header_output",
            "/home/rosubu/ME_557_Robot/onlineWeights/robotMovements/trajectory_data.h",
        )
        self.declare_parameter("pen_lift_pause_sec", 0.2)
        self.declare_parameter("pen_lift_speed_scale", 0.3)
        self.declare_parameter("pen_retract_forward_m", inch(14.5))
        # pen_retract_forward_m: distance from robot base to the retracted pen position
        # (measured along the same axis as forward_draw).  Must be < forward_draw.
        # Default inch(14.5) = ~369mm.  If the spring-loaded tip still brushes the board
        # during lateral travel, decrease this value (e.g. inch(13.0) or inch(12.0))
        # to pull the arm further back before sweeping to the next letter.
        self.declare_parameter("pen_clearance_z_m", 0.012)
        # pen_clearance_z_m: retained for backwards-compatibility with existing YAML files.
        # No longer used for the pen-retract waypoint.  The retract now keeps (lateral, z)
        # fixed in semantic space so that the EE moves perpendicular to the board surface
        # (along _board_normal_hat), giving a clean lift with no surface sliding.
        # Increase if a short diagonal drag mark is still visible after the main fix.
        # Fraction of joint velocity/acceleration limits for planning.
        # Values < 1.0 leave headroom so the real servo can track the profile.
        self.declare_parameter("velocity_scaling", .75)
        self.declare_parameter("acceleration_scaling", 1.0)
        self.declare_parameter(
            "letter_coordinates_file",
            "/home/rosubu/ME_557_Robot/onlineWeights/letterCoordinates.json",
        )
        self.declare_parameter("scene_setup_enabled", False)
        self.declare_parameter("scene_setup_timeout_sec", 5.0)
        self.declare_parameter("apply_planning_scene_service", "/apply_planning_scene")
        self.declare_parameter("scene_frame_id", "")
        self.declare_parameter("scene_board_enabled", True)
        self.declare_parameter("scene_board_id", "writing_board")
        self.declare_parameter("scene_board_center_x", 0.0)
        # Front face of board is inch(16.1) forward; board is 0.01 m thick → center 0.005 m further
        self.declare_parameter("scene_board_center_y", -(inch(16.1) + 0.005))
        # Board bottom = inch(3.75), height = inch(10) → center at inch(8.75)
        self.declare_parameter("scene_board_center_z", inch(8.75))
        # Board is 13 in wide (lateral) × 10 in tall × 1 cm thick
        self.declare_parameter("scene_board_size_x", inch(13))
        self.declare_parameter("scene_board_size_y", 0.01)
        self.declare_parameter("scene_board_size_z", inch(10))
        # --- Four-corner board definition ---
        # Pre-filled to match: center=(0.4139,0.0,0.2286), size_y=0.45, size_z=0.40, yaw=90°
        #   half_width=0.225 m (along world Y), half_height=0.200 m (along world Z)
        self.declare_parameter("scene_board_corner_bl_x",  0.4139)
        self.declare_parameter("scene_board_corner_bl_y", -0.225)
        self.declare_parameter("scene_board_corner_bl_z",  0.0286)
        self.declare_parameter("scene_board_corner_br_x",  0.4139)
        self.declare_parameter("scene_board_corner_br_y",  0.225)
        self.declare_parameter("scene_board_corner_br_z",  0.0286)
        self.declare_parameter("scene_board_corner_tr_x",  0.4139)
        self.declare_parameter("scene_board_corner_tr_y",  0.225)
        self.declare_parameter("scene_board_corner_tr_z",  0.4286)
        self.declare_parameter("scene_board_corner_tl_x",  0.4139)
        self.declare_parameter("scene_board_corner_tl_y", -0.225)
        self.declare_parameter("scene_board_corner_tl_z",  0.4286)
        # Simple calibration block for quick trial-and-error tuning.
        # When enabled, you only change:
        #   - distance from robot (applied to all corner X values)
        #   - vertical/horizontal tilt (degrees)
        self.declare_parameter("scene_board_use_calibration_block", True)
        self.declare_parameter("scene_board_distance_from_robot_m", 0.4139)
        self.declare_parameter("scene_board_cal_tilt_vertical_deg", 0.0)
        self.declare_parameter("scene_board_cal_tilt_horizontal_deg", 0.0)
        # Tilt angles (degrees) — applied after corners are resolved
        self.declare_parameter("scene_board_tilt_vertical_deg",   0.0)
        self.declare_parameter("scene_board_tilt_horizontal_deg", 0.0)
        # Board thickness (metres) — depth of the collision box along the normal axis
        self.declare_parameter("scene_board_thickness", 0.01)
        self.declare_parameter("scene_table_enabled", False)
        self.declare_parameter("scene_table_id", "writing_table")
        self.declare_parameter("scene_table_center_x", 0.0)
        self.declare_parameter("scene_table_center_y", -0.34)
        self.declare_parameter("scene_table_center_z", 0.10)
        self.declare_parameter("scene_table_size_x", 0.60)
        self.declare_parameter("scene_table_size_y", 0.60)
        self.declare_parameter("scene_table_size_z", 0.05)

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
        self.semantic_yaw_deg = float(self.get_parameter("semantic_yaw_deg").value)
        self.semantic_yaw_rad = math.radians(self.semantic_yaw_deg)
        self._semantic_yaw_c = math.cos(self.semantic_yaw_rad)
        self._semantic_yaw_s = math.sin(self.semantic_yaw_rad)
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
        self.max_joint_span_rad = float(self.get_parameter("max_joint_span_rad").value)
        self.plan_only_capture = bool(self.get_parameter("plan_only_capture").value)
        self.trajectory_header_output = str(
            self.get_parameter("trajectory_header_output").value
        )
        self.pen_lift_pause_sec = float(self.get_parameter("pen_lift_pause_sec").value)
        self.pen_lift_speed_scale = float(self.get_parameter("pen_lift_speed_scale").value)
        self.pen_retract_forward_m = float(self.get_parameter("pen_retract_forward_m").value)
        self.pen_clearance_z_m = float(self.get_parameter("pen_clearance_z_m").value)
        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        self.letter_coordinates_file = str(
            self.get_parameter("letter_coordinates_file").value
        )
        self.scene_setup_enabled = bool(self.get_parameter("scene_setup_enabled").value)
        self.scene_setup_timeout_sec = float(
            self.get_parameter("scene_setup_timeout_sec").value
        )
        self.apply_planning_scene_service = str(
            self.get_parameter("apply_planning_scene_service").value
        )
        self.scene_frame_id = str(self.get_parameter("scene_frame_id").value).strip()
        self.scene_board_enabled = bool(self.get_parameter("scene_board_enabled").value)
        self.scene_board_id = str(self.get_parameter("scene_board_id").value)
        self.scene_board_center = (
            float(self.get_parameter("scene_board_center_x").value),
            float(self.get_parameter("scene_board_center_y").value),
            float(self.get_parameter("scene_board_center_z").value),
        )
        self.scene_board_size = (
            float(self.get_parameter("scene_board_size_x").value),
            float(self.get_parameter("scene_board_size_y").value),
            float(self.get_parameter("scene_board_size_z").value),
        )
        self.scene_board_corners = {
            "bl": (
                float(self.get_parameter("scene_board_corner_bl_x").value),
                float(self.get_parameter("scene_board_corner_bl_y").value),
                float(self.get_parameter("scene_board_corner_bl_z").value),
            ),
            "br": (
                float(self.get_parameter("scene_board_corner_br_x").value),
                float(self.get_parameter("scene_board_corner_br_y").value),
                float(self.get_parameter("scene_board_corner_br_z").value),
            ),
            "tr": (
                float(self.get_parameter("scene_board_corner_tr_x").value),
                float(self.get_parameter("scene_board_corner_tr_y").value),
                float(self.get_parameter("scene_board_corner_tr_z").value),
            ),
            "tl": (
                float(self.get_parameter("scene_board_corner_tl_x").value),
                float(self.get_parameter("scene_board_corner_tl_y").value),
                float(self.get_parameter("scene_board_corner_tl_z").value),
            ),
        }
        self.scene_board_use_calibration_block = bool(
            self.get_parameter("scene_board_use_calibration_block").value
        )
        self.scene_board_distance_from_robot_m = float(
            self.get_parameter("scene_board_distance_from_robot_m").value
        )
        self.scene_board_cal_tilt_vertical_deg = float(
            self.get_parameter("scene_board_cal_tilt_vertical_deg").value
        )
        self.scene_board_cal_tilt_horizontal_deg = float(
            self.get_parameter("scene_board_cal_tilt_horizontal_deg").value
        )
        self.scene_board_tilt_vertical_deg   = float(self.get_parameter("scene_board_tilt_vertical_deg").value)
        self.scene_board_tilt_horizontal_deg = float(self.get_parameter("scene_board_tilt_horizontal_deg").value)
        self.scene_board_thickness           = float(self.get_parameter("scene_board_thickness").value)

        if self.scene_board_use_calibration_block:
            # 1) Distance from robot: apply one X value to all four board corners.
            # This keeps board width/height intact while moving the plane in/out.
            x = self.scene_board_distance_from_robot_m
            self.scene_board_corners = {
                key: (x, value[1], value[2])
                for key, value in self.scene_board_corners.items()
            }
            # 2) Tilt overrides from the quick calibration block.
            self.scene_board_tilt_vertical_deg = self.scene_board_cal_tilt_vertical_deg
            self.scene_board_tilt_horizontal_deg = self.scene_board_cal_tilt_horizontal_deg

            self.get_logger().info(
                "[board_calibration] using calibration block: "
                f"distance={self.scene_board_distance_from_robot_m:.4f} m, "
                f"tilt_v={self.scene_board_tilt_vertical_deg:.2f}°, "
                f"tilt_h={self.scene_board_tilt_horizontal_deg:.2f}°"
            )
        # Compute the tilted board frame immediately so waypoint generation
        # uses the same orientation that is applied to the collision box.
        # Must be called after corners/tilts are stored; forward_draw is set
        # later in __init__, so store a temporary sentinel and recompute then.
        self._board_centre       = None  # set by _setup_board_frame()
        self._board_face_centre  = None
        self._board_approach_hat = None  # pre-tilt normal (retract/extend direction)
        self._board_width_hat   = None
        self._board_height_hat  = None
        self._board_normal_hat  = None
        self.scene_table_enabled = bool(self.get_parameter("scene_table_enabled").value)
        self.scene_table_id = str(self.get_parameter("scene_table_id").value)
        self.scene_table_center = (
            float(self.get_parameter("scene_table_center_x").value),
            float(self.get_parameter("scene_table_center_y").value),
            float(self.get_parameter("scene_table_center_z").value),
        )
        self.scene_table_size = (
            float(self.get_parameter("scene_table_size_x").value),
            float(self.get_parameter("scene_table_size_y").value),
            float(self.get_parameter("scene_table_size_z").value),
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
        if self.use_cartesian and self.weight_orientation <= 0.0:
            self.get_logger().warn(
                "cartesian + weight_orientation<=0 is unstable with current pymoveit2; "
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
            f"semantic_yaw_deg={self.semantic_yaw_deg:.1f} "
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
            f"max_joint_span={self.max_joint_span_rad:g} "
            f"plan_only={self.plan_only_capture} "
            f"scene_setup={self.scene_setup_enabled}"
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
        self.moveit2.max_velocity = self.velocity_scaling
        self.moveit2.max_acceleration = self.acceleration_scaling
        self.get_logger().info(
            f"Velocity scaling: {self.velocity_scaling:.2f}  "
            f"Acceleration scaling: {self.acceleration_scaling:.2f}"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self._planning_scene_client = self.create_client(
            ApplyPlanningScene, self.apply_planning_scene_service
        )

        # Run a background executor so action/service callbacks are processed.
        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()
        try:
            if self.scene_setup_enabled:
                self._apply_configured_planning_scene()
            if not self.plan_only_capture:
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
            Marker, "/board_writer/trajectory_marker", marker_qos
        )

        # Create service to run plan on demand
        self.run_plan_service = self.create_service(
            Trigger,
            "/board_writer/run_plan",
            self._run_plan_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info("="*60)
        self.get_logger().info("WRITER READY!")
        self.get_logger().info("Service: /board_writer/run_plan")
        self.get_logger().info("Ready to receive replay commands!")
        self.get_logger().info("="*60)

        # Path dimensions.
        # Semantic frame for path points:
        # - forward: distance in front of robot (mapped to world -y)
        # - lateral: left/right across the writing surface (mapped to world x)
        # - z: up/down (mapped to world z)
        self.forward_draw = inch(16.1)
        self.forward_ret = self.pen_retract_forward_m
        # Now that forward_draw is known, build the board frame used for waypoint projection.
        self._setup_board_frame()

        self.height = inch(5.0)
        # Board bottom = 3.75 in above ground (matches scene_board_center_z - size_z/2)
        # Calibrated from physical observation: 1" margin drew at the board edge,
        # confirming z_bot was 1" too low. Corrected from inch(2.75) → inch(3.75).
        self.z_bot = inch(3.75)
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

    def _rotate_semantic_xy(self, x: float, y: float):
        """Legacy 2-D yaw rotation — still used by relative-mode fallback path."""
        return (
            self._semantic_yaw_c * x - self._semantic_yaw_s * y,
            self._semantic_yaw_s * x + self._semantic_yaw_c * y,
        )

    def _setup_board_frame(self) -> None:
        """Compute the tilted board's 3-D coordinate frame from the four corner
        positions and the two tilt angles, then cache the results so that
        _world_from_semantic can project any (forward, lateral, z) waypoint
        onto the actual physical writing surface.

        Board-local axes (before tilt):
          width_hat  — from BL to BR (world Y on the default flat board)
          height_hat — from BL to TL (world Z)
          normal_hat — width × height (world +X, pointing away from the robot)

        Tilt rotations (applied in order):
          tilt_vertical_deg   — rotation about width_hat  (tips top toward/away from robot)
          tilt_horizontal_deg — rotation about height_hat (rotates board left/right)

        After tilting, we cache:
          _board_centre      — mean of the four corners (physically unchanged; the
                               collision box rotates around this point)
          _board_face_centre — surface the pen tip actually touches:
                               centre − half_thickness × normal_hat
          _board_width_hat   — tilted board horizontal axis
          _board_height_hat  — tilted board vertical axis
          _board_normal_hat  — tilted outward normal (toward robot when negative)
        """
        import numpy as _np

        bl = _np.array(self.scene_board_corners["bl"], dtype=float)
        br = _np.array(self.scene_board_corners["br"], dtype=float)
        tr = _np.array(self.scene_board_corners["tr"], dtype=float)
        tl = _np.array(self.scene_board_corners["tl"], dtype=float)

        centre = (bl + br + tr + tl) / 4.0

        # Derive orthonormal board-local axes from the corners.
        width_vec  = br - bl
        width_m    = float(_np.linalg.norm(width_vec))
        width_hat  = width_vec / width_m if width_m > 1e-9 else _np.array([0.0, 1.0, 0.0])

        height_vec = tl - bl
        height_m   = float(_np.linalg.norm(height_vec))
        height_hat = height_vec / height_m if height_m > 1e-9 else _np.array([0.0, 0.0, 1.0])

        normal_hat = _np.cross(width_hat, height_hat)
        n_norm     = float(_np.linalg.norm(normal_hat))
        normal_hat = normal_hat / n_norm if n_norm > 1e-9 else _np.array([1.0, 0.0, 0.0])
        # Re-orthogonalise height so [normal, width, height] is a proper frame.
        height_hat = _np.cross(normal_hat, width_hat)
        h_norm     = float(_np.linalg.norm(height_hat))
        if h_norm > 1e-9:
            height_hat = height_hat / h_norm

        def _rot_mat(axis: "_np.ndarray", deg: float) -> "_np.ndarray":
            """Rodrigues rotation matrix: rotate *deg* degrees about *axis*."""
            rad = math.radians(deg)
            c, s = math.cos(rad), math.sin(rad)
            ax, ay, az = axis / _np.linalg.norm(axis)
            return _np.array([
                [c + ax*ax*(1-c),       ax*ay*(1-c) - az*s,  ax*az*(1-c) + ay*s],
                [ay*ax*(1-c) + az*s,    c + ay*ay*(1-c),      ay*az*(1-c) - ax*s],
                [az*ax*(1-c) - ay*s,    az*ay*(1-c) + ax*s,   c + az*az*(1-c)   ],
            ])

        # Cache the pre-tilt normal as the depth/retract direction before tilts are
        # applied.  For a board whose untilted face normal = [1,0,0] (standard setup
        # with corners all at the same X), this yields approach_hat = [1,0,0].
        # Using this for the depth term in _world_from_semantic makes pen-retract moves
        # go purely backward in world X — matching the 'online' (flat-board) behavior
        # that was proven drag-free.  Using the tilted normal instead adds a Z component
        # (-7.5 mm per 40 mm retract) which causes the spring tip to slide along the
        # board surface as the tilt breaks the alignment between the spring axis and the
        # board perpendicular.
        approach_hat = normal_hat.copy()

        # Apply tilt_vertical_deg first (rotation about the board's width axis).
        if abs(self.scene_board_tilt_vertical_deg) > 1e-6:
            R = _rot_mat(width_hat, self.scene_board_tilt_vertical_deg)
            width_hat  = R @ width_hat
            height_hat = R @ height_hat
            normal_hat = R @ normal_hat

        # Then apply tilt_horizontal_deg (rotation about the board's height axis).
        if abs(self.scene_board_tilt_horizontal_deg) > 1e-6:
            R = _rot_mat(height_hat, self.scene_board_tilt_horizontal_deg)
            width_hat  = R @ width_hat
            height_hat = R @ height_hat
            normal_hat = R @ normal_hat

        # Store the tilted frame.
        self._board_centre       = centre
        self._board_width_hat    = width_hat
        self._board_height_hat   = height_hat
        self._board_normal_hat   = normal_hat
        self._board_approach_hat = approach_hat  # pre-tilt normal: retract/extend direction

        # Board face = front writing surface (toward the robot).
        # The collision box is centred at *centre*; the pen tip touches the face:
        #   face = centre − half_thickness × normal_hat
        half_t = self.scene_board_thickness / 2.0
        self._board_face_centre = centre - half_t * normal_hat

        self.get_logger().info(
            f"[board_frame] centre=({centre[0]:.4f},{centre[1]:.4f},{centre[2]:.4f})  "
            f"face_centre=({self._board_face_centre[0]:.4f},{self._board_face_centre[1]:.4f},{self._board_face_centre[2]:.4f})  "
            f"normal_hat=({normal_hat[0]:.4f},{normal_hat[1]:.4f},{normal_hat[2]:.4f})  "
            f"approach_hat=({approach_hat[0]:.4f},{approach_hat[1]:.4f},{approach_hat[2]:.4f})  "
            f"tilt_v={self.scene_board_tilt_vertical_deg:.2f}°  "
            f"tilt_h={self.scene_board_tilt_horizontal_deg:.2f}°"
        )

    def _world_from_semantic(
        self,
        forward: float,
        lateral: float,
        z: float,
        anchor_semantic=None,
        anchor_world=None,
    ):
        """Map a semantic (forward, lateral, z) waypoint to world-frame XYZ.

        Absolute mode (anchor_semantic is None)
        ----------------------------------------
        Projects the 2-D board coordinate (lateral, z) onto the tilted board
        plane, then offsets by (forward − forward_draw) along the tilted board
        normal (_board_normal_hat) to handle pen retract / extend moves.  When
        (lateral, z) are held fixed the retract is perpendicular to the board
        surface, giving a clean pen lift with no sliding.

        Relative mode (anchor_semantic / anchor_world provided)
        ---------------------------------------------------------
        Unchanged from the original implementation: offsets are computed in
        the legacy semantic-yaw frame and added to the current EE pose. This
        path is still used when relative_to_current_pose=true.
        """
        # ── Relative mode: unchanged ─────────────────────────────────────────
        if anchor_semantic is not None and anchor_world is not None:
            anchor_f, anchor_l, anchor_z = anchor_semantic
            ax, ay, az = anchor_world
            s = self.relative_scale
            dx_sem = -(lateral - anchor_l)
            dy_sem = -(forward - anchor_f)
            dx_rot, dy_rot = self._rotate_semantic_xy(dx_sem, dy_sem)
            x = ax + s * dx_rot + self.offset_x
            y = ay + s * dy_rot + self.offset_y
            return x, y, az + s * (z - anchor_z) + self.offset_z

        # ── Absolute mode: tilted-board projection ───────────────────────────
        #
        # Board-space coordinates (all in metres from the board centre):
        #   u     = position along the board width axis
        #           (-lateral because positive lateral = robot's right, which
        #            is the negative width direction on the default board)
        #   v     = position along the board height axis
        #           (z is delivered as an absolute-world-Z value calibrated
        #            for the flat board; subtracting _board_centre[2] recovers
        #            the board-local height offset from centre)
        #   depth = (forward − forward_draw): 0 when the pen is on the writing
        #           surface, negative when retracted toward the robot
        #
        # The 3-D world position is then:
        #   P = _board_face_centre
        #       + u     × _board_width_hat
        #       + v     × _board_height_hat
        #       + depth × _board_approach_hat
        #
        # IMPORTANT: depth uses _board_approach_hat (pre-tilt normal, ≈ world +X)
        # NOT _board_normal_hat (tilted normal).  The tilted normal has a Z component
        # which causes the spring tip to slide along the board surface during retract.
        # The pre-tilt direction is the actual arm approach direction and gives a
        # drag-free lift, matching the 'online' flat-board behavior.
        #
        # For a flat, untilted board _board_approach_hat == _board_normal_hat so
        # this reduces exactly to the 'online' formula.
        u     = -lateral
        v     = z - float(self._board_centre[2])
        depth = forward - self.forward_draw

        pos = (
            self._board_face_centre
            + u     * self._board_width_hat
            + v     * self._board_height_hat
            + depth * self._board_approach_hat
        )
        return (
            float(pos[0]) + self.offset_x,
            float(pos[1]) + self.offset_y,
            float(pos[2]) + self.offset_z,
        )

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

    @staticmethod
    def _build_box_from_corners(
        object_id: str,
        frame_id: str,
        corners: dict,
        thickness: float,
        tilt_vertical_deg: float,
        tilt_horizontal_deg: float,
    ) -> CollisionObject:
        """Build a CollisionObject box from four named corners (bl, br, tr, tl).

        Steps:
          1. Derive centre and orthonormal board-local axes from the corners.
          2. Apply tilt_vertical_deg  (rotation about the board width  axis).
          3. Apply tilt_horizontal_deg (rotation about the board height axis).
          4. Convert to a Pose with a box whose dims are
             [thickness, width_m, height_m].
        """
        import numpy as _np

        bl = _np.array(corners["bl"], dtype=float)
        br = _np.array(corners["br"], dtype=float)
        tr = _np.array(corners["tr"], dtype=float)
        tl = _np.array(corners["tl"], dtype=float)

        centre = (bl + br + tr + tl) / 4.0

        width_vec = br - bl
        width_m   = float(_np.linalg.norm(width_vec))
        width_hat = width_vec / width_m if width_m > 1e-9 else _np.array([0.0, 1.0, 0.0])

        height_vec = tl - bl
        height_m   = float(_np.linalg.norm(height_vec))
        height_hat = height_vec / height_m if height_m > 1e-9 else _np.array([0.0, 0.0, 1.0])

        # Normal = width × height, then re-orthogonalise height
        normal_hat = _np.cross(width_hat, height_hat)
        n_norm = float(_np.linalg.norm(normal_hat))
        normal_hat = normal_hat / n_norm if n_norm > 1e-9 else _np.array([1.0, 0.0, 0.0])
        height_hat = _np.cross(normal_hat, width_hat)
        h_norm = float(_np.linalg.norm(height_hat))
        if h_norm > 1e-9:
            height_hat = height_hat / h_norm

        # Rotation matrix: columns are [normal, width, height]
        R = _np.column_stack([normal_hat, width_hat, height_hat])

        def _rot(axis, deg):
            rad = math.radians(deg)
            c, s = math.cos(rad), math.sin(rad)
            ax, ay, az = axis / _np.linalg.norm(axis)
            return _np.array([
                [c + ax*ax*(1-c),     ax*ay*(1-c) - az*s, ax*az*(1-c) + ay*s],
                [ay*ax*(1-c) + az*s,  c + ay*ay*(1-c),     ay*az*(1-c) - ax*s],
                [az*ax*(1-c) - ay*s,  az*ay*(1-c) + ax*s,  c + az*az*(1-c)  ],
            ])

        if abs(tilt_vertical_deg) > 1e-6:
            R = _rot(width_hat, tilt_vertical_deg) @ R
        if abs(tilt_horizontal_deg) > 1e-6:
            R = _rot(height_hat, tilt_horizontal_deg) @ R

        # Rotation matrix → quaternion (Shepperd's method)
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s2 = math.sqrt(trace + 1.0) * 2
            qw = 0.25 * s2
            qx = (R[2, 1] - R[1, 2]) / s2
            qy = (R[0, 2] - R[2, 0]) / s2
            qz = (R[1, 0] - R[0, 1]) / s2
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s2 = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s2
            qx = 0.25 * s2
            qy = (R[0, 1] + R[1, 0]) / s2
            qz = (R[0, 2] + R[2, 0]) / s2
        elif R[1, 1] > R[2, 2]:
            s2 = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s2
            qx = (R[0, 1] + R[1, 0]) / s2
            qy = 0.25 * s2
            qz = (R[1, 2] + R[2, 1]) / s2
        else:
            s2 = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s2
            qx = (R[0, 2] + R[2, 0]) / s2
            qy = (R[1, 2] + R[2, 1]) / s2
            qz = 0.25 * s2

        box = CollisionObject()
        box.header.frame_id = frame_id
        box.id = object_id
        box.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        # X = thickness (normal axis), Y = width, Z = height
        primitive.dimensions = [float(thickness), float(width_m), float(height_m)]

        pose = Pose()
        pose.position.x = float(centre[0])
        pose.position.y = float(centre[1])
        pose.position.z = float(centre[2])
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        box.primitives.append(primitive)
        box.primitive_poses.append(pose)
        return box

    @staticmethod
    def _build_box(
        object_id: str,
        frame_id: str,
        center_xyz,
        size_xyz,
    ) -> CollisionObject:
        if any(v <= 0.0 for v in size_xyz):
            raise ValueError(
                f"Collision box '{object_id}' has non-positive dimensions: {size_xyz}"
            )

        box = CollisionObject()
        box.header.frame_id = frame_id
        box.id = object_id
        box.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size_xyz[0]), float(size_xyz[1]), float(size_xyz[2])]

        pose = Pose()
        pose.position.x = float(center_xyz[0])
        pose.position.y = float(center_xyz[1])
        pose.position.z = float(center_xyz[2])
        pose.orientation.w = 1.0

        box.primitives.append(primitive)
        box.primitive_poses.append(pose)
        return box

    def _apply_configured_planning_scene(self) -> None:
        frame_id = self.scene_frame_id if self.scene_frame_id else self.base_link_name
        objects = []
        if self.scene_board_enabled:
            objects.append(
                self._build_box_from_corners(
                    object_id=self.scene_board_id,
                    frame_id=frame_id,
                    corners=self.scene_board_corners,
                    thickness=self.scene_board_thickness,
                    tilt_vertical_deg=self.scene_board_tilt_vertical_deg,
                    tilt_horizontal_deg=self.scene_board_tilt_horizontal_deg,
                )
            )
            self.get_logger().info(
                f"Board collision object '{self.scene_board_id}' built from corners "
                f"BL={self.scene_board_corners['bl']} BR={self.scene_board_corners['br']} "
                f"TR={self.scene_board_corners['tr']} TL={self.scene_board_corners['tl']} "
                f"tilt_v={self.scene_board_tilt_vertical_deg:.1f}° "
                f"tilt_h={self.scene_board_tilt_horizontal_deg:.1f}°"
            )
        if self.scene_table_enabled:
            objects.append(
                self._build_box(
                    object_id=self.scene_table_id,
                    frame_id=frame_id,
                    center_xyz=self.scene_table_center,
                    size_xyz=self.scene_table_size,
                )
            )

        if not objects:
            self.get_logger().warn(
                "scene_setup_enabled=true but no collision objects are enabled."
            )
            return

        if not self._planning_scene_client.wait_for_service(
            timeout_sec=self.scene_setup_timeout_sec
        ):
            raise RuntimeError(
                "Planning scene service not available on "
                f"'{self.apply_planning_scene_service}' within "
                f"{self.scene_setup_timeout_sec:.1f}s."
            )

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects = objects

        fut = self._planning_scene_client.call_async(req)
        deadline = time.monotonic() + self.scene_setup_timeout_sec
        while time.monotonic() < deadline:
            if fut.done():
                break
            time.sleep(0.05)

        if not fut.done():
            raise RuntimeError(
                "Timed out while waiting for planning scene application response."
            )

        if fut.exception() is not None:
            raise RuntimeError(
                f"Planning scene service call failed: {type(fut.exception()).__name__}: "
                f"{fut.exception()}"
            )

        resp = fut.result()
        if resp is None or not resp.success:
            raise RuntimeError("MoveIt rejected planning scene collision object update.")

        self.get_logger().info(
            "Applied planning scene collision objects: "
            + ", ".join(obj.id for obj in objects)
        )

    def _plan_pose(self, x: float, y: float, z: float, cartesian: bool):
        # With orientation disabled, use a position-only request.
        # Some cartesian plan calls raise instead of returning None.
        try:
            start_state = self.moveit2.joint_state
            if self.weight_orientation <= 0.0:
                traj = self.moveit2.plan(
                    position=[x, y, z],
                    tolerance_position=self.tolerance_position,
                    weight_position=self.weight_position,
                    start_joint_state=start_state,
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
                    start_joint_state=start_state,
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
                if self.max_joint_span_rad > 0.0 and max_span > self.max_joint_span_rad:
                    raise RuntimeError(
                        "Planning returned excessive joint motion "
                        f"(max_span={max_span:.3f} rad > {self.max_joint_span_rad:.3f}); "
                        "rejecting to avoid branch-flip style detours."
                    )
        if planned_cartesian != self.use_cartesian:
            self.get_logger().info(
                f"segment planned with cartesian={planned_cartesian}"
            )
        self._append_planned_samples(traj)
        if self.plan_only_capture:
            return
        self.moveit2.execute(traj)
        if not self.moveit2.wait_until_executed():
            exec_err = self.moveit2.get_last_execution_error_code()
            err_suffix = ""
            if exec_err is not None and hasattr(exec_err, "val"):
                err_suffix = f" (moveit_error_code={exec_err.val})"
            raise RuntimeError(
                "Trajectory execution failed: execute_trajectory returned non-success"
                f"{err_suffix}."
            )
   

    def _append_planned_samples(self, traj) -> None:
        names = list(traj.joint_names)
        index = self._expected_joint_indices(names)
        if index is None:
            return

        if not traj.points:
            return

        # --- SMART SAMPLING CONFIGURATION ---
        # Target: one keyframe per ~80 ms of servo travel (KEYFRAME_INTERVAL_MS floor).
        # AX-12A @ speed=10  ->  0.0093 rad / 80 ms
        # MX-64  @ speed=20  ->  0.0191 rad / 80 ms
        # Setting threshold to half the AX-12A 80 ms travel gives ~2-3x more
        # keyframes and keeps each step well within what the slowest servo can
        # cover in one KEYFRAME_INTERVAL_MS window.
        # Smaller number = More steps (Smoother, but slower to generate)
        # Larger number = Fewer steps (Blockier/jumpy)
        MIN_CHANGE_RAD = 0.004
        # ------------------------------------

        last_saved_vals = None
        last_saved_time_ns = 0

        for i, pt in enumerate(traj.points):
            vals = [float(pt.positions[j]) for j in index]
            t_ns = pt.time_from_start.sec * 1_000_000_000 + pt.time_from_start.nanosec

            # Always save the very first point of this segment
            if last_saved_vals is None:
                # First segment: no delay on first point; subsequent segments:
                # small gap so the Arduino doesn't merge with the previous segment.
                dt_ms = 0 if not self._captured_samples else 60
                self._captured_samples.append((dt_ms, vals, False))
                last_saved_vals = vals
                last_saved_time_ns = t_ns
                continue

            # Check: Did any joint move enough to justify a new command?
            max_diff = max(abs(vals[k] - last_saved_vals[k]) for k in range(len(vals)))

            # Save IF: Moved enough OR it is the final destination (Corner)
            if max_diff > MIN_CHANGE_RAD or i == len(traj.points) - 1:
                # Accumulate elapsed time from the last *saved* point (includes
                # any skipped intermediate waypoints).
                dt_ns = t_ns - last_saved_time_ns
                if dt_ns > 0:
                    dt_ms = int(max(20, min(2000, round(dt_ns / 1_000_000))))
                else:
                    dt_ms = 20
                self._captured_samples.append((dt_ms, vals, False))
                last_saved_vals = vals
                last_saved_time_ns = t_ns

    def old_append_planned_samples(self, traj) -> None:
        names = list(traj.joint_names)
        index = self._expected_joint_indices(names)
        if index is None:
            return

        if not traj.points:
            return

        # Trajectory segments restart time_from_start at zero. Drop the repeated
        # first point for subsequent segments and keep MoveIt's segment timing.
        start_idx = 0 if not self._captured_samples else 1
        if start_idx >= len(traj.points):
            return

        for i in range(start_idx, len(traj.points)):
            pt = traj.points[i]
            vals = [float(pt.positions[j]) for j in index]
            if i == 0:
                dt_ms = 120 if not self._captured_samples else 60
            else:
                t0 = traj.points[i - 1].time_from_start
                t1 = pt.time_from_start
                dt_ns = (t1.sec - t0.sec) * 1_000_000_000 + (t1.nanosec - t0.nanosec)
                if dt_ns <= 0:
                    raise RuntimeError(
                        "Planner returned non-increasing time_from_start values."
                    )
                dt_ms = int(
                    max(
                        20,
                        min(
                            2000,
                            round(dt_ns / 1e6),
                        ),
                    )
                )
            self._captured_samples.append((dt_ms, vals, False))

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
            f.write("  uint8_t pause;  // 0=stream; 1=pen-lift/plant boundary (full stop); "
                    "2=draw-to-draw corner (stop, no dwell); 3=user pause point (full stop + PAUSE_POINT_MS dwell)\n")
            f.write("} JointSample;\n\n")
            f.write("static const JointSample kTrajectory[] = {\n")
            for dt_ms, vals, pause in self._captured_samples:
                f.write(
                    "  { %d, %.8ff, %.8ff, %.8ff, %.8ff, %.8ff, %d },\n"
                    % (dt_ms, vals[0], vals[1], vals[2], vals[3], vals[4], int(pause))
                )
            f.write("};\n\n")
            f.write(
                "static const size_t kTrajectoryCount = "
                "sizeof(kTrajectory) / sizeof(kTrajectory[0]);\n"
            )
        self.get_logger().info(
            f"Wrote {len(self._captured_samples)} joint samples to {out}"
        )

    def _execute_segment_with_backoff(self, prev_xyz, target_xyz, mode: str, prev_mode: str = None):
        is_lifting = mode == "travel" and prev_mode == "draw"
        if prev_xyz is not None and is_lifting:
            is_lifting = target_xyz[2] > prev_xyz[2]
        if is_lifting:
            self.get_logger().info(
                f"Pausing {self.pen_lift_pause_sec:.2f}s before pen lift"
            )
            time.sleep(self.pen_lift_pause_sec)

        # Try full segment first, then progressively smaller fractions to keep motion feasible.
        scales = (1.0, 0.5, 0.25, 0.1, 0.05, 0.02)
        if prev_xyz is None:
            px, py, pz = target_xyz
        else:
            px, py, pz = prev_xyz
        tx, ty, tz = target_xyz
        last_err = None
        samples_before = len(self._captured_samples)

        # Store original cartesian_max_step for lifting moves
        original_max_step = self.cartesian_max_step
        if is_lifting:
            self.cartesian_max_step = original_max_step * self.pen_lift_speed_scale
            self.get_logger().info(
                f"Reducing cartesian step for pen lift: {original_max_step:.4f} → {self.cartesian_max_step:.4f}"
            )

        try:
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
        finally:
            # Restore original cartesian_max_step after lifting
            if is_lifting:
                self.cartesian_max_step = original_max_step

    def _build_semantic_plan(self):
        return [
            # A - 5 inches tall, centered on lateral=0

            # Go directly to retracted start position (diagonal from wherever arm parks)
            ("travel", (self.forward_ret, -0.05, self.z_bot)),
            # Extend to touch the board at the start point
            ("draw",   (self.forward_draw, -0.05, self.z_bot)),
            # Draw left diagonal up to peak
            ("draw",   (self.forward_draw, 0.0, self.z_top)),
            # Draw right diagonal down to bottom-right
            ("draw",   (self.forward_draw, 0.05, self.z_bot)),
            # Retract cleanly at stroke end height
            ("travel", (self.forward_ret, 0.05, self.z_bot)),
            # Go directly (diagonally) to crossbar start, retracted
            ("travel", (self.forward_ret, -0.03, self.z_mid)),
            # Extend to touch board at crossbar start
            ("draw",   (self.forward_draw, -0.03, self.z_mid)),
            # Draw crossbar
            ("draw",   (self.forward_draw, 0.03, self.z_mid)),
            # Retract cleanly
            ("travel", (self.forward_ret, 0.03, self.z_mid)),
            # Park high
            ("travel", (self.forward_ret, 0.03, self.z_top)),
        ]

    def _resolve_world_targets(self, plan, anchor_semantic=None, anchor_world=None):
        resolved = []
        for entry in plan:
            # Support both 2-tuple (mode, xyz) from _build_semantic_plan and
            # 3-tuple (mode, xyz, user_pause) from _build_plan_from_file.
            if len(entry) == 3:
                mode, (forward, lateral, z), user_pause = entry
            else:
                mode, (forward, lateral, z) = entry
                user_pause = False
            x, y, z2 = self._world_from_semantic(
                forward,
                lateral,
                z,
                anchor_semantic=anchor_semantic,
                anchor_world=anchor_world,
            )
            x, y, z2, clipped = self._apply_workspace_bounds(x, y, z2)
            resolved.append((mode, forward, lateral, x, y, z2, clipped, user_pause))
        return resolved

    def _load_plan(self):
        """Load plan from letterCoordinates.json if it exists, else use hardcoded.

        Also checks the sibling online/ workspace copy and uses whichever file
        was modified most recently, so the node works regardless of which GUI
        instance the user ran.
        """
        configured = Path(self.letter_coordinates_file) if self.letter_coordinates_file else None

        # Build a list of candidate paths: configured path + known sibling copy.
        candidates = []
        if configured:
            candidates.append(configured)
        sibling = Path("/home/rosubu/ME_557_Robot/online/letterCoordinates.json")
        if sibling not in candidates and sibling.exists():
            candidates.append(sibling)

        # Pick the candidate that exists and has the most recent mtime.
        best: Path | None = None
        for p in candidates:
            if p.exists():
                if best is None or p.stat().st_mtime > best.stat().st_mtime:
                    best = p

        if best is not None:
            try:
                plan = self._build_plan_from_file(str(best))
                self.get_logger().info(
                    f"Loaded {len(plan)} steps from {best}"
                )
                return plan
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to load coordinates file ({exc}); using hardcoded plan."
                )
        return self._build_semantic_plan()

    def _build_plan_from_file(self, file_path: str):
        """Convert letterCoordinates.json segments into a semantic plan.

        Each segment may have:
          "lift_after": true  → lift pen after stroke and travel to next start (default)
          "lift_after": false → pen stays on board; draw directly to next stroke start
          "pause_point_start": true → insert pause=3 when pen arrives at stroke start
          "pause_point_end":   true → insert pause=3 when pen arrives at stroke end

        Plan entries are 3-tuples: (mode, xyz_tuple, user_pause_bool)
        """
        with open(file_path) as f:
            data = json.load(f)
        segments = data["segments"]
        if not segments:
            raise ValueError("No segments found in coordinates file.")

        plan = []
        pen_is_down = False   # track whether pen is currently on the board

        for i, seg in enumerate(segments):
            lat_start_m = seg["start"][0] * 0.0254   # inches → metres
            z_start     = self.z_bot + inch(seg["start"][1])  # relative → absolute
            lat_end_m   = seg["end"][0]   * 0.0254
            z_end       = self.z_bot + inch(seg["end"][1])
            lift_after  = seg.get("lift_after", True)   # default: lift
            pause_start = bool(seg.get("pause_point_start", False))
            pause_end   = bool(seg.get("pause_point_end",   False))

            if not pen_is_down:
                # Pen is in the air – go directly (diagonally) to start position while retracted
                plan.append(("travel", (self.forward_ret,  lat_start_m, z_start), False))
                # Extend forward to touch the board at the exact start point
                plan.append(("draw",   (self.forward_draw, lat_start_m, z_start), pause_start))
            else:
                # Pen is already on the board – draw across to new stroke start
                plan.append(("draw", (self.forward_draw, lat_start_m, z_start), pause_start))

            # Draw the stroke itself
            plan.append(("draw", (self.forward_draw, lat_end_m, z_end), pause_end))

            if lift_after:
                # Retract from the board.  Hold (lateral, z) FIXED in semantic
                # space — only forward changes — so the EE moves perpendicular
                # to the tilted board surface.  This gives a clean pen lift with
                # no component along the board surface to drag the tip.
                plan.append(("travel", (self.forward_ret, lat_end_m, z_end), False))
                pen_is_down = False
                # If this is the last segment, park high
                if i == len(segments) - 1:
                    plan.append(("travel", (self.forward_ret, lat_end_m, self.z_top), False))
            else:
                # Pen stays on the board for the next segment
                pen_is_down = True

        # Safety: if the last segment left the pen down, retract then rise
        if pen_is_down and segments:
            last = segments[-1]
            lat_end_m = last["end"][0] * 0.0254
            z_last_end = self.z_bot + inch(last["end"][1])
            plan.append(("travel", (self.forward_ret, lat_end_m, z_last_end), False))
            plan.append(("travel", (self.forward_ret, lat_end_m, self.z_top),  False))

        return plan

    def _run_plan_callback(self, request, response):
        """Service callback to run the plan on demand."""
        try:
            self.get_logger().info("Running plan from service call...")
            self.run_plan()
            response.success = True
            response.message = "Plan executed successfully"
        except Exception as e:
            self.get_logger().error(f"Plan execution failed: {e}")
            response.success = False
            response.message = f"Plan execution failed: {str(e)}"
        return response

    def run_plan(self) -> None:
        # Reset captured samples so each trigger produces a fresh .h file
        self._captured_samples = []
        plan = self._load_plan()

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
                    "Using relative origin at current end-effector pose: "
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
        prev_mode = "travel"  # treat start-of-plan as if coming from a travel so the first draw triggers a touch-down pause
        for mode, forward, lateral, x, y, z, clipped, user_pause in world_targets:
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

            # Corner settle: two consecutive draw segments meet at a corner
            # (e.g. the apex of an 'A'). The arm must fully stop there before
            # changing direction; without a wait it blends/cuts the corner.
            # pause=2 → waitForMotionComplete, no retract dwell.
            # Do NOT overwrite a user-defined pause=3.
            if mode == "draw" and prev_mode == "draw" and self._captured_samples:
                dt_ms, vals, _p = self._captured_samples[-1]
                if _p != 3:
                    self._captured_samples[-1] = (dt_ms, vals, 2)

            # Pause before retract: we are about to execute a travel that follows
            # a draw. The arm is still at the pen-down position (end of stroke).
            # Mark the last already-captured sample — the final keyframe of that
            # draw — so the Arduino pauses after the arm settles there.
            # pause=1 → waitForMotionComplete + CONTACT_PAUSE_MS dwell.
            # Do NOT overwrite a user-defined pause=3.
            if mode == "travel" and prev_mode == "draw" and self._captured_samples:
                dt_ms, vals, _p = self._captured_samples[-1]
                if _p != 3:
                    self._captured_samples[-1] = (dt_ms, vals, 1)

            # Pause between retract-in-place and lateral travel: the previous
            # travel pulled the pen back from the board surface. The next travel
            # will sweep laterally. Slow servos (TRAJECTORY_SPEED=10) cannot
            # complete a retract in the time allotted by MoveIt's dt_ms alone.
            # Forcing a full stop here guarantees the arm is truly retracted
            # before lateral movement begins — preventing pen drag.
            # pause=1 → waitForMotionComplete + CONTACT_PAUSE_MS dwell.
            if mode == "travel" and prev_mode == "travel" and self._captured_samples:
                dt_ms, vals, _p = self._captured_samples[-1]
                if _p != 3:
                    self._captured_samples[-1] = (dt_ms, vals, 1)

            # Pause at hover before plunge: we are about to execute a draw that
            # follows a travel. The arm should be stationary at the hover position
            # (forward_ret, above segment start) before it extends to the board.
            # Marking the last travel waypoint forces a full stop here, preventing
            # the plunge from starting while the arm is still moving laterally.
            # pause=1 → waitForMotionComplete + CONTACT_PAUSE_MS dwell.
            if mode == "draw" and prev_mode == "travel" and self._captured_samples:
                dt_ms, vals, _p = self._captured_samples[-1]
                if _p != 3:
                    self._captured_samples[-1] = (dt_ms, vals, 1)

            samples_before = len(self._captured_samples)

            prev_xyz, ok = self._execute_segment_with_backoff(
                prev_xyz=prev_xyz, target_xyz=(x, y, z), mode=mode, prev_mode=prev_mode
            )
            prev_mode_before = prev_mode
            prev_mode = mode

            # Pause after touch-down: we just executed a draw that followed a
            # travel (pen extended forward to the board surface). Mark the last
            # keyframe of this draw segment so the Arduino pauses once the arm
            # has fully arrived at the board.
            # pause=1 → waitForMotionComplete + CONTACT_PAUSE_MS dwell.
            if mode == "draw" and prev_mode_before == "travel":
                if len(self._captured_samples) > samples_before:
                    dt_ms, vals, _p = self._captured_samples[-1]
                    self._captured_samples[-1] = (dt_ms, vals, 1)

            if ok:
                executed_segments += 1
            else:
                failed_segments += 1

            # User-defined pause point: overrides all auto-detected flags.
            # pause=3 tells the Arduino to stop and dwell for PAUSE_POINT_MS.
            if user_pause and self._captured_samples:
                dt_ms, vals, _p = self._captured_samples[-1]
                self._captured_samples[-1] = (dt_ms, vals, 3)
                self.get_logger().info(
                    f"user pause=3 tagged at waypoint {len(self._captured_samples) - 1}"
                )

        self.get_logger().info(
            "WRITE_SUMMARY "
            f"attempted={attempted_segments} "
            f"executed={executed_segments} "
            f"failed={failed_segments} "
            f"skipped={skipped_segments} "
            f"bounded={bounded_segments}"
        )
        if executed_segments > 0 and self._captured_samples:
            self._write_header()

    def _publish_trace_marker(self, world_targets) -> None:
        stamp = self.get_clock().now().to_msg()

        def make_marker(mid, r, g, b, thickness):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = stamp
            m.ns = "board_trace"
            m.id = mid
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = thickness
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            return m

        # Blue  – pen on board (draw)
        draw_marker   = make_marker(0, 0.1, 0.6, 1.0, 0.004)
        # Yellow – free-air lateral/z repositioning (travel at forward_ret)
        travel_marker = make_marker(1, 1.0, 0.8, 0.1, 0.003)
        # Purple – retract/extend (forward dimension changing)
        retract_marker = make_marker(2, 0.7, 0.1, 1.0, 0.003)

        prev_pt = None
        prev_forward = None
        for mode, forward, lateral, x, y, z, _, _up in world_targets:
            curr_pt = Point(x=x, y=y, z=z)
            if prev_pt is not None:
                forward_changed = (prev_forward is not None and
                                   abs(forward - prev_forward) > 0.001)
                if forward_changed:
                    # Retract (draw→ret) or extend (ret→draw)
                    retract_marker.points.append(prev_pt)
                    retract_marker.points.append(curr_pt)
                elif mode == "draw":
                    draw_marker.points.append(prev_pt)
                    draw_marker.points.append(curr_pt)
                else:
                    travel_marker.points.append(prev_pt)
                    travel_marker.points.append(curr_pt)
            prev_pt = curr_pt
            prev_forward = forward

        self.marker_pub.publish(draw_marker)
        self.marker_pub.publish(travel_marker)
        self.marker_pub.publish(retract_marker)


def main() -> None:
    rclpy.init()
    node = AceWriter()
    try:
        node.get_logger().info("BoardWriter node ready. Waiting for /board_writer/run_plan service calls.")
        # Keep the node running to handle service requests
        rclpy.spin(node)
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
