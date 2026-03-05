#!/usr/bin/env python3
"""Generate ik_points.h for code 888 from a list of Cartesian corner positions.

Requires a running ROS 2 / MoveIt stack with /compute_ik service available.
Usually started with:
    ros2 launch me557_pen_moveit_config move_group.launch.py

---- HOW TO USE ---------------------------------------------------------------
1.  In the LETTER CORNER DEFINITIONS section below, fill in the corners of each
    letter as (x_m, y_m, z_m, pen) tuples.
    - x/y/z are in METRES in the MoveIt planning frame (base_link).
    - pen: 0 = PEN UP  (travel to this point without touching the board)
           1 = PEN DOWN (arm draws to this point)

2.  Tune ORIENTATION_CANDIDATES if IK fails for your specific end-effector pose.
    The script tries each candidate in order and uses the first that solves.

3.  Run (with ROS 2 sourced):
        python3 onlineWeights/tools/generate_ik_points.py

4.  The output file onlineWeights/robotMovements/ik_points.h is written
    (overwriting the placeholder).
-------------------------------------------------------------------------------

Conversion used (matches moveitRadToLogical on the Arduino):
    logical_deg = 180.0 + DIR_SIGN[i] * (rad - HOME_RAD[i]) * (180/π)

    DIR_SIGN  = [-1,  1,  1, -1,  1]   for joints [1, 2, 4, 5, 6]
    HOME_RAD  = [ 0,  0,  0,  0,  0]
"""

from __future__ import annotations

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

# =============================================================================
# CONFIGURATION — edit these to match your URDF / MoveIt config
# =============================================================================
PLANNING_GROUP = "me557_pen_arm"           # MoveIt planning group name
BASE_LINK      = "base_link"
TIP_LINK       = "Link_EE"                 # end-effector link used for IK

OUTPUT_PATH = Path(__file__).resolve().parents[1] / "robotMovements" / "ik_points.h"

# MoveIt joint names in order [J1, J2, J4, J5, J6]
EXPECTED_JOINTS = [
    "Motor1_joint",
    "Motor2_L",
    "Motor4_elb",
    "Motor5_wr",
    "Joint_EE",
]

# Arduino calibration constants (must match CONFIGURATION block in robotMovements.ino)
DIR_SIGN = [-1.0, 1.0, 1.0, -1.0, 1.0]   # MOVEIT_DIR_SIGN
HOME_RAD = [0.0,  0.0, 0.0,  0.0, 0.0]   # MOVEIT_HOME_RAD

# Orientation candidates tried in order when solving IK.
# The first one that succeeds is used.  Add more if your arm needs a specific
# end-effector orientation (e.g. tilted pen holder).
ORIENTATION_CANDIDATES = [
    (0.0, 0.0, 0.0, 1.0),                    # identity
    (0.0, 0.0,  0.70710678, 0.70710678),      # 90° yaw
    (0.0, 0.0, -0.70710678, 0.70710678),      # -90° yaw
    (0.70710678, 0.0, 0.0,  0.70710678),      # 90° roll
    (0.0, 0.70710678, 0.0,  0.70710678),      # 90° pitch
]

# =============================================================================
# LETTER CORNER DEFINITIONS
# =============================================================================
# Each row: (x_m, y_m, z_m, pen)
#   pen 0 = travel (pen UP)
#   pen 1 = draw   (pen DOWN — arm draws to this corner)
#
# Typical stroke pattern for a rectangular stroke:
#   (x0, y0, z0, 0)   ← move above stroke start, pen up
#   (x0, y0, z0, 1)   ← pen lowers here (first drawn point)
#   (x1, y1, z1, 1)   ← draw to second corner
#   ...
#   (xN, yN, zN, 0)   ← lift pen before next stroke
#
# Replace the example below with your actual letter corners.
# Measure coordinates in RViz (drag the interactive marker to each corner and
# read the "Position" from the MotionPlanning panel) or compute from your
# writing surface geometry.
# =============================================================================
def inch(v: float) -> float:
    """Convert inches to metres (handy if your setup dimensions are in inches)."""
    return v * 0.0254


CORNERS: list[tuple[float, float, float, int]] = [
    # --- Example: blocky letter "L" (two strokes) ----------------------------
    # Vertical stroke: top → bottom
    (0.35,  0.05, 0.25, 0),   # travel above top of L
    (0.35,  0.05, 0.25, 1),   # pen down at top
    (0.35,  0.05, 0.15, 1),   # draw to bottom-left of L

    # Horizontal stroke: bottom-left → bottom-right
    (0.35,  0.05, 0.15, 0),   # pen up (already here — just lifts)
    (0.35, -0.05, 0.15, 1),   # pen down — draws right foot of L (END CORNER)
    (0.35, -0.05, 0.15, 0),   # pen up, done
]

# =============================================================================
# (No edits needed below this line)
# =============================================================================

def rad_to_logical(rad: float, joint_idx: int) -> float:
    """Convert MoveIt radian output to Arduino logical degrees."""
    return 180.0 + DIR_SIGN[joint_idx] * (rad - HOME_RAD[joint_idx]) * (180.0 / math.pi)


@dataclass
class IKPoint:
    j1: float; j2: float; j4: float; j5: float; j6: float
    pen: int


class IKPointsGenerator(Node):
    def __init__(self):
        super().__init__("ik_points_generator")
        self._last_js: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 20)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

    def _js_cb(self, msg: JointState) -> None:
        if msg.name and msg.position:
            self._last_js = msg

    def wait_ready(self, timeout_sec: float = 8.0) -> None:
        self.get_logger().info("Waiting for /compute_ik service …")
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("/compute_ik service not available — is MoveIt running?")
        deadline = self.get_clock().now() + RclpyDuration(seconds=timeout_sec)
        while self._last_js is None and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._last_js is None:
            raise RuntimeError("/joint_states not available.")
        self.get_logger().info("Ready.")

    def _seed(self) -> JointState:
        """Use current robot joint state as IK seed."""
        return self._last_js  # type: ignore[return-value]

    def solve(self, xyz, pen: int) -> IKPoint | None:
        for quat in ORIENTATION_CANDIDATES:
            req = GetPositionIK.Request()
            req.ik_request.group_name = PLANNING_GROUP
            req.ik_request.ik_link_name = TIP_LINK
            req.ik_request.pose_stamped = PoseStamped()
            req.ik_request.pose_stamped.header.frame_id = BASE_LINK
            req.ik_request.pose_stamped.pose.position.x = float(xyz[0])
            req.ik_request.pose_stamped.pose.position.y = float(xyz[1])
            req.ik_request.pose_stamped.pose.position.z = float(xyz[2])
            req.ik_request.pose_stamped.pose.orientation.x = float(quat[0])
            req.ik_request.pose_stamped.pose.orientation.y = float(quat[1])
            req.ik_request.pose_stamped.pose.orientation.z = float(quat[2])
            req.ik_request.pose_stamped.pose.orientation.w = float(quat[3])
            req.ik_request.robot_state.joint_state = self._seed()
            req.ik_request.timeout = Duration(sec=1, nanosec=500_000_000)
            req.ik_request.avoid_collisions = False

            fut = self.ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=2.5)
            if not fut.done() or fut.result() is None:
                continue
            resp = fut.result()
            if resp.error_code.val != MoveItErrorCodes.SUCCESS:
                continue

            idx_map = {n: i for i, n in enumerate(resp.solution.joint_state.name)}
            if not all(n in idx_map for n in EXPECTED_JOINTS):
                continue

            rads = [resp.solution.joint_state.position[idx_map[n]] for n in EXPECTED_JOINTS]
            return IKPoint(
                j1=rad_to_logical(rads[0], 0),
                j2=rad_to_logical(rads[1], 1),
                j4=rad_to_logical(rads[2], 2),
                j5=rad_to_logical(rads[3], 3),
                j6=rad_to_logical(rads[4], 4),
                pen=pen,
            )
        return None  # all orientation candidates failed


def write_ik_points_header(points: list[IKPoint], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "#pragma once",
        "#include <stddef.h>",
        "",
        "// Auto-generated by generate_ik_points.py — DO NOT EDIT BY HAND.",
        "// Re-run the script to regenerate from updated CORNERS in that file.",
        "//",
        "// pen: 0 = PEN UP (travel)   1 = PEN DOWN (draw)",
        "// ID 3 mirrors ID 2 automatically in the Arduino sketch.",
        "",
        "struct IKPoint {",
        "  float   j1, j2, j4, j5, j6;",
        "  uint8_t pen;",
        "};",
        "",
        "const IKPoint kIKPoints[] = {",
    ]
    for i, pt in enumerate(points):
        lines.append(
            f"  {{ {pt.j1:.4f}f, {pt.j2:.4f}f, {pt.j4:.4f}f, {pt.j5:.4f}f, {pt.j6:.4f}f, {pt.pen} }},"
            f"  // point {i}"
        )
    lines += [
        "};",
        "",
        "const size_t kIKPointCount = sizeof(kIKPoints) / sizeof(kIKPoints[0]);",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {len(points)} IK points → {path}")


def main() -> None:
    rclpy.init()
    node = IKPointsGenerator()

    try:
        node.wait_ready()
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)

    results: list[IKPoint] = []
    failed: list[int] = []

    for i, (x, y, z, pen) in enumerate(CORNERS):
        print(f"  [{i+1}/{len(CORNERS)}] Solving IK for ({x:.4f}, {y:.4f}, {z:.4f})  pen={pen} …", end=" ")
        pt = node.solve((x, y, z), pen)
        if pt is None:
            print("FAILED — all orientation candidates exhausted")
            failed.append(i)
            # Insert a placeholder so the rest of the points keep their indices.
            results.append(IKPoint(j1=180.0, j2=180.0, j4=90.0, j5=180.0, j6=180.0, pen=pen))
        else:
            print(f"OK  j1={pt.j1:.1f} j2={pt.j2:.1f} j4={pt.j4:.1f} j5={pt.j5:.1f} j6={pt.j6:.1f}")
            results.append(pt)

    rclpy.shutdown()

    write_ik_points_header(results, OUTPUT_PATH)

    if failed:
        print(f"\nWARNING: IK failed for {len(failed)} point(s): indices {failed}")
        print("Placeholders were written. Adjust those corner positions or add")
        print("more ORIENTATION_CANDIDATES at the top of this script.")
    else:
        print(f"\nAll {len(results)} points solved successfully.")


if __name__ == "__main__":
    main()
