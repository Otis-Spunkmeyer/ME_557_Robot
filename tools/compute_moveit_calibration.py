#!/usr/bin/env python3
"""Compute MoveIt-to-Arduino calibration constants.

Sketch model:
  logical_deg = 180 + sign * (moveit_rad - home_rad) * RAD2DEG

Solved for home_rad:
  home_rad = moveit_rad - sign * (logical_deg - 180) * DEG2RAD
"""

from __future__ import annotations

import argparse

DEG2RAD = 0.01745329252
JOINTS = ["Motor1_joint", "Motor2_L", "Motor4_elb", "Motor5_wr", "Joint_EE"]


def parse_n(values: list[str], name: str, n: int) -> list[float]:
    if len(values) != n:
        raise SystemExit(f"{name} needs {n} values, got {len(values)}")
    return [float(v) for v in values]


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--moveit-rad", nargs=5, required=True)
    p.add_argument("--logical-deg", nargs=5, required=True)
    p.add_argument("--sign", nargs=5, required=True)
    args = p.parse_args()

    moveit = parse_n(args.moveit_rad, "--moveit-rad", 5)
    logical = parse_n(args.logical_deg, "--logical-deg", 5)
    signs = parse_n(args.sign, "--sign", 5)

    homes = []
    for i in range(5):
        sign = 1.0 if signs[i] >= 0 else -1.0
        home = moveit[i] - sign * (logical[i] - 180.0) * DEG2RAD
        homes.append(home)

    print("Use these constants:\n")
    print(
        "const float MOVEIT_DIR_SIGN[5] = { %.1ff, %.1ff, %.1ff, %.1ff, %.1ff };"
        % tuple(1.0 if s >= 0 else -1.0 for s in signs)
    )
    print("const float MOVEIT_HOME_RAD[5] = {")
    for i, h in enumerate(homes):
        comma = "," if i < 4 else ""
        print(f"  {h:.8f}f{comma}  // {JOINTS[i]}")
    print("};")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
