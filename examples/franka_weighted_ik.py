#!/usr/bin/env python3
"""Run the LeFranX weighted redundant-joint IK on a loaded URDF.

The original LeFranX ``geofik`` formulas are hard-coded for the official
Franka geometry.  This example uses the same weighted free-joint search and
candidate ranking, while Pinocchio supplies FK/Jacobians for the selected
URDF.  It therefore also works with ``models/URDF0820/URDF0820.urdf`` and its
mimic joints.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import fr3_control_sim as fr3


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=PROJECT_ROOT / "models" / "URDF0820" / "URDF0820.urdf",
        help="7-DoF URDF (default: models/URDF0820/URDF0820.urdf)",
    )
    parser.add_argument("--end-effector", default="")
    parser.add_argument(
        "--target",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="target xyz in meters and RPY in radians; defaults to a nearby pose",
    )
    parser.add_argument(
        "--q-seed",
        type=float,
        nargs="+",
        metavar="DEG",
        help="seed joint angles in degrees (default: model home)",
    )
    parser.add_argument("--samples", type=int, default=121)
    parser.add_argument("--max-iterations", type=int, default=80)
    parser.add_argument("--weight-manipulability", type=float, default=1.0)
    parser.add_argument("--weight-neutral", type=float, default=1.0)
    parser.add_argument("--weight-current", type=float, default=2.0)
    args = parser.parse_args()

    urdf = args.urdf.expanduser().resolve()
    if not urdf.is_file():
        raise FileNotFoundError(f"URDF does not exist: {urdf}")
    model = fr3.RobotModel(str(urdf), args.end_effector)
    home = np.asarray(model.home_configuration(), dtype=float)
    if args.q_seed is None:
        q_seed = home
    else:
        q_seed = np.radians(np.asarray(args.q_seed, dtype=float))
        if q_seed.shape != (model.nq,):
            raise ValueError(f"--q-seed requires {model.nq} values")

    if args.target is None:
        target_q = q_seed + np.radians(
            np.array([2.0, -2.0, 2.0, 1.0, 2.0, -1.0, 2.0])[: model.nq]
        )
        target = np.asarray(model.forward_kinematics(target_q), dtype=float)
    else:
        values = np.asarray(args.target, dtype=float)
        target = np.asarray(
            fr3.pose_from_xyz_rpy(values[:3], values[3:]), dtype=float
        )

    options = fr3.FrankaWeightedIKOptions()
    options.samples = args.samples
    options.max_iterations = args.max_iterations
    options.weight_manipulability = args.weight_manipulability
    options.weight_neutral = args.weight_neutral
    options.weight_current = args.weight_current

    print(f"URDF: {urdf}")
    print(f"end effector: {model.end_effector_frame}")
    print(f"joints: {', '.join(model.joint_names)}")
    print(f"seed [deg]: {np.array2string(np.degrees(q_seed), precision=3)}")
    print("target pose:\n", np.array2string(target, precision=6))
    result = model.franka_weighted_ik(target, q_seed, options)
    print(
        "weighted IK: success={} error={:.3e} score={:.6g} "
        "valid_solutions={} samples={} free_joint={} ".format(
            result.success,
            result.error,
            result.score,
            result.valid_solutions,
            result.samples_tested,
            result.free_joint_index + 1,
        )
    )
    print(f"  manipulability: {result.manipulability:.6g}")
    print(f"  neutral distance: {result.neutral_distance:.6g}")
    print(f"  current distance: {result.current_distance:.6g}")
    print(f"q [deg]: {np.array2string(np.degrees(result.q), precision=3)}")
    solved = np.asarray(model.forward_kinematics(result.q), dtype=float)
    print("solved pose:\n", np.array2string(solved, precision=6))
    if not result.success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
