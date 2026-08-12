#!/usr/bin/env python3
"""Run the native C++ Mink-style differential IK loop for the FR3.

Unlike ``RobotModel.inverse_kinematics()``, ``differential_ik_step()`` solves
one local weighted QP and returns a tangent displacement/velocity.  This
example deliberately performs the outer integration loop in Python, just as a
controller would do; all FK, Jacobians, task assembly, and the box-QP solve
remain in C++.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import fr3_control_sim as fr3


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=PROJECT_ROOT / "models" / "fr3_franka_hand.urdf",
        help="FR3 + hand URDF (default: models/fr3_franka_hand.urdf).",
    )
    parser.add_argument(
        "--target",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="Target x y z [m] and roll pitch yaw [rad].",
    )
    parser.add_argument(
        "--seed",
        type=float,
        nargs=7,
        metavar="DEG",
        help="Initial seven joint angles in degrees (default: FR3 home).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=200,
        help="Number of external differential-IK steps (default: 200).",
    )
    parser.add_argument(
        "--dt", type=float, default=0.02, help="Control period in seconds."
    )
    parser.add_argument(
        "--posture-cost",
        type=float,
        default=0.01,
        help="Mink-style soft home PostureTask cost (0 disables it).",
    )
    parser.add_argument(
        "--posture-gain",
        type=float,
        default=1.0,
        help="Posture task gain in [0, 1] (default: 1).",
    )
    parser.add_argument(
        "--velocity-limits",
        action="store_true",
        help="Enforce the FR3 URDF velocity limits in the QP.",
    )
    parser.add_argument(
        "--no-position-limits",
        action="store_true",
        help="Disable the default linearized configuration limits.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.steps <= 0:
        raise SystemExit("--steps must be positive")
    if not np.isfinite(args.dt) or args.dt <= 0.0:
        raise SystemExit("--dt must be finite and positive")
    if not np.isfinite(args.posture_cost) or args.posture_cost < 0.0:
        raise SystemExit("--posture-cost must be finite and non-negative")
    if not np.isfinite(args.posture_gain) or not 0.0 <= args.posture_gain <= 1.0:
        raise SystemExit("--posture-gain must be in [0, 1]")

    urdf = args.urdf.expanduser().resolve()
    if not urdf.is_file():
        raise SystemExit(f"URDF does not exist: {urdf}")
    model = fr3.RobotModel(str(urdf))
    q_home = np.asarray(model.home_configuration(), dtype=float)
    q = q_home.copy() if args.seed is None else np.radians(args.seed)

    if args.target is None:
        # A small reachable displacement from home is a useful out-of-the-box
        # demonstration and keeps the orientation convention unambiguous.
        target = np.asarray(model.forward_kinematics(q_home), dtype=float)
        target[:3, 3] += np.array([0.03, -0.04, 0.02])
    else:
        target = np.asarray(
            fr3.pose_from_xyz_rpy(
                np.asarray(args.target[:3], dtype=float),
                np.asarray(args.target[3:], dtype=float),
            ),
            dtype=float,
        )

    options = fr3.DifferentialIKOptions()
    options.dt = float(args.dt)
    options.posture_cost = float(args.posture_cost)
    options.posture_gain = float(args.posture_gain)
    options.enforce_position_limits = not args.no_position_limits
    options.enforce_velocity_limits = bool(args.velocity_limits)

    print("Mink-style FR3 differential IK (native C++)")
    print(f"  urdf:           {urdf}")
    print(f"  dt:             {options.dt:.6g} s")
    print(f"  steps:          {args.steps}")
    print(f"  posture_cost:   {options.posture_cost:.6g}")
    print(f"  posture_gain:   {options.posture_gain:.6g}")
    print(f"  position limits: {options.enforce_position_limits}")
    print(f"  velocity limits: {options.enforce_velocity_limits}")

    last = None
    for index in range(args.steps):
        last = model.differential_ik_step(q, target, options)
        if not last.success:
            raise SystemExit(
                f"differential IK failed at step {index}: {last.status}"
            )
        q = np.asarray(last.next_q, dtype=float)
        if index == 0 or (index + 1) % 20 == 0 or index + 1 == args.steps:
            print(
                f"  step {index + 1:4d}: task_error={last.task_error:.3e} "
                f"next={last.next_task_error:.3e} "
                f"|v|={np.linalg.norm(last.velocity):.3e}"
            )

    assert last is not None
    print("final q [deg]:", np.array2string(np.degrees(q), precision=4))
    final_pose = np.asarray(model.forward_kinematics(q), dtype=float)
    print("final xyz [m]:", np.array2string(final_pose[:3, 3], precision=6))
    print(
        "final rpy [rad]:",
        np.array2string(np.asarray(fr3.rpy_from_pose(final_pose)), precision=6),
    )
    print(
        f"final frame errors: position={last.position_error:.3e} m, "
        f"orientation={last.orientation_error:.3e} rad"
    )


if __name__ == "__main__":
    main()
