#!/usr/bin/env python3
"""FR3 forward/inverse-kinematics and trajectory simulation."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Sequence

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]

import fr3_control_sim as fr3


DEFAULT_DESCRIPTION_ROOT = Path(
    os.environ.get("FRANKA_DESCRIPTION_ROOT", "/home/xense/fastiter/franka_description")
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("demo", "fk", "ik"), default="demo")
    parser.add_argument("--headless", action="store_true", help="Run without MeshCat.")
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Start MeshCat and print its URL without opening a browser.",
    )
    parser.add_argument("--urdf", type=Path, help="Path to a generated FR3 + hand URDF.")
    parser.add_argument(
        "--description-root",
        type=Path,
        default=DEFAULT_DESCRIPTION_ROOT,
        help="Official franka_description package root (used to resolve meshes).",
    )
    parser.add_argument(
        "--q",
        type=float,
        nargs="+",
        help="FK joint angles in degrees; defaults to a built-in FR3 pose.",
    )
    parser.add_argument(
        "--target",
        type=float,
        nargs="+",
        metavar="VALUE",
        help="IK target: x y z [roll pitch yaw], in meters and radians.",
    )
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--dt", type=float, default=0.02)
    return parser.parse_args()


def _resolve_urdf(requested: Path | None, description_root: Path) -> Path:
    if requested is not None:
        result = requested.expanduser().resolve()
        if not result.is_file():
            raise FileNotFoundError(f"URDF does not exist: {result}")
        return result

    candidates = (
        description_root.expanduser() / "urdfs" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "models" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "resources" / "fr3_franka_hand.urdf",
        PROJECT_ROOT / "share" / "fr3_control_sim" / "fr3_franka_hand.urdf",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n  ".join(str(path) for path in candidates)
    raise FileNotFoundError(
        "No generated official FR3 URDF was found. Generate one with\n"
        f"  cd {description_root.expanduser()} && python3 scripts/create_urdf.py fr3\n"
        "or pass --urdf explicitly. Searched:\n  " + searched
    )


def _demo_configuration(home: np.ndarray) -> np.ndarray:
    if home.shape != (7,):
        return home.copy()
    return np.array([0.35, -0.55, 0.25, -2.0, 0.15, 1.65, 0.40], dtype=float)


def _trajectory_array(trajectory: object, nq: int) -> np.ndarray:
    try:
        array = np.asarray(trajectory, dtype=float)
    except (TypeError, ValueError):
        array = np.asarray(
            [point.q if hasattr(point, "q") else point for point in trajectory],
            dtype=float,
        )
    if array.ndim == 1 and array.shape == (nq,):
        array = array.reshape(1, nq)
    if array.ndim != 2 or array.shape[1] != nq:
        raise ValueError(f"trajectory must have shape (N, {nq}), got {array.shape}")
    return array


def _print_pose(label: str, pose: Sequence[Sequence[float]]) -> None:
    matrix = np.asarray(pose, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"forward_kinematics must return a 4x4 pose, got {matrix.shape}")
    xyz = matrix[:3, 3]
    rpy = np.asarray(fr3.rpy_from_pose(matrix), dtype=float)
    print(f"{label} xyz [m]: {np.array2string(xyz, precision=5)}")
    print(f"{label} rpy [rad]: {np.array2string(rpy, precision=5)}")


def _make_target(model: fr3.RobotModel, home: np.ndarray, values: list[float] | None) -> np.ndarray:
    if values is None:
        return np.asarray(model.forward_kinematics(_demo_configuration(home)), dtype=float)
    if len(values) not in (3, 6):
        raise ValueError("--target requires x y z or x y z roll pitch yaw")
    if len(values) == 3:
        home_pose = np.asarray(model.forward_kinematics(home), dtype=float)
        rpy = np.asarray(fr3.rpy_from_pose(home_pose), dtype=float)
    else:
        rpy = np.asarray(values[3:], dtype=float)
    return np.asarray(
        fr3.pose_from_xyz_rpy(np.asarray(values[:3], dtype=float), rpy),
        dtype=float,
    )


def _run_fk(model: fr3.RobotModel, home: np.ndarray, q_values: list[float] | None) -> np.ndarray:
    q = _demo_configuration(home) if q_values is None else np.radians(q_values)
    if q.shape != (model.nq,):
        raise ValueError(f"--q requires {model.nq} joint values, got {q.size}")
    print(f"q [deg]: {np.array2string(np.degrees(q), precision=2)}")
    _print_pose("end effector", model.forward_kinematics(q))
    return q


def _solve_ik(
    model: fr3.RobotModel,
    home: np.ndarray,
    target_values: list[float] | None,
) -> np.ndarray:
    target = _make_target(model, home, target_values)
    _print_pose("target", target)
    result = model.inverse_kinematics(target, home, fr3.IKOptions())
    error = getattr(result, "error", getattr(result, "residual", float("nan")))
    iterations = getattr(result, "iterations", -1)
    print(f"IK success={result.success} iterations={iterations} error={error:.3e}")
    print(f"q [deg]: {np.array2string(np.degrees(result.q), precision=2)}")
    if not result.success:
        raise RuntimeError("inverse kinematics did not converge")
    solved = np.asarray(result.q, dtype=float)
    _print_pose("solved", model.forward_kinematics(solved))
    return solved


def main() -> None:
    args = _arguments()
    if args.duration <= 0.0 or args.dt <= 0.0:
        raise ValueError("--duration and --dt must be positive")

    description_root = args.description_root.expanduser().resolve()
    urdf_path = _resolve_urdf(args.urdf, description_root)
    model = fr3.RobotModel(str(urdf_path))
    home = np.asarray(model.home_configuration(), dtype=float)
    print(f"URDF: {urdf_path}")
    print(f"end effector: {model.end_effector_frame}")
    print(f"joints ({model.nq}): {', '.join(model.joint_names)}")

    visualizer = None
    if not args.headless:
        from fr3_control_sim.visualizer import Visualizer

        visualizer = Visualizer(
            model,
            urdf_path,
            description_root,
            open_browser=not args.no_open_browser,
        )
        print(f"MeshCat: {visualizer.url}")

    if args.mode == "fk":
        q_goal = _run_fk(model, home, args.q)
    else:
        q_goal = _solve_ik(model, home, args.target)

    if args.mode == "demo":
        _print_pose("home", model.forward_kinematics(home))

    if args.mode == "fk":
        if visualizer is not None:
            visualizer.update(q_goal)
        return

    trajectory = _trajectory_array(
        model.minimum_jerk_trajectory(home, q_goal, args.duration, args.dt),
        model.nq,
    )
    print(
        f"trajectory: {len(trajectory)} samples, duration={args.duration:.3f}s, "
        f"dt={args.dt:.3f}s"
    )
    if visualizer is not None:
        tcp_path = [
            np.asarray(model.forward_kinematics(q), dtype=float)[:3, 3]
            for q in trajectory
        ]
        visualizer.draw_path(tcp_path)
        visualizer.play(trajectory, args.dt)
        if args.mode == "demo":
            return_path = _trajectory_array(
                model.minimum_jerk_trajectory(q_goal, home, args.duration, args.dt),
                model.nq,
            )
            visualizer.play(return_path, args.dt)


if __name__ == "__main__":
    main()
