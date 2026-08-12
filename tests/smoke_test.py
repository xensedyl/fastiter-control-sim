#!/usr/bin/env python3
"""Pybind-level FK/IK smoke test; all kinematics execute in C++."""

from __future__ import annotations

from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

import fr3_control_sim as fr3


def main() -> None:
    model = fr3.RobotModel(str(PROJECT_ROOT / "models" / "fr3_franka_hand.urdf"))
    assert model.nq == 7
    assert model.joint_names == [f"fr3_joint{index}" for index in range(1, 8)]

    home = np.asarray(model.home_configuration(), dtype=float)
    target_q = home + np.array([0.20, 0.10, -0.15, 0.10, 0.12, -0.10, -0.20])
    target = np.asarray(model.forward_kinematics(target_q), dtype=float)
    result = model.inverse_kinematics(target, home, fr3.IKOptions())
    assert result.success, result

    recovered = np.asarray(model.forward_kinematics(result.q), dtype=float)
    position_error = float(np.linalg.norm(recovered[:3, 3] - target[:3, 3]))
    assert position_error < 1e-5, position_error

    trajectory = np.asarray(
        model.minimum_jerk_trajectory(home, result.q, 1.0, 0.02), dtype=float
    )
    assert trajectory.shape == (51, 7)
    assert np.allclose(trajectory[0], home)
    assert np.allclose(trajectory[-1], result.q)

    skewed_seed = np.radians(
        [-27.75, -48.05, 17.44, -134.54, 12.89, 87.89, 29.59]
    )
    near_home_target = np.asarray(
        fr3.pose_from_xyz_rpy(
            np.array([0.3069, 0.0, 0.4869]),
            np.array([-3.1416, 0.0, 0.0]),
        ),
        dtype=float,
    )
    posture_options = fr3.IKOptions()
    posture_options.max_retries = 0
    posture_options.posture_gain = 0.1
    posture_result = model.inverse_kinematics(
        near_home_target, skewed_seed, posture_options
    )
    assert posture_result.success, posture_result
    posture_distance = float(np.linalg.norm(posture_result.q - home))
    assert posture_distance < 1e-3, posture_distance

    print("Python/pybind smoke test passed")
    print(f"  IK residual: {result.error:.3e}")
    print(f"  position round-trip error: {position_error:.3e} m")
    print(f"  null-space home distance: {posture_distance:.3e} rad")


if __name__ == "__main__":
    main()
