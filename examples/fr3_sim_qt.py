#!/usr/bin/env python3
"""Qt slider control panel for the FR3 Mink-style differential IK demo."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
from typing import Sequence

import numpy as np

import fr3_control_sim as fr3

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DESCRIPTION_ROOT = Path(
    os.environ.get("FRANKA_DESCRIPTION_ROOT", "/home/xense/fastiter/franka_description")
)
_DEFAULT_MINK_OPTIONS = fr3.DifferentialIKOptions()
DEFAULT_POSTURE_COST = 0.01
DEFAULT_POSTURE_GAIN = float(_DEFAULT_MINK_OPTIONS.posture_gain)
DEFAULT_MINK_STEPS = int(_DEFAULT_MINK_OPTIONS.max_iterations)
DEFAULT_MINK_DT = float(_DEFAULT_MINK_OPTIONS.dt)

try:
    from PySide6.QtCore import QSignalBlocker, Qt, QTimer, QUrl, Signal
    from PySide6.QtGui import QDesktopServices
    from PySide6.QtWidgets import (
        QApplication,
        QDoubleSpinBox,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QSlider,
        QSpinBox,
        QCheckBox,
        QTabWidget,
        QVBoxLayout,
        QWidget,
    )
except ModuleNotFoundError as exc:
    raise SystemExit(
        "PySide6 is required for fr3_sim_qt.py. Install it with:\n"
        "  mamba install -n fr3sim -c conda-forge pyside6\n"
        "or update the environment with:\n"
        "  mamba env update -n fr3sim -f environment.yml"
    ) from exc


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
    raise FileNotFoundError("No FR3 URDF was found. Searched:\n  " + searched)


class FloatControl(QWidget):
    """A floating-point value controlled by a slider and a spin box."""

    value_changed = Signal(float)

    def __init__(
        self,
        label: str,
        minimum: float,
        maximum: float,
        value: float,
        *,
        decimals: int,
        single_step: float,
        suffix: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        if not minimum < maximum:
            raise ValueError(f"invalid range for {label}: [{minimum}, {maximum}]")

        self._minimum = float(minimum)
        self._maximum = float(maximum)
        self._slider_steps = 20_000

        self.name_label = QLabel(label)
        self.name_label.setMinimumWidth(105)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0, self._slider_steps)
        self.slider.setTracking(True)

        self.spin_box = QDoubleSpinBox()
        self.spin_box.setRange(self._minimum, self._maximum)
        self.spin_box.setDecimals(decimals)
        self.spin_box.setSingleStep(single_step)
        self.spin_box.setSuffix(suffix)
        self.spin_box.setKeyboardTracking(True)
        self.spin_box.setMinimumWidth(135)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 2, 0, 2)
        layout.addWidget(self.name_label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin_box)

        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spin_box.valueChanged.connect(self._on_spin_changed)
        self.set_value(value)

    def _position_from_value(self, value: float) -> int:
        ratio = (value - self._minimum) / (self._maximum - self._minimum)
        return int(round(np.clip(ratio, 0.0, 1.0) * self._slider_steps))

    def _value_from_position(self, position: int) -> float:
        ratio = float(position) / self._slider_steps
        return self._minimum + ratio * (self._maximum - self._minimum)

    def _on_slider_changed(self, position: int) -> None:
        value = self._value_from_position(position)
        blocker = QSignalBlocker(self.spin_box)
        self.spin_box.setValue(value)
        del blocker
        self.value_changed.emit(value)

    def _on_spin_changed(self, value: float) -> None:
        blocker = QSignalBlocker(self.slider)
        self.slider.setValue(self._position_from_value(value))
        del blocker
        self.value_changed.emit(float(value))

    def value(self) -> float:
        return float(self.spin_box.value())

    def set_value(self, value: float, *, emit: bool = False) -> None:
        clamped = float(np.clip(value, self._minimum, self._maximum))
        slider_blocker = QSignalBlocker(self.slider)
        spin_blocker = QSignalBlocker(self.spin_box)
        self.slider.setValue(self._position_from_value(clamped))
        self.spin_box.setValue(clamped)
        del slider_blocker
        del spin_blocker
        if emit:
            self.value_changed.emit(clamped)


class Fr3SimWindow(QMainWindow):
    """FK and IK slider controls backed by the C++ ``RobotModel``."""

    def __init__(
        self,
        model: fr3.RobotModel,
        visualizer: object | None,
        urdf_path: Path,
        *,
        initial_mode: str = "fk",
        posture_gain: float = DEFAULT_POSTURE_GAIN,
        posture_cost: float = DEFAULT_POSTURE_COST,
        mink_steps: int = DEFAULT_MINK_STEPS,
        dt: float = DEFAULT_MINK_DT,
        velocity_limits: bool = False,
    ) -> None:
        super().__init__()
        self.model = model
        self.visualizer = visualizer
        self.urdf_path = urdf_path
        self.home_q = np.asarray(model.home_configuration(), dtype=float)
        self.current_q = self.home_q.copy()
        self.ik_options = fr3.DifferentialIKOptions()
        if not np.isfinite(posture_gain) or not 0.0 <= posture_gain <= 1.0:
            raise ValueError("--posture-gain must be finite and in [0, 1]")
        if not np.isfinite(posture_cost) or posture_cost < 0.0:
            raise ValueError("--posture-cost must be finite and non-negative")
        if int(mink_steps) <= 0:
            raise ValueError("--mink-steps must be positive")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("--dt must be finite and positive")
        self.ik_options.posture_gain = float(posture_gain)
        self.ik_options.posture_cost = float(posture_cost)
        self.ik_options.max_iterations = int(mink_steps)
        self.ik_options.dt = float(dt)
        self.ik_options.enforce_velocity_limits = bool(velocity_limits)
        self.velocity_limits_enabled = bool(velocity_limits)
        self._syncing_controls = False

        self.fk_timer = QTimer(self)
        self.fk_timer.setSingleShot(True)
        self.fk_timer.setInterval(20)
        self.fk_timer.timeout.connect(self._apply_fk)

        self.ik_timer = QTimer(self)
        self.ik_timer.setSingleShot(True)
        self.ik_timer.setInterval(80)
        self.ik_timer.timeout.connect(self._solve_ik)

        self.setWindowTitle("FR3 Mink Differential IK Control")
        self.resize(900, 700)
        self._build_ui()
        self._reset_home()
        self.tabs.setCurrentIndex(0 if initial_mode == "fk" else 1)

    def _build_ui(self) -> None:
        central = QWidget()
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(16, 14, 16, 14)
        root_layout.setSpacing(10)

        title = QLabel("FR3 C++ FK / Mink Differential IK")
        title.setStyleSheet("font-size: 20px; font-weight: 600;")
        root_layout.addWidget(title)
        root_layout.addWidget(QLabel(f"URDF: {self.urdf_path}"))

        meshcat_row = QHBoxLayout()
        if self.visualizer is None:
            self.meshcat_label = QLabel("MeshCat: disabled")
            self.open_meshcat_button = QPushButton("Open MeshCat")
            self.open_meshcat_button.setEnabled(False)
        else:
            self.meshcat_url = str(self.visualizer.url)
            self.meshcat_label = QLabel(f"MeshCat: {self.meshcat_url}")
            self.open_meshcat_button = QPushButton("Open MeshCat")
            self.open_meshcat_button.clicked.connect(self._open_meshcat)
        meshcat_row.addWidget(self.meshcat_label, 1)
        meshcat_row.addWidget(self.open_meshcat_button)
        root_layout.addLayout(meshcat_row)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_fk_tab(), "FK - Joint sliders")
        self.tabs.addTab(self._build_ik_tab(), "IK - XYZ / RPY sliders")
        self.tabs.currentChanged.connect(self._on_tab_changed)
        root_layout.addWidget(self.tabs, 1)

        buttons = QHBoxLayout()
        home_button = QPushButton("Reset Home")
        home_button.clicked.connect(self._reset_home)
        buttons.addStretch(1)
        buttons.addWidget(home_button)
        root_layout.addLayout(buttons)

        self.setCentralWidget(central)

    def _build_fk_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        description = QLabel(
            "Drag joint1 ... joint7 in degrees. Joint ranges come from the C++ model."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        group = QGroupBox("Joint angles")
        group_layout = QVBoxLayout(group)
        limits = np.asarray(self.model.joint_limits, dtype=float)
        home_degrees = np.degrees(self.home_q)
        self.fk_controls: list[FloatControl] = []
        for index, (joint_name, limit, value) in enumerate(
            zip(self.model.joint_names, limits, home_degrees, strict=True)
        ):
            control = FloatControl(
                f"joint{index + 1} ({joint_name})",
                math.degrees(float(limit[0])),
                math.degrees(float(limit[1])),
                float(value),
                decimals=2,
                single_step=0.1,
                suffix=" deg",
            )
            control.value_changed.connect(self._schedule_fk)
            self.fk_controls.append(control)
            group_layout.addWidget(control)
        layout.addWidget(group)

        self.fk_status = QLabel()
        self.fk_status.setWordWrap(True)
        self.fk_status.setStyleSheet("font-family: monospace;")
        layout.addWidget(self.fk_status)
        layout.addStretch(1)
        return page

    def _build_ik_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        description = QLabel(
            "Drag x/y/z in meters and roll/pitch/yaw in radians. "
            "IK is solved from the last successful configuration."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        group = QGroupBox("Target pose")
        group_layout = QVBoxLayout(group)
        self.ik_controls: dict[str, FloatControl] = {}
        specifications = (
            ("x", -0.9, 0.9, 4, 0.001, " m"),
            ("y", -0.9, 0.9, 4, 0.001, " m"),
            ("z", 0.0, 1.2, 4, 0.001, " m"),
            ("roll", -math.pi, math.pi, 4, 0.01, " rad"),
            ("pitch", -math.pi, math.pi, 4, 0.01, " rad"),
            ("yaw", -math.pi, math.pi, 4, 0.01, " rad"),
        )
        for name, minimum, maximum, decimals, step, suffix in specifications:
            control = FloatControl(
                name,
                minimum,
                maximum,
                0.0,
                decimals=decimals,
                single_step=step,
                suffix=suffix,
            )
            control.value_changed.connect(self._schedule_ik)
            self.ik_controls[name] = control
            group_layout.addWidget(control)
        layout.addWidget(group)

        options_group = QGroupBox("Mink IK options")
        options_layout = QHBoxLayout(options_group)

        options_layout.addWidget(QLabel("posture_cost:"))
        self.posture_cost_spin_box = QDoubleSpinBox()
        self.posture_cost_spin_box.setRange(0.0, 10.0)
        self.posture_cost_spin_box.setDecimals(5)
        self.posture_cost_spin_box.setSingleStep(0.001)
        self.posture_cost_spin_box.setValue(float(self.ik_options.posture_cost))
        self.posture_cost_spin_box.setToolTip(
            "Soft Mink PostureTask cost; zero disables the posture task."
        )
        self.posture_cost_spin_box.valueChanged.connect(
            self._on_posture_cost_changed
        )
        options_layout.addWidget(self.posture_cost_spin_box)

        options_layout.addWidget(QLabel("posture_gain:"))
        self.posture_gain_spin_box = QDoubleSpinBox()
        self.posture_gain_spin_box.setRange(0.0, 1.0)
        self.posture_gain_spin_box.setDecimals(3)
        self.posture_gain_spin_box.setSingleStep(0.01)
        self.posture_gain_spin_box.setValue(float(self.ik_options.posture_gain))
        self.posture_gain_spin_box.setToolTip(
            "Mink task gain in [0, 1]. It scales the posture correction."
        )
        self.posture_gain_spin_box.valueChanged.connect(
            self._on_posture_gain_changed
        )
        options_layout.addWidget(self.posture_gain_spin_box)

        options_layout.addWidget(QLabel("steps:"))
        self.mink_steps_spin_box = QSpinBox()
        self.mink_steps_spin_box.setRange(1, 10000)
        self.mink_steps_spin_box.setSingleStep(10)
        self.mink_steps_spin_box.setValue(int(self.ik_options.max_iterations))
        self.mink_steps_spin_box.setToolTip(
            "Number of external Mink differential IK steps."
        )
        self.mink_steps_spin_box.valueChanged.connect(self._on_mink_steps_changed)
        options_layout.addWidget(self.mink_steps_spin_box)

        options_layout.addWidget(QLabel("dt:"))
        self.dt_spin_box = QDoubleSpinBox()
        self.dt_spin_box.setRange(1e-4, 1.0)
        self.dt_spin_box.setDecimals(4)
        self.dt_spin_box.setSingleStep(0.001)
        self.dt_spin_box.setValue(float(self.ik_options.dt))
        self.dt_spin_box.setSuffix(" s")
        self.dt_spin_box.setToolTip("Mink differential IK integration period.")
        self.dt_spin_box.valueChanged.connect(self._on_dt_changed)
        options_layout.addWidget(self.dt_spin_box)

        self.velocity_limits_check = QCheckBox("velocity limits")
        self.velocity_limits_check.setChecked(self.velocity_limits_enabled)
        self.velocity_limits_check.setToolTip(
            "Add FR3 joint velocity bounds to the Mink box QP."
        )
        self.velocity_limits_check.toggled.connect(
            self._on_velocity_limits_changed
        )
        options_layout.addWidget(self.velocity_limits_check)
        options_layout.addStretch(1)
        layout.addWidget(options_group)

        ik_buttons = QHBoxLayout()
        current_target_button = QPushButton("Set target from current pose")
        current_target_button.clicked.connect(self._sync_ik_target_from_current)
        solve_button = QPushButton("Solve IK now")
        solve_button.clicked.connect(self._solve_ik)
        ik_buttons.addWidget(current_target_button)
        ik_buttons.addStretch(1)
        ik_buttons.addWidget(solve_button)
        layout.addLayout(ik_buttons)

        self.ik_status = QLabel()
        self.ik_status.setWordWrap(True)
        self.ik_status.setStyleSheet("font-family: monospace;")
        layout.addWidget(self.ik_status)
        layout.addStretch(1)
        return page

    def _on_posture_cost_changed(self, value: float) -> None:
        """Apply a new Mink PostureTask cost and re-solve the target."""
        if self._syncing_controls:
            return
        self.ik_options.posture_cost = float(value)
        self.ik_status.setText(
            f"posture_cost = {self.ik_options.posture_cost:.5f}\n"
            "Mink option updated; solving..."
        )
        self.ik_status.setStyleSheet("font-family: monospace; color: #b36b00;")
        if self.tabs.currentIndex() == 1:
            self._schedule_ik(value)

    def _on_posture_gain_changed(self, value: float) -> None:
        """Apply a new Mink task gain and re-solve the current target."""
        if self._syncing_controls:
            return
        self.ik_options.posture_gain = float(value)
        self.ik_status.setText(
            f"posture_gain = {self.ik_options.posture_gain:.3f}\n"
            "IK option updated; solving..."
        )
        self.ik_status.setStyleSheet("font-family: monospace; color: #b36b00;")
        if self.tabs.currentIndex() == 1:
            self._schedule_ik(value)

    def _on_mink_steps_changed(self, value: int) -> None:
        if self._syncing_controls:
            return
        self.ik_options.max_iterations = int(value)
        if self.tabs.currentIndex() == 1:
            self._schedule_ik(float(value))

    def _on_dt_changed(self, value: float) -> None:
        if self._syncing_controls:
            return
        self.ik_options.dt = float(value)
        if self.tabs.currentIndex() == 1:
            self._schedule_ik(value)

    def _on_velocity_limits_changed(self, enabled: bool) -> None:
        if self._syncing_controls:
            return
        self.velocity_limits_enabled = bool(enabled)
        self.ik_options.enforce_velocity_limits = bool(enabled)
        if self.tabs.currentIndex() == 1:
            self._schedule_ik(1.0 if enabled else 0.0)

    @staticmethod
    def _pose_text(pose: Sequence[Sequence[float]]) -> str:
        matrix = np.asarray(pose, dtype=float)
        xyz = matrix[:3, 3]
        rpy = np.asarray(fr3.rpy_from_pose(matrix), dtype=float)
        return (
            f"xyz [m]   = {np.array2string(xyz, precision=5)}\n"
            f"rpy [rad] = {np.array2string(rpy, precision=5)}"
        )

    def _update_visualizer(self, q: np.ndarray) -> None:
        if self.visualizer is not None:
            self.visualizer.update(q)

    def _schedule_fk(self, _value: float) -> None:
        if self._syncing_controls:
            return
        self.ik_timer.stop()
        self.fk_timer.start()

    def _schedule_ik(self, _value: float) -> None:
        if self._syncing_controls:
            return
        self.fk_timer.stop()
        self.ik_status.setText("Solving IK...")
        self.ik_status.setStyleSheet("font-family: monospace; color: #b36b00;")
        self.ik_timer.start()

    def _joint_values_radians(self) -> np.ndarray:
        return np.radians([control.value() for control in self.fk_controls])

    def _target_pose(self) -> np.ndarray:
        xyz = np.array(
            [self.ik_controls[name].value() for name in ("x", "y", "z")],
            dtype=float,
        )
        rpy = np.array(
            [
                self.ik_controls[name].value()
                for name in ("roll", "pitch", "yaw")
            ],
            dtype=float,
        )
        return np.asarray(fr3.pose_from_xyz_rpy(xyz, rpy), dtype=float)

    def _set_fk_controls(self, q: np.ndarray) -> None:
        self._syncing_controls = True
        try:
            for control, value in zip(
                self.fk_controls, np.degrees(q), strict=True
            ):
                control.set_value(float(value))
        finally:
            self._syncing_controls = False

    def _set_ik_controls_from_pose(self, pose: np.ndarray) -> None:
        xyz = pose[:3, 3]
        rpy = np.asarray(fr3.rpy_from_pose(pose), dtype=float)
        self._syncing_controls = True
        try:
            for name, value in zip(("x", "y", "z"), xyz, strict=True):
                self.ik_controls[name].set_value(float(value))
            for name, value in zip(("roll", "pitch", "yaw"), rpy, strict=True):
                self.ik_controls[name].set_value(float(value))
        finally:
            self._syncing_controls = False

    def _apply_fk(self) -> None:
        try:
            q = self._joint_values_radians()
            pose = np.asarray(self.model.forward_kinematics(q), dtype=float)
            self.current_q = q
            self._update_visualizer(q)
            self.fk_status.setText(
                f"q [deg] = {np.array2string(np.degrees(q), precision=2)}\n"
                + self._pose_text(pose)
            )
            self.fk_status.setStyleSheet("font-family: monospace; color: #1b7f2a;")
        except Exception as exc:  # keep the GUI responsive on invalid input
            self.fk_status.setText(f"FK failed: {exc}")
            self.fk_status.setStyleSheet("font-family: monospace; color: #b00020;")

    def _solve_ik(self) -> None:
        self.ik_timer.stop()
        try:
            target = self._target_pose()
            result = self.model.mink_inverse_kinematics(
                target, self.current_q, self.ik_options
            )
            if not result.success:
                self.ik_status.setText(
                    "Mink IK not converged\n"
                    f"posture_cost={self.ik_options.posture_cost:.5f} "
                    f"posture_gain={self.ik_options.posture_gain:.3f}\n"
                    f"iterations={result.iterations} "
                    f"error={result.error:.3e}\n"
                    f"position_error={result.position_error:.3e} "
                    f"orientation_error={result.orientation_error:.3e}"
                )
                self.ik_status.setStyleSheet(
                    "font-family: monospace; color: #b00020;"
                )
                return

            solved = np.asarray(result.q, dtype=float)
            self.current_q = solved
            self._set_fk_controls(solved)
            self._update_visualizer(solved)
            solved_pose = np.asarray(
                self.model.forward_kinematics(solved), dtype=float
            )
            self.ik_status.setText(
                "Mink IK converged\n"
                f"posture_cost={self.ik_options.posture_cost:.5f} "
                f"posture_gain={self.ik_options.posture_gain:.3f}\n"
                f"iterations={result.iterations} "
                f"error={result.error:.3e}\n"
                f"q [deg] = {np.array2string(np.degrees(solved), precision=2)}\n"
                + self._pose_text(solved_pose)
            )
            self.ik_status.setStyleSheet(
                "font-family: monospace; color: #1b7f2a;"
            )
        except Exception as exc:  # keep the GUI responsive on invalid targets
            self.ik_status.setText(f"IK failed: {exc}")
            self.ik_status.setStyleSheet("font-family: monospace; color: #b00020;")

    def _sync_ik_target_from_current(self) -> None:
        self.ik_timer.stop()
        pose = np.asarray(
            self.model.forward_kinematics(self.current_q), dtype=float
        )
        self._set_ik_controls_from_pose(pose)
        self.ik_status.setText(
            "IK target synchronized with the current pose.\n"
            + self._pose_text(pose)
        )
        self.ik_status.setStyleSheet("font-family: monospace;")

    def _reset_home(self) -> None:
        self.fk_timer.stop()
        self.ik_timer.stop()
        self.current_q = self.home_q.copy()
        self._set_fk_controls(self.current_q)
        pose = np.asarray(
            self.model.forward_kinematics(self.current_q), dtype=float
        )
        self._set_ik_controls_from_pose(pose)
        self._update_visualizer(self.current_q)
        self.fk_status.setText(
            f"q [deg] = {np.array2string(np.degrees(self.current_q), precision=2)}\n"
            + self._pose_text(pose)
        )
        self.fk_status.setStyleSheet("font-family: monospace; color: #1b7f2a;")
        self.ik_status.setText("IK target initialized from home.\n" + self._pose_text(pose))
        self.ik_status.setStyleSheet("font-family: monospace;")

    def _on_tab_changed(self, index: int) -> None:
        if index == 0:
            self.ik_timer.stop()
            self._set_fk_controls(self.current_q)
            self._apply_fk()
        else:
            self.fk_timer.stop()
            self._sync_ik_target_from_current()

    def _open_meshcat(self) -> None:
        if self.visualizer is not None:
            QDesktopServices.openUrl(QUrl(self.meshcat_url))


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("fk", "ik"), default="fk")
    parser.add_argument("--urdf", type=Path, help="Path to the generated FR3 URDF.")
    parser.add_argument(
        "--description-root",
        type=Path,
        default=DEFAULT_DESCRIPTION_ROOT,
        help="franka_description package root used to resolve MeshCat meshes.",
    )
    parser.add_argument(
        "--no-open-browser",
        action="store_true",
        help="Start MeshCat without opening its browser page automatically.",
    )
    parser.add_argument(
        "--no-meshcat",
        action="store_true",
        help="Run the Qt control panel without MeshCat visualization.",
    )
    parser.add_argument(
        "--posture-gain",
        type=float,
        default=DEFAULT_POSTURE_GAIN,
        help=(
            f"Mink PostureTask gain in [0, 1] (default: {DEFAULT_POSTURE_GAIN:g})."
        ),
    )
    parser.add_argument(
        "--posture-cost",
        type=float,
        default=DEFAULT_POSTURE_COST,
        help=f"Mink PostureTask cost (default: {DEFAULT_POSTURE_COST:g}).",
    )
    parser.add_argument(
        "--mink-steps",
        type=int,
        default=DEFAULT_MINK_STEPS,
        help=f"External Mink differential IK steps (default: {DEFAULT_MINK_STEPS}).",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_MINK_DT,
        help=f"Mink integration period in seconds (default: {DEFAULT_MINK_DT:g}).",
    )
    parser.add_argument(
        "--velocity-limits",
        action="store_true",
        help="Enforce FR3 joint velocity limits in the Mink QP.",
    )
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    description_root = args.description_root.expanduser().resolve()
    urdf_path = _resolve_urdf(args.urdf, description_root)
    model = fr3.RobotModel(str(urdf_path))
    print(
        "Mink IK: "
        f"posture_cost={args.posture_cost:g}, "
        f"posture_gain={args.posture_gain:g}, "
        f"steps={args.mink_steps}, dt={args.dt:g}, "
        f"velocity_limits={args.velocity_limits}"
    )

    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("FR3 Control Sim")

    visualizer = None
    if not args.no_meshcat:
        from fr3_control_sim.visualizer import Visualizer

        visualizer = Visualizer(
            model,
            urdf_path,
            description_root,
            open_browser=not args.no_open_browser,
        )
        print(f"MeshCat: {visualizer.url}")

    window = Fr3SimWindow(
        model,
        visualizer,
        urdf_path,
        initial_mode=args.mode,
        posture_gain=args.posture_gain,
        posture_cost=args.posture_cost,
        mink_steps=args.mink_steps,
        dt=args.dt,
        velocity_limits=args.velocity_limits,
    )
    window.show()
    return app.exec()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        app = QApplication.instance()
        if app is not None:
            QMessageBox.critical(None, "FR3 simulation error", str(exc))
        raise
