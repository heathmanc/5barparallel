"""Settings tab: view/edit robot geometry and workspace guard thresholds.

Geometry (L1, L2, base spacing, joint limits, branch, drivetrain) is editable,
but Claude.md §3 forbids silently reverting the *verified* design. So Apply
re-runs the full work-zone validation (the six-hole span + cap pick + ±2 in
cross-conveyor tolerance) and REFUSES any geometry that can't clear every
singularity/reach check. Only geometry that passes is pushed to the live
controller; Save then writes it back to config/robot_config.yaml.

The geometry must also match the physically-built arms — this tab does not
change hardware, only what the kinematics believe.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController
from ..robot.fivebar_kinematics import FiveBarConfig, FiveBarKinematics
from ..robot.workspace import SingularityLimits, WorkspaceValidator

# The verified work zone in the robot frame (Claude.md §3, §4). New geometry
# must keep all of these valid to be accepted.
_Y_NOM = 250.0
_TOL = 50.8
WORK_ZONE: List[Tuple[float, float]] = [
    (x, y)
    for y in (_Y_NOM - _TOL, _Y_NOM, _Y_NOM + _TOL)
    for x in (-175.0, -125.0, -75.0, 0.0, 75.0, 125.0, 175.0)
]


def validate_work_zone(validator: WorkspaceValidator) -> List[Tuple[float, float, str]]:
    """Return [(x, y, reason)] for every work-zone point the geometry fails.
    Empty list means the geometry covers the whole verified work zone."""
    failures: List[Tuple[float, float, str]] = []
    for x, y in WORK_ZONE:
        res = validator.validate(x, y)
        if not res.ok:
            failures.append((x, y, res.reason))
    return failures


class SettingsTab(QWidget):
    geometryChanged = Signal()

    def __init__(
        self,
        controller: RobotTestController,
        config_path: Optional[str | Path] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.config_path = Path(config_path) if config_path else None
        self._floats: Dict[str, QDoubleSpinBox] = {}

        root = QVBoxLayout(self)
        root.addWidget(self._build_geometry_group())
        root.addWidget(self._build_limits_group())
        root.addWidget(self._build_buttons())
        self.status_label = QLabel("")
        root.addWidget(self.status_label)
        root.addStretch(1)

        self._load_from(controller.kin.config, controller.validator.limits)

    # --- widgets ------------------------------------------------------------
    def _fspin(self, key: str, lo: float, hi: float, step: float, decimals: int):
        w = QDoubleSpinBox()
        w.setRange(lo, hi)
        w.setSingleStep(step)
        w.setDecimals(decimals)
        self._floats[key] = w
        return w

    def _build_geometry_group(self) -> QGroupBox:
        box = QGroupBox("Mechanical geometry (must match the built arms)")
        form = QFormLayout(box)
        form.addRow("Proximal link L1 (mm)", self._fspin("l1_mm", 50, 600, 1, 2))
        form.addRow("Distal link L2 (mm)", self._fspin("l2_mm", 50, 600, 1, 2))
        form.addRow("Base spacing (mm)", self._fspin("base_spacing_mm", 10, 400, 1, 2))
        form.addRow("Joint min (deg)", self._fspin("joint_min_deg", -180, 360, 1, 1))
        form.addRow("Joint max (deg)", self._fspin("joint_max_deg", -180, 360, 1, 1))

        self.left_elbow = QComboBox()
        self.left_elbow.addItems(["up", "down"])
        self.right_elbow = QComboBox()
        self.right_elbow.addItems(["up", "down"])
        form.addRow("Left elbow branch", self.left_elbow)
        form.addRow("Right elbow branch", self.right_elbow)

        self.pulses_per_rev = QSpinBox()
        self.pulses_per_rev.setRange(200, 200000)
        self.pulses_per_rev.setSingleStep(200)
        form.addRow("Pulses / motor rev", self.pulses_per_rev)
        form.addRow("Belt reduction (:1)", self._fspin("belt_reduction", 1, 50, 0.5, 3))

        self.derived_label = QLabel()
        form.addRow("Derived", self.derived_label)
        for w in self._floats.values():
            w.valueChanged.connect(self._update_derived)
        self.pulses_per_rev.valueChanged.connect(self._update_derived)
        return box

    def _build_limits_group(self) -> QGroupBox:
        box = QGroupBox("Workspace guard thresholds")
        form = QFormLayout(box)
        form.addRow("Parallel min (deg)", self._fspin("parallel_min_deg", 0, 90, 1, 1))
        form.addRow("Serial min (deg)", self._fspin("serial_min_deg", 0, 90, 1, 1))
        form.addRow(
            "Reach fraction max", self._fspin("reach_fraction_max", 0.1, 1.0, 0.01, 3)
        )
        return box

    def _build_buttons(self) -> QWidget:
        wrap = QWidget()
        row = QHBoxLayout(wrap)
        apply_btn = QPushButton("Validate && Apply")
        apply_btn.clicked.connect(self._on_apply)
        save_btn = QPushButton("Save to YAML")
        save_btn.clicked.connect(self._on_save)
        reload_btn = QPushButton("Reload from YAML")
        reload_btn.clicked.connect(self._on_reload)
        row.addWidget(apply_btn)
        row.addWidget(save_btn)
        row.addWidget(reload_btn)
        row.addStretch(1)
        self.save_btn = save_btn
        self.save_btn.setEnabled(self.config_path is not None)
        return wrap

    # --- state <-> widgets --------------------------------------------------
    def _load_from(self, config: FiveBarConfig, limits: SingularityLimits) -> None:
        self._floats["l1_mm"].setValue(config.l1_mm)
        self._floats["l2_mm"].setValue(config.l2_mm)
        self._floats["base_spacing_mm"].setValue(config.base_spacing_mm)
        self._floats["joint_min_deg"].setValue(config.joint_min_deg)
        self._floats["joint_max_deg"].setValue(config.joint_max_deg)
        self._floats["belt_reduction"].setValue(config.belt_reduction)
        self.left_elbow.setCurrentText(config.left_elbow)
        self.right_elbow.setCurrentText(config.right_elbow)
        self.pulses_per_rev.setValue(config.pulses_per_rev)
        self._floats["parallel_min_deg"].setValue(limits.parallel_min_deg)
        self._floats["serial_min_deg"].setValue(limits.serial_min_deg)
        self._floats["reach_fraction_max"].setValue(limits.reach_fraction_max)
        self._update_derived()

    def _read_config(self) -> FiveBarConfig:
        return FiveBarConfig(
            l1_mm=self._floats["l1_mm"].value(),
            l2_mm=self._floats["l2_mm"].value(),
            base_spacing_mm=self._floats["base_spacing_mm"].value(),
            left_elbow=self.left_elbow.currentText(),
            right_elbow=self.right_elbow.currentText(),
            joint_min_deg=self._floats["joint_min_deg"].value(),
            joint_max_deg=self._floats["joint_max_deg"].value(),
            pulses_per_rev=self.pulses_per_rev.value(),
            belt_reduction=self._floats["belt_reduction"].value(),
        )

    def _read_limits(self) -> SingularityLimits:
        return SingularityLimits(
            parallel_min_deg=self._floats["parallel_min_deg"].value(),
            serial_min_deg=self._floats["serial_min_deg"].value(),
            reach_fraction_max=self._floats["reach_fraction_max"].value(),
        )

    def _update_derived(self) -> None:
        try:
            cfg = self._read_config()
            self.derived_label.setText(
                f"reach {cfg.max_reach_mm:.0f} mm  |  "
                f"pulses/deg {cfg.pulses_per_degree:.4f}"
            )
        except ValueError as exc:
            self.derived_label.setText(f"invalid: {exc}")

    # --- actions ------------------------------------------------------------
    def _on_apply(self) -> bool:
        try:
            config = self._read_config()
            limits = self._read_limits()
        except ValueError as exc:
            self._status(f"Invalid geometry: {exc}", ok=False)
            return False

        validator = WorkspaceValidator(FiveBarKinematics(config), limits)
        failures = validate_work_zone(validator)
        if failures:
            x, y, reason = failures[0]
            self._status(
                f"Refused: {len(failures)}/{len(WORK_ZONE)} work-zone points fail "
                f"(e.g. ({x:.0f}, {y:.0f}): {reason})",
                ok=False,
            )
            return False

        self.controller.update_geometry(config, validator)
        self.geometryChanged.emit()
        self._status("Applied — work zone fully valid.", ok=True)
        return True

    def _on_save(self) -> None:
        if self.config_path is None:
            self._status("No config path set; cannot save.", ok=False)
            return
        if not self._on_apply():  # only ever persist validated geometry
            return
        import yaml

        cfg = self._read_config()
        lim = self._read_limits()
        data = {
            "geometry": {
                "l1_mm": cfg.l1_mm,
                "l2_mm": cfg.l2_mm,
                "base_spacing_mm": cfg.base_spacing_mm,
            },
            "assembly": {"left_elbow": cfg.left_elbow, "right_elbow": cfg.right_elbow},
            "joint_limits": {"min_deg": cfg.joint_min_deg, "max_deg": cfg.joint_max_deg},
            "drivetrain": {
                "pulses_per_rev": cfg.pulses_per_rev,
                "belt_reduction": cfg.belt_reduction,
            },
            "singularity": {
                "parallel_min_deg": lim.parallel_min_deg,
                "serial_min_deg": lim.serial_min_deg,
                "reach_fraction_max": lim.reach_fraction_max,
            },
        }
        self.config_path.write_text(yaml.safe_dump(data, sort_keys=False))
        self._status(f"Saved to {self.config_path}.", ok=True)

    def _on_reload(self) -> None:
        if self.config_path is None:
            self._status("No config path set; cannot reload.", ok=False)
            return
        config = FiveBarConfig.from_yaml(self.config_path)
        limits = SingularityLimits.from_yaml(self.config_path)
        self._load_from(config, limits)
        self._status("Reloaded from YAML (not yet applied).", ok=True)

    # --- helpers ------------------------------------------------------------
    def _status(self, text: str, *, ok: bool) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #2e7d32;" if ok else "color: #c62828; font-weight: bold;"
        )
