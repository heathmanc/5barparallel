"""Settings tab: view/edit robot geometry, home, and workspace guard thresholds.

Geometry (L1, L2, base spacing, joint limits, branch, drivetrain) and the home
TCP are editable. Apply auto-derives the largest safe work area the geometry
supports (largest reachable + singularity-clear rectangle) — so a smaller robot
yields a smaller area instead of being refused — recomputes the home shoulder
angles, and refuses only if the geometry has no safe area or the chosen home is
unreachable/unsafe. Passing geometry is pushed to the live controller; Save then
writes geometry + the recomputed home back to config/robot_config.yaml.

Changing link lengths/base spacing (or the home) changes the home shoulder
angles, so HOME_ANGLE_L/R and HOME_OFFSET on the PLC must be updated + re-
calibrated afterward. The geometry must also match the physically-built arms —
this tab changes only what the kinematics believe, not the hardware.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

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
from . import theme
from ..robot.fivebar_kinematics import FiveBarConfig, FiveBarKinematics, KinematicsError
from ..robot.workspace import (
    SingularityLimits,
    WorkArea,
    WorkspaceValidator,
    largest_safe_rectangle,
)


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
        self._work_area: Optional[WorkArea] = None   # last auto-derived safe area

        root = QVBoxLayout(self)
        root.addWidget(self._build_geometry_group())
        root.addWidget(self._build_home_group())
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

    def _build_home_group(self) -> QGroupBox:
        box = QGroupBox("Home reference (recomputed on Apply)")
        form = QFormLayout(box)
        form.addRow("Home TCP X (mm)", self._fspin("home_x", -600, 600, 1, 1))
        form.addRow("Home TCP Y (mm)", self._fspin("home_y", 0, 600, 1, 1))
        self.home_angles_label = QLabel()
        self.home_angles_label.setWordWrap(True)
        form.addRow("Home angles", self.home_angles_label)
        for key in ("home_x", "home_y"):
            self._floats[key].valueChanged.connect(self._update_derived)
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
        apply_btn.setProperty("accent", "primary")
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
        hx, hy = self.controller.home_xy
        self._floats["home_x"].setValue(hx)
        self._floats["home_y"].setValue(hy)
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
        except ValueError as exc:
            self.derived_label.setText(f"invalid: {exc}")
            return
        self.derived_label.setText(
            f"reach {cfg.max_reach_mm:.0f} mm  |  pulses/deg {cfg.pulses_per_degree:.4f}"
        )
        # Live home-angle preview from the current (unapplied) fields.
        try:
            jt = FiveBarKinematics(cfg).inverse(
                self._floats["home_x"].value(), self._floats["home_y"].value())
            self.home_angles_label.setText(
                f"L {jt.left_deg:.2f}°   R {jt.right_deg:.2f}°")
            self.home_angles_label.setStyleSheet("")
        except (KinematicsError, ValueError):
            self.home_angles_label.setText("home unreachable with this geometry")
            self.home_angles_label.setStyleSheet(f"color:{theme.WARN};")

    # --- actions ------------------------------------------------------------
    def _on_apply(self) -> bool:
        try:
            config = self._read_config()
            limits = self._read_limits()
            home_x = self._floats["home_x"].value()
            home_y = self._floats["home_y"].value()
        except ValueError as exc:
            self._status(f"Invalid geometry: {exc}", ok=False)
            return False

        validator = WorkspaceValidator(FiveBarKinematics(config), limits)
        # Auto-derive the usable work area from the geometry (largest safe rectangle).
        area = largest_safe_rectangle(validator)
        if area is None:
            self._status(
                "Refused: this geometry has no safe work area (check link lengths "
                "and the singularity thresholds).", ok=False)
            return False
        # The home must sit inside the safe area, or the robot can't reference there.
        if not validator.is_safe(home_x, home_y):
            reason = validator.validate(home_x, home_y).reason
            self._status(
                f"Refused: home ({home_x:.0f}, {home_y:.0f}) mm is not safe: {reason}. "
                f"Set the home inside the derived area (x [{area.x_min:.0f}, "
                f"{area.x_max:.0f}], y [{area.y_min:.0f}, {area.y_max:.0f}]).",
                ok=False)
            return False

        self.controller.update_geometry(config, validator, home_xy=(home_x, home_y))
        self._work_area = area
        jt = self.controller.kin.inverse(home_x, home_y)
        self.geometryChanged.emit()
        self._update_derived()
        self._status(
            f"Applied. Work area {area.width:.0f} × {area.height:.0f} mm "
            f"(x [{area.x_min:.0f}, {area.x_max:.0f}], y [{area.y_min:.0f}, "
            f"{area.y_max:.0f}]).  Home ({home_x:.0f}, {home_y:.0f}) → "
            f"L {jt.left_deg:.2f}°  R {jt.right_deg:.2f}°.  "
            f"⚠ Update HOME_ANGLE_L/R and RE-CALIBRATE HOME_OFFSET on the PLC.",
            ok=True)
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
        # Preserve any other sections (e.g. homing) already in the file.
        data = {}
        if self.config_path.exists():
            data = yaml.safe_load(self.config_path.read_text()) or {}
        data["geometry"] = {
            "l1_mm": cfg.l1_mm,
            "l2_mm": cfg.l2_mm,
            "base_spacing_mm": cfg.base_spacing_mm,
        }
        data["assembly"] = {"left_elbow": cfg.left_elbow, "right_elbow": cfg.right_elbow}
        data["joint_limits"] = {"min_deg": cfg.joint_min_deg, "max_deg": cfg.joint_max_deg}
        data["drivetrain"] = {
            "pulses_per_rev": cfg.pulses_per_rev,
            "belt_reduction": cfg.belt_reduction,
        }
        data["singularity"] = {
            "parallel_min_deg": lim.parallel_min_deg,
            "serial_min_deg": lim.serial_min_deg,
            "reach_fraction_max": lim.reach_fraction_max,
        }
        # Persist the (recomputed) home into the homing block, preserving its other
        # keys (flag radius, joint limits) so the homing reference stays consistent.
        hx, hy = self.controller.home_xy
        jt = self.controller.kin.inverse(hx, hy)
        homing = dict(data.get("homing", {}) or {})
        homing["home_tcp_mm"] = [round(hx, 3), round(hy, 3)]
        homing["home_left_deg"] = round(jt.left_deg, 4)
        homing["home_right_deg"] = round(jt.right_deg, 4)
        data["homing"] = homing
        self.config_path.write_text(yaml.safe_dump(data, sort_keys=False))
        self._status(
            f"Saved to {self.config_path}. Home angles → L {jt.left_deg:.2f}° "
            f"R {jt.right_deg:.2f}°; re-calibrate HOME_OFFSET on the PLC.", ok=True)

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
            f"color: {theme.TEXT};" if ok else f"color: {theme.WARN}; font-weight: bold;"
        )
