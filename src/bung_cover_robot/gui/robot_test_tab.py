"""Robot Test tab: establish home + jog the robot.

A thin Qt view over RobotTestController — all validation/kinematics live there.
Buttons call controller methods; the readouts and status line reflect the
resulting (or rejected) pose.
"""

from __future__ import annotations

from typing import Dict

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import MoveResult, RobotTestController


class RobotTestTab(QWidget):
    def __init__(self, controller: RobotTestController, parent: QWidget | None = None):
        super().__init__(parent)
        self.controller = controller
        self._value_labels: Dict[str, QLabel] = {}

        root = QVBoxLayout(self)
        root.addWidget(self._build_enable_group())
        root.addWidget(self._build_home_group())
        root.addWidget(self._build_jog_group())
        root.addWidget(self._build_readout_group())
        self.status_label = QLabel("Ready.")
        self.status_label.setObjectName("statusLabel")
        root.addWidget(self.status_label)
        root.addStretch(1)

        self._refresh()
        self._update_enable_state()

    # --- groups -------------------------------------------------------------
    def _build_enable_group(self) -> QGroupBox:
        box = QGroupBox("Drives")
        row = QHBoxLayout(box)
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setCheckable(True)
        self.enable_btn.clicked.connect(self._on_enable_toggled)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.clicked.connect(self._on_stop)
        self.homed_label = QLabel()
        row.addWidget(self.enable_btn)
        row.addWidget(self.stop_btn)
        row.addStretch(1)
        row.addWidget(self.homed_label)
        return box

    def _build_home_group(self) -> QGroupBox:
        box = QGroupBox("Home")
        row = QHBoxLayout(box)
        set_btn = QPushButton("Set Home (teach)")
        set_btn.clicked.connect(self._on_set_home)
        go_btn = QPushButton("Go Home")
        go_btn.clicked.connect(self._on_go_home)
        self.home_label = QLabel()
        row.addWidget(set_btn)
        row.addWidget(go_btn)
        row.addStretch(1)
        row.addWidget(self.home_label)
        return box

    def _build_jog_group(self) -> QGroupBox:
        box = QGroupBox("Jog")
        grid = QGridLayout(box)

        # Step sizes.
        grid.addWidget(QLabel("Joint step (deg):"), 0, 0)
        self.joint_step = QDoubleSpinBox()
        self.joint_step.setRange(0.1, 30.0)
        self.joint_step.setSingleStep(0.5)
        self.joint_step.setValue(1.0)
        grid.addWidget(self.joint_step, 0, 1)

        grid.addWidget(QLabel("Cartesian step (mm):"), 0, 2)
        self.cart_step = QDoubleSpinBox()
        self.cart_step.setRange(0.1, 50.0)
        self.cart_step.setSingleStep(1.0)
        self.cart_step.setValue(5.0)
        grid.addWidget(self.cart_step, 0, 3)

        # Joint jog row.
        grid.addWidget(QLabel("Left shoulder"), 1, 0)
        grid.addWidget(self._jog_btn("L −", lambda: self._jog_joint("left", -1)), 1, 1)
        grid.addWidget(self._jog_btn("L +", lambda: self._jog_joint("left", +1)), 1, 2)
        grid.addWidget(QLabel("Right shoulder"), 2, 0)
        grid.addWidget(self._jog_btn("R −", lambda: self._jog_joint("right", -1)), 2, 1)
        grid.addWidget(self._jog_btn("R +", lambda: self._jog_joint("right", +1)), 2, 2)

        # Cartesian jog row (robot frame: X along conveyor, Y across / reach).
        grid.addWidget(QLabel("TCP X (along)"), 3, 0)
        grid.addWidget(self._jog_btn("X −", lambda: self._jog_cart("x", -1)), 3, 1)
        grid.addWidget(self._jog_btn("X +", lambda: self._jog_cart("x", +1)), 3, 2)
        grid.addWidget(QLabel("TCP Y (reach)"), 4, 0)
        grid.addWidget(self._jog_btn("Y −", lambda: self._jog_cart("y", -1)), 4, 1)
        grid.addWidget(self._jog_btn("Y +", lambda: self._jog_cart("y", +1)), 4, 2)

        self._jog_buttons = [
            grid.itemAt(i).widget()
            for i in range(grid.count())
            if isinstance(grid.itemAt(i).widget(), QPushButton)
        ]
        return box

    def _build_readout_group(self) -> QGroupBox:
        box = QGroupBox("Position / workspace")
        grid = QGridLayout(box)
        fields = [
            ("tcp_x", "TCP X (mm)"),
            ("tcp_y", "TCP Y (mm)"),
            ("left_deg", "Left shoulder (deg)"),
            ("right_deg", "Right shoulder (deg)"),
            ("left_pulses", "Left pulses"),
            ("right_pulses", "Right pulses"),
            ("parallel", "Parallel margin (deg)"),
            ("serial", "Serial margin (deg)"),
            ("reach", "Reach fraction"),
        ]
        for i, (key, label) in enumerate(fields):
            r, c = divmod(i, 3)
            cell = QVBoxLayout()
            name = QLabel(label)
            value = QLabel("—")
            value.setAlignment(Qt.AlignmentFlag.AlignLeft)
            value.setStyleSheet("font-weight: bold;")
            self._value_labels[key] = value
            cell.addWidget(name)
            cell.addWidget(value)
            wrapper = QWidget()
            wrapper.setLayout(cell)
            wrapper.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            grid.addWidget(wrapper, r, c)
        return box

    # --- widget helpers -----------------------------------------------------
    def _jog_btn(self, text: str, slot) -> QPushButton:
        btn = QPushButton(text)
        btn.clicked.connect(slot)
        return btn

    # --- handlers -----------------------------------------------------------
    def _on_enable_toggled(self, checked: bool) -> None:
        if checked:
            self.controller.enable()
            self._set_status("Drives enabled.", ok=True)
        else:
            self.controller.disable()
            self._set_status("Drives disabled.", ok=True)
        self._update_enable_state()

    def _on_stop(self) -> None:
        self.controller.stop()
        self.controller.disable()
        self.enable_btn.setChecked(False)
        self._set_status("STOP — drives disabled.", ok=False)
        self._update_enable_state()

    def _on_set_home(self) -> None:
        x, y = self.controller.set_home()
        self._set_status(f"Home taught at TCP ({x:.1f}, {y:.1f}).", ok=True)
        self._refresh()

    def _on_go_home(self) -> None:
        self._apply(self.controller.go_home())

    def _jog_joint(self, joint: str, sign: int) -> None:
        self._apply(self.controller.jog_joint(joint, sign * self.joint_step.value()))

    def _jog_cart(self, axis: str, sign: int) -> None:
        self._apply(self.controller.jog_cartesian(axis, sign * self.cart_step.value()))

    # --- view update --------------------------------------------------------
    def _apply(self, result: MoveResult) -> None:
        if result.ok:
            self._set_status("Move OK.", ok=True)
        else:
            self._set_status(f"Rejected: {result.reason}", ok=False)
        self._refresh()

    def _refresh(self) -> None:
        s = self.controller.state
        m = s.metrics
        self._value_labels["tcp_x"].setText(f"{s.tcp[0]:.2f}")
        self._value_labels["tcp_y"].setText(f"{s.tcp[1]:.2f}")
        self._value_labels["left_deg"].setText(f"{s.left_deg:.2f}")
        self._value_labels["right_deg"].setText(f"{s.right_deg:.2f}")
        self._value_labels["left_pulses"].setText(str(s.left_pulses))
        self._value_labels["right_pulses"].setText(str(s.right_pulses))
        self._value_labels["parallel"].setText(
            f"{m.get('parallel_margin_deg', float('nan')):.1f}"
        )
        self._value_labels["serial"].setText(
            f"{m.get('serial_margin_deg', float('nan')):.1f}"
        )
        self._value_labels["reach"].setText(f"{m.get('reach_fraction', float('nan')):.3f}")

        hx, hy = self.controller.home_xy
        self.home_label.setText(f"Home: ({hx:.1f}, {hy:.1f})")
        self.homed_label.setText("HOMED" if self.controller.is_homed else "NOT HOMED")
        self.homed_label.setStyleSheet(
            "color: #2e7d32; font-weight: bold;"
            if self.controller.is_homed
            else "color: #c62828; font-weight: bold;"
        )

    def _update_enable_state(self) -> None:
        enabled = self.controller.is_enabled
        self.enable_btn.setText("Enabled" if enabled else "Enable")
        for btn in getattr(self, "_jog_buttons", []):
            btn.setEnabled(enabled)

    def _set_status(self, text: str, *, ok: bool) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #2e7d32;" if ok else "color: #c62828; font-weight: bold;"
        )
