"""Robot Test tab: establish home + jog the robot.

A thin Qt view over RobotTestController — all validation/kinematics live there.
Buttons call controller methods; the readouts and status line reflect the
resulting (or rejected) pose.
"""

from __future__ import annotations

from typing import Dict

from PySide6.QtCore import Qt, QThread, QTimer, Signal
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

# FaultCode -> operator text (matches docs/plc_program.md §9).
FAULT_TEXT = {
    1: "Drive alarm (EM806 ALM)",
    2: "E-stop / guard open",
    3: "Hard limit tripped",
    4: "Homing failed / timed out",
    5: "Move commanded while not enabled",
    6: "Move commanded while not homed",
    7: "Target outside soft limits",
    8: "Move timed out",
    9: "Vacuum not confirmed",
    10: "Command watchdog / comms loss",
}


class _CommandWorker(QThread):
    """Runs one blocking controller call (home/jog/move) off the GUI thread so a
    10 s command can't freeze the UI. Emits the MoveResult, or the error text."""

    done = Signal(object)
    failed = Signal(str)

    def __init__(self, call, parent=None) -> None:
        super().__init__(parent)
        self._call = call

    def run(self) -> None:
        try:
            self.done.emit(self._call())
        except Exception as exc:  # noqa: BLE001 - surfaced to the status line
            self.failed.emit(str(exc))


class RobotTestTab(QWidget):
    def __init__(self, controller: RobotTestController, parent: QWidget | None = None):
        super().__init__(parent)
        self.controller = controller
        self._value_labels: Dict[str, QLabel] = {}
        self._worker: _CommandWorker | None = None
        self._command_busy = False

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

        # Live poll so async faults (e-stop, drive alarm, hard limit, a homing
        # fault that latches seconds later) surface without the operator clicking.
        self._poll_timer = QTimer(self)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start(300)

    # --- groups -------------------------------------------------------------
    def _build_enable_group(self) -> QGroupBox:
        box = QGroupBox("Drives")
        v = QVBoxLayout(box)
        row = QHBoxLayout()
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.setToolTip("Clear a latched fault so the drives can be enabled.")
        self.reset_btn.clicked.connect(self._on_reset)
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.setCheckable(True)
        self.enable_btn.clicked.connect(self._on_enable_toggled)
        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setProperty("accent", "danger")
        self.stop_btn.clicked.connect(self._on_stop)
        self.referenced_label = QLabel()
        row.addWidget(self.reset_btn)
        row.addWidget(self.enable_btn)
        row.addWidget(self.stop_btn)
        row.addStretch(1)
        row.addWidget(self.referenced_label)
        v.addLayout(row)

        # Persistent fault banner (shown only while a fault is latched) and a
        # next-step hint that sequences the operator: Reset -> Enable -> Home.
        self.fault_banner = QLabel("")
        self.fault_banner.setStyleSheet(
            "color: #f85149; font-weight: bold; padding: 2px 0;"
        )
        self.fault_banner.setVisible(False)
        self.hint_label = QLabel("")
        self.hint_label.setStyleSheet("color: #8b949e;")
        v.addWidget(self.fault_banner)
        v.addWidget(self.hint_label)
        return box

    def _build_home_group(self) -> QGroupBox:
        box = QGroupBox("Home")
        row = QHBoxLayout(box)
        self.ref_btn = QPushButton("Home (find ref)")
        self.ref_btn.clicked.connect(self._on_home_reference)
        set_btn = QPushButton("Set Home (teach)")
        set_btn.clicked.connect(self._on_set_home)
        go_btn = QPushButton("Go Home")
        go_btn.clicked.connect(self._on_go_home)
        self.home_label = QLabel()
        row.addWidget(self.ref_btn)
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
    def _on_reset(self) -> None:
        try:
            res = self.controller.reset()
        except Exception as exc:  # belt-and-suspenders; controller catches already
            self._set_status(f"Reset error: {exc}", ok=False)
        else:
            if res.ok and not self.controller.is_faulted:
                self._set_status("Fault cleared — reset OK.", ok=True)
            elif res.ok:
                self._set_status("Reset sent (fault still active).", ok=False)
            else:
                self._set_status(f"Reset failed: {res.reason}", ok=False)
        self._refresh()
        self._update_enable_state()

    def _on_enable_toggled(self, checked: bool) -> None:
        try:
            res = self.controller.enable() if checked else self.controller.disable()
            if res.ok:
                # message reflects the ACTUAL drive state, not the request
                self._set_status(
                    "Drives enabled." if self.controller.is_enabled else "Drives disabled.",
                    ok=True,
                )
            else:
                self._set_status(
                    f"{'Enable' if checked else 'Disable'} failed: {res.reason}",
                    ok=False,
                )
        except Exception as exc:
            self._set_status(f"{'Enable' if checked else 'Disable'} error: {exc}", ok=False)
        finally:
            # Always re-sync from reality so a failed enable un-highlights the button.
            self._refresh()
            self._update_enable_state()

    def _on_stop(self) -> None:
        try:
            self.controller.stop()
            self.controller.disable()
            self._set_status("STOP — drives disabled.", ok=False)
        except Exception as exc:
            self._set_status(f"STOP error: {exc}", ok=False)
        finally:
            self.enable_btn.setChecked(False)
            self._refresh()
            self._update_enable_state()

    def _on_home_reference(self) -> None:
        self._guarded(lambda: self.controller.home_reference())

    def _on_set_home(self) -> None:
        try:
            x, y = self.controller.set_home()
            self._set_status(f"Home taught at TCP ({x:.1f}, {y:.1f}).", ok=True)
        except Exception as exc:
            self._set_status(f"Set-home error: {exc}", ok=False)
        self._refresh()
        self._update_enable_state()

    def _on_go_home(self) -> None:
        self._guarded(lambda: self.controller.go_home())

    def _jog_joint(self, joint: str, sign: int) -> None:
        self._guarded(lambda: self.controller.jog_joint(joint, sign * self.joint_step.value()))

    def _jog_cart(self, axis: str, sign: int) -> None:
        self._guarded(lambda: self.controller.jog_cartesian(axis, sign * self.cart_step.value()))

    def _guarded(self, call) -> None:
        """Run a controller call that returns a MoveResult on a worker thread so a
        blocking command (home/jog can wait up to command_timeout_s) never freezes
        the GUI. The outcome is rendered when the worker finishes."""
        if self._worker is not None and self._worker.isRunning():
            self._set_status("Busy — a command is already running.", ok=False)
            return
        self._set_command_busy(True)
        self._worker = _CommandWorker(call, self)
        self._worker.done.connect(self._on_command_done)
        self._worker.failed.connect(self._on_command_failed)
        self._worker.finished.connect(self._on_command_finished)
        self._worker.start()

    def _on_command_done(self, result) -> None:
        self._apply(result)

    def _on_command_failed(self, msg: str) -> None:
        self._set_status(f"Error: {msg}", ok=False)
        self._refresh()
        self._update_enable_state()

    def _on_command_finished(self) -> None:
        self._worker = None
        self._set_command_busy(False)

    def _set_command_busy(self, busy: bool) -> None:
        self._command_busy = busy
        if busy:
            self._set_status("Working…", ok=True)
        self._update_enable_state()

    def _await_command(self, timeout_ms: int = 5000) -> None:
        """Test/teardown helper: block until the running command worker finishes
        and its result signal has been delivered. Not used in normal GUI flow."""
        if self._worker is not None:
            self._worker.wait(timeout_ms)
        from PySide6.QtWidgets import QApplication
        app = QApplication.instance()
        if app is not None:
            app.processEvents()

    # --- view update --------------------------------------------------------
    def refresh_all(self) -> None:
        """Re-read everything from the controller (e.g. after a geometry change)."""
        self._refresh()
        self._update_enable_state()

    def _apply(self, result: MoveResult) -> None:
        if result.ok:
            self._set_status("Move OK.", ok=True)
        else:
            self._set_status(f"Rejected: {result.reason}", ok=False)
        self._refresh()
        self._update_enable_state()

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
        referenced = self.controller.is_referenced
        self.referenced_label.setText("REFERENCED" if referenced else "NOT REFERENCED")
        self.referenced_label.setStyleSheet(
            "color: #3fb950; font-weight: bold;"
            if referenced
            else "color: #f85149; font-weight: bold;"
        )

    def _update_enable_state(self) -> None:
        faulted = self.controller.is_faulted
        enabled = self.controller.is_enabled
        referenced = self.controller.is_referenced
        busy = self._command_busy
        self.enable_btn.setText("Enabled" if enabled else "Enable")
        self.enable_btn.setChecked(enabled)  # keep the toggle in sync with reality
        # While faulted, only Reset is live — the fault must be cleared first.
        # While a command runs, freeze the command buttons (STOP stays live).
        self.reset_btn.setEnabled(faulted and not busy)
        self.enable_btn.setEnabled(not faulted and not busy)
        self.ref_btn.setEnabled(enabled and not faulted and not busy)
        # Jogging needs enabled drives, a found home reference, and no fault.
        for btn in getattr(self, "_jog_buttons", []):
            btn.setEnabled(enabled and referenced and not faulted and not busy)

        # Keep the REFERENCED label live off the poll too (not just after a
        # command) so a disable / drive power-cycle that drops the reference
        # updates the readout immediately, not only after the next click.
        self.referenced_label.setText("REFERENCED" if referenced else "NOT REFERENCED")
        self.referenced_label.setStyleSheet(
            "color: #3fb950; font-weight: bold;"
            if referenced
            else "color: #f85149; font-weight: bold;"
        )

        # Fault banner + next-step sequencing hint.
        if faulted:
            code = self.controller.fault_code()
            text = FAULT_TEXT.get(code or 0, "unknown fault")
            self.fault_banner.setText(f"FAULT {code} — {text}. Press Reset.")
            self.fault_banner.setVisible(True)
            self.hint_label.setText("Fault active — press Reset.")
        else:
            self.fault_banner.setVisible(False)
            if not enabled:
                self.hint_label.setText("Next: Enable the drives.")
            elif not referenced:
                self.hint_label.setText("Next: Home (find ref).")
            else:
                self.hint_label.setText("Ready — jog or run.")

    def _poll(self) -> None:
        """Live status tick — surfaces async faults (e-stop, drive alarm, hard
        limit) and keeps the button/banner state in sync with the PLC."""
        try:
            self._update_enable_state()
        except Exception:  # a status read hiccup must never kill the timer
            pass

    def _set_status(self, text: str, *, ok: bool) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #3fb950;" if ok else "color: #f85149; font-weight: bold;"
        )
