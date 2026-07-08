"""Diagnostics tab — one screen with every tag you'd otherwise hunt for in
Studio 5000: controller status, homing state machine, the raw ClearLink motor
status bits, and the tuning constants.

PLC reads run on a background QThread (``PlcPoller``) and arrive via a signal, so
the GUI never blocks on network I/O — the whole point being that watching this
screen can't freeze the app. Values are batch-read in one round trip.
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController

# kind: "bool" green/gray · "fault" red-when-true · "num" plain value
Field = Tuple[str, str, str]  # (label, tag, kind)

FAULT_CODES = {
    0: "none", 1: "drive alarm", 2: "e-stop/guard", 3: "hard limit",
    4: "homing fail/timeout", 5: "move-while-disabled", 6: "move-while-unhomed",
    7: "soft limits", 9: "vacuum",
}


def _clearlink_status(m: int) -> List[Field]:
    i = f"ClearLink:I1.Motor{m}_"
    return [
        (f"M{m} Enabled (b10)", f"{i}Status_Enabled", "bool"),
        (f"M{m} HLFB_ON (b14)", f"{i}Status_HLFB_ON", "bool"),
        (f"M{m} Motor_In_Fault (b9)", f"{i}Status_Motor_In_Fault", "fault"),
        (f"M{m} Shutdowns_Pres (b17)", f"{i}Status_Shutdowns_Pres", "fault"),
        (f"M{m} Ready_To_Home (b16)", f"{i}Status_Ready_To_Home", "bool"),
        (f"M{m} At_Target_Posn (b0)", f"{i}Status_At_Target_Posn", "bool"),
        (f"M{m} Steps_Active (b1)", f"{i}Status_Steps_Active", "bool"),
        (f"M{m} Has_Homed (b13)", f"{i}Status_Has_Homed", "bool"),
        (f"M{m} In_Home_Sensor (b7)", f"{i}Status_In_Home_Sensor", "bool"),
        (f"M{m} LoadPosnAck (b19)", f"{i}Status_Load_Posn_Move_Ack", "bool"),
        (f"M{m} LoadVelAck (b20)", f"{i}Status_Load_Vel_Move_Ack", "bool"),
        (f"M{m} CommandedPosn", f"{i}CommandedPosn", "num"),
    ]


GROUPS: List[Tuple[str, List[Field]]] = [
    ("Controller status", [
        ("Enabled", "VisionRobot.Status.Enabled", "bool"),
        ("Homed", "VisionRobot.Status.Homed", "bool"),
        ("Faulted", "VisionRobot.Status.Faulted", "fault"),
        ("FaultCode", "VisionRobot.Status.FaultCode", "faultcode"),
        ("InPosition", "VisionRobot.Status.InPosition", "bool"),
        ("ActualLeftDeg", "VisionRobot.Status.ActualLeftDeg", "num"),
        ("ActualRightDeg", "VisionRobot.Status.ActualRightDeg", "num"),
        ("SafetyOK", "SafetyOK", "bool"),
    ]),
    ("Homing state machine", [
        ("HomeStep (R30)", "HomeStep", "num"),
        ("Home0_State", "Home0_State", "num"),
        ("Home1_State", "Home1_State", "num"),
        ("Ax0_HomeDone", "Ax0_HomeDone", "bool"),
        ("Ax1_HomeDone", "Ax1_HomeDone", "bool"),
        ("Ax0_HomeFault", "Ax0_HomeFault", "fault"),
        ("Ax1_HomeFault", "Ax1_HomeFault", "fault"),
        ("Home0_Moved", "Home0_Moved", "bool"),
        ("Home1_Moved", "Home1_Moved", "bool"),
        ("Bypass_Homing", "Bypass_Homing", "bool"),
    ]),
    ("Move engine", [
        ("Move0_Execute", "Move0_Execute", "bool"),
        ("Move0_Loaded", "Move0_Loaded", "bool"),
        ("Move0_InPosition", "Move0_InPosition", "bool"),
        ("Move0_Fault", "Move0_Fault", "fault"),
        ("Move1_Execute", "Move1_Execute", "bool"),
        ("Move1_Loaded", "Move1_Loaded", "bool"),
        ("Move1_InPosition", "Move1_InPosition", "bool"),
        ("Move1_Fault", "Move1_Fault", "fault"),
    ]),
    ("Tuning constants", [
        ("MOVE_VEL", "MOVE_VEL", "num"),
        ("MOVE_ACC", "MOVE_ACC", "num"),
        ("HOME_VEL_0", "HOME_VEL_0", "numwarn0"),
        ("HOME_VEL_1", "HOME_VEL_1", "numwarn0"),
        ("HOME_ACC", "HOME_ACC", "num"),
        ("STEPS_PER_DEG", "STEPS_PER_DEG", "num"),
    ]),
    ("ClearLink Motor 0", _clearlink_status(0)),
    ("ClearLink Motor 1", _clearlink_status(1)),
]

_ALL_TAGS = [tag for _, fields in GROUPS for _, tag, _ in fields]


class PlcPoller(QThread):
    """Batch-reads a fixed tag list off the GUI thread and emits the results."""

    polled = Signal(dict)
    failed = Signal(str)

    def __init__(self, client_getter: Callable, tags: List[str],
                 interval_ms: int = 400, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._client_getter = client_getter
        self._tags = tags
        self._interval_ms = interval_ms
        self._stop = False

    def stop(self) -> None:
        self._stop = True
        self.wait(2000)

    def run(self) -> None:  # pragma: no cover - thread loop
        while not self._stop:
            client = self._client_getter()
            if client is not None:
                try:
                    self.polled.emit(client.read_many(self._tags))
                except Exception as exc:  # noqa: BLE001 - report, keep polling
                    self.failed.emit(str(exc))
            self.msleep(self._interval_ms)


class DiagnosticsTab(QWidget):
    def __init__(self, controller: RobotTestController,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.controller = controller
        self._value_labels: Dict[str, Tuple[QLabel, str]] = {}
        self._poller: Optional[PlcPoller] = None

        root = QVBoxLayout(self)
        header = QHBoxLayout()
        header.addWidget(QLabel("<b>Live PLC diagnostics</b> — polled off the GUI thread."))
        header.addStretch(1)
        self.status_label = QLabel()
        header.addWidget(self.status_label)
        root.addLayout(header)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        inner = QWidget()
        grid = QGridLayout(inner)
        col = 0
        for title, fields in GROUPS:
            grid.addWidget(self._build_group(title, fields), col // 2, col % 2)
            col += 1
        scroll.setWidget(inner)
        root.addWidget(scroll, 1)

        self.refresh()

    def _build_group(self, title: str, fields: List[Field]) -> QGroupBox:
        box = QGroupBox(title)
        g = QGridLayout(box)
        for row, (label, tag, kind) in enumerate(fields):
            g.addWidget(QLabel(label), row, 0)
            val = QLabel("—")
            val.setStyleSheet("font-family: monospace;")
            g.addWidget(val, row, 1)
            self._value_labels[tag] = (val, kind)
        return box

    # --- polling lifecycle --------------------------------------------------
    def _client(self):
        return getattr(self.controller.driver, "client", None)

    def refresh(self) -> None:
        """Start/stop the poller to match the PLC connection + tab visibility
        (call on connectionChanged, show, hide)."""
        connected = self._client() is not None and self.isVisible()
        if connected and self._poller is None:
            self._poller = PlcPoller(self._client, _ALL_TAGS, parent=self)
            self._poller.polled.connect(self._on_polled)
            self._poller.failed.connect(self._on_failed)
            self._poller.start()
            self.status_label.setText("<span style='color:#3fb950'>polling</span>")
        elif not connected:
            self._stop_poller()
            self.status_label.setText(
                "<span style='color:#8b949e'>no PLC — connect on the PLC tab</span>")
            for tag, (lbl, _kind) in self._value_labels.items():
                lbl.setText("—")
                lbl.setStyleSheet("font-family: monospace; color:#8b949e;")

    def _stop_poller(self) -> None:
        if self._poller is not None:
            self._poller.stop()
            self._poller = None

    def _on_failed(self, msg: str) -> None:
        self.status_label.setText(f"<span style='color:#f85149'>read error: {msg}</span>")

    def _on_polled(self, values: Dict) -> None:
        self.status_label.setText("<span style='color:#3fb950'>polling</span>")
        for tag, (lbl, kind) in self._value_labels.items():
            lbl.setText(self._format(values.get(tag), kind))
            lbl.setStyleSheet("font-family: monospace; " + self._color(values.get(tag), kind))

    # --- formatting ---------------------------------------------------------
    @staticmethod
    def _format(value, kind: str) -> str:
        if value is None:
            return "—"
        if kind in ("bool", "fault"):
            return "1" if value else "0"
        if kind == "faultcode":
            try:
                code = int(value)
            except (TypeError, ValueError):
                return str(value)
            return f"{code} ({FAULT_CODES.get(code, '?')})"
        if isinstance(value, float):
            return f"{value:.3f}"
        return str(value)

    @staticmethod
    def _color(value, kind: str) -> str:
        if value is None:
            return "color:#8b949e;"
        if kind == "bool":
            return "color:#3fb950;" if value else "color:#8b949e;"
        if kind == "fault":
            return "color:#f85149; font-weight:bold;" if value else "color:#8b949e;"
        if kind == "faultcode":
            try:
                return "color:#f85149; font-weight:bold;" if int(value) else "color:#3fb950;"
            except (TypeError, ValueError):
                return ""
        if kind == "numwarn0":
            # HOME_VEL etc: zero is almost always "forgot to set it after import".
            try:
                return "color:#f85149; font-weight:bold;" if int(value) == 0 else ""
            except (TypeError, ValueError):
                return ""
        return ""

    def showEvent(self, event) -> None:  # pragma: no cover - Qt visibility
        super().showEvent(event)
        self.refresh()

    def hideEvent(self, event) -> None:  # pragma: no cover - Qt visibility
        self._stop_poller()
        super().hideEvent(event)

    def closeEvent(self, event) -> None:  # pragma: no cover - Qt teardown
        self._stop_poller()
        super().closeEvent(event)
