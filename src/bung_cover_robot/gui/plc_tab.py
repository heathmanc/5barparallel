"""PLC tab: live connection controls + the tag contract.

Top: connect/disconnect the motion driver at runtime — dry-run (in-process),
a simulated PLC (real handshake, no hardware), or a real CompactLogix over
EtherNet/IP. Connecting hot-swaps the controller's driver (RobotTestController.
set_driver) and emits connectionChanged so the Robot Test tab refreshes.

Bottom: a read-only table of every tag the Studio 5000 program must implement,
generated from plc.tags.TAG_SPECS so it can't drift from the code.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController
from ..plc import (
    CompactLogixClient,
    PlcError,
    PlcRobotDriver,
    SimulatedPlcClient,
)
from ..plc import tags as T
from ..robot.driver import DryRunRobotDriver

_COLUMNS = ["Group", "Tag", "Type", "Direction", "Description"]
_CMD_TINT = QColor("#e8f0fe")     # PC → PLC
_STATUS_TINT = QColor("#e9f7ec")  # PLC → PC


class PlcTab(QWidget):
    connectionChanged = Signal()

    def __init__(
        self,
        controller: RobotTestController,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller

        root = QVBoxLayout(self)
        root.addWidget(self._build_connection_group())
        root.addWidget(self._build_contract_header())
        self.table = self._build_table(T.TAG_SPECS)
        root.addWidget(self.table)

        row = QHBoxLayout()
        export = QPushButton("Export CSV…")
        export.clicked.connect(self._on_export)
        row.addStretch(1)
        row.addWidget(export)
        root.addLayout(row)

        self._refresh_status()

    # --- connection ---------------------------------------------------------
    def _build_connection_group(self) -> QGroupBox:
        box = QGroupBox("Connection")
        outer = QVBoxLayout(box)

        self.status_label = QLabel()
        outer.addWidget(self.status_label)

        row = QHBoxLayout()
        row.addWidget(QLabel("IP/slot:"))
        self.path_edit = QLineEdit()
        self.path_edit.setPlaceholderText("192.168.1.10/0")
        row.addWidget(self.path_edit)
        connect_btn = QPushButton("Connect PLC")
        connect_btn.clicked.connect(self._on_connect_real)
        sim_btn = QPushButton("Simulated PLC")
        sim_btn.clicked.connect(self._on_connect_sim)
        disconnect_btn = QPushButton("Disconnect")
        disconnect_btn.clicked.connect(self._on_disconnect)
        for b in (connect_btn, sim_btn, disconnect_btn):
            row.addWidget(b)
        outer.addLayout(row)
        return box

    def _on_connect_real(self) -> None:
        path = self.path_edit.text().strip()
        if not path:
            self._set_status("Enter an IP/slot (e.g. 192.168.1.10/0).", ok=False)
            return
        driver = PlcRobotDriver(CompactLogixClient(path))
        try:
            driver.connect()
        except PlcError as exc:
            self._set_status(f"Connect failed: {exc}", ok=False)
            return
        self.controller.set_driver(driver)
        self.connectionChanged.emit()
        self._refresh_status()

    def _on_connect_sim(self) -> None:
        client = SimulatedPlcClient(home_angles=self._home_angles()).connect()
        self.controller.set_driver(PlcRobotDriver(client))
        self.connectionChanged.emit()
        self._refresh_status()

    def _on_disconnect(self) -> None:
        self.controller.set_driver(DryRunRobotDriver(home_angles=self._home_angles()))
        self.connectionChanged.emit()
        self._refresh_status()

    def _home_angles(self):
        jt = self.controller.kin.inverse(*self.controller.home_xy)
        return (jt.left_deg, jt.right_deg)

    def _refresh_status(self) -> None:
        self._set_status(f"Active driver: {self._describe_driver()}", ok=True)

    def _describe_driver(self) -> str:
        driver = self.controller.driver
        if isinstance(driver, DryRunRobotDriver):
            return "dry-run (in-process simulation)"
        if isinstance(driver, PlcRobotDriver):
            client = driver.client
            if isinstance(client, SimulatedPlcClient):
                return "simulated PLC (real handshake, no hardware)"
            if isinstance(client, CompactLogixClient):
                return f"CompactLogix @ {client.path}"
            return "PLC"
        return type(driver).__name__

    def _set_status(self, text: str, *, ok: bool) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #2e7d32; font-weight: bold;"
            if ok
            else "color: #c62828; font-weight: bold;"
        )

    # --- tag contract -------------------------------------------------------
    def _build_contract_header(self) -> QLabel:
        specs = T.TAG_SPECS
        n_cmd = sum(1 for s in specs if s.direction == T.PC_TO_PLC)
        n_status = len(specs) - n_cmd
        return QLabel(
            "PLC tag contract — the Studio 5000 program must implement every tag "
            "below (base name <b>VisionRobot</b>).\n"
            f"{len(specs)} tags: {n_cmd} commands/targets (PC → PLC, Python writes) "
            f"and {n_status} status (PLC → PC, Python reads)."
        )

    def _build_table(self, specs) -> QTableWidget:
        table = QTableWidget(len(specs), len(_COLUMNS))
        table.setHorizontalHeaderLabels(_COLUMNS)
        table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        table.verticalHeader().setVisible(False)
        table.setAlternatingRowColors(True)

        for r, s in enumerate(specs):
            tint = _CMD_TINT if s.direction == T.PC_TO_PLC else _STATUS_TINT
            for c, value in enumerate(
                (s.group, s.name, s.dtype, s.direction, s.description)
            ):
                item = QTableWidgetItem(value)
                item.setBackground(tint)
                table.setItem(r, c, item)

        header = table.horizontalHeader()
        for c in range(len(_COLUMNS) - 1):
            header.setSectionResizeMode(c, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(len(_COLUMNS) - 1, QHeaderView.ResizeMode.Stretch)
        return table

    def _on_export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export PLC tag list", "plc_tags.csv", "CSV files (*.csv)"
        )
        if path:
            with open(path, "w", newline="") as fh:
                fh.write(T.tag_table_csv())
