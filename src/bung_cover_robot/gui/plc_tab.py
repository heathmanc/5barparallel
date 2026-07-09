"""PLC tab: live connection controls + the tag contract.

Top: connect/disconnect the motion driver at runtime — dry-run (in-process),
a simulated PLC (real handshake, no hardware), or a real CompactLogix over
EtherNet/IP. Connecting hot-swaps the controller's driver (RobotTestController.
set_driver) and emits connectionChanged so the Robot Test tab refreshes.

Bottom: a read-only table of every tag the Studio 5000 program must implement,
generated from plc.tags.TAG_SPECS so it can't drift from the code.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QDoubleSpinBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController
from ..plc import (
    COMMISSIONING_CONSTANTS,
    CompactLogixClient,
    PlcConstantStore,
    PlcError,
    PlcRobotDriver,
    SimulatedPlcClient,
    push_constants,
    read_constants,
)
from ..plc import tags as T
from ..robot import KinematicsError, solve_home_offsets
from ..robot.driver import DryRunRobotDriver

# ClearLink commanded-position tags (steps, zeroed at the prox trip point).
_CMD_POSN = {
    "HOME_OFFSET_L": "ClearLink:I1.Motor0_CommandedPosn",   # Motor0 = left
    "HOME_OFFSET_R": "ClearLink:I1.Motor1_CommandedPosn",   # Motor1 = right
}

_COLUMNS = ["Group", "Tag", "Type", "Direction", "Description"]
_CMD_TINT = QColor("#1e2b3d")     # PC → PLC
_STATUS_TINT = QColor("#1d2e22")  # PLC → PC
_OK_TINT = QColor("#1d2e22")
_FAIL_TINT = QColor("#3d1e22")
# Operator-specific machine values (HOME_OFFSET, tuned speeds, ...); git-ignored.
_CONST_PATH = Path(__file__).resolve().parents[3] / "config" / "plc_constants.yaml"


class PlcTab(QWidget):
    connectionChanged = Signal()

    def __init__(
        self,
        controller: RobotTestController,
        settings=None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.settings = settings          # AppSettings: remembers the PLC IP/slot
        self.const_store = PlcConstantStore.load(_CONST_PATH)

        root = QVBoxLayout(self)
        root.addWidget(self._build_connection_group())
        root.addWidget(self._build_constants_group())
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
        if self.settings is not None:                # restore the last-used IP/slot
            self.path_edit.setText(str(self.settings.get("plc_ip", "")))
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
        if self.settings is not None:                # remember it for next launch
            self.settings.set("plc_ip", path)
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
            "color: #3fb950; font-weight: bold;"
            if ok
            else "color: #f85149; font-weight: bold;"
        )

    # --- commissioning constants (disaster recovery) ------------------------
    def _build_constants_group(self) -> QGroupBox:
        box = QGroupBox("Commissioning constants — push to PLC (disaster recovery)")
        v = QVBoxLayout(box)
        v.addWidget(QLabel(
            "Set-by-hand tuning/home values Studio 5000 does NOT restore on a "
            "download. Snapshot the live set after commissioning, then Push to "
            "restore it if the controller is reloaded or cleared. Needs a "
            "connected PLC (simulated or real)."
        ))
        cols = ["Tag", "Value", "Unit", "Live PLC"]
        self.const_table = QTableWidget(len(COMMISSIONING_CONSTANTS), len(cols))
        self.const_table.setHorizontalHeaderLabels(cols)
        self.const_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self.const_table.verticalHeader().setVisible(False)
        self.const_table.setAlternatingRowColors(True)
        self.const_table.setMaximumHeight(230)
        self._seed_constants_table()
        header = self.const_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        v.addWidget(self.const_table)

        row = QHBoxLayout()
        self.const_read_btn = QPushButton("Read from PLC (snapshot)")
        self.const_push_btn = QPushButton("Push to PLC…")
        save_btn = QPushButton("Save values")
        self.const_read_btn.clicked.connect(self._on_const_read)
        self.const_push_btn.clicked.connect(self._on_const_push)
        save_btn.clicked.connect(self._on_const_save)
        for b in (self.const_read_btn, self.const_push_btn, save_btn):
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)

        v.addWidget(self._build_offset_calibrator())

        self.const_status = QLabel()
        v.addWidget(self.const_status)

        self.connectionChanged.connect(self._refresh_const_enabled)
        self._refresh_const_enabled()
        return box

    def _build_offset_calibrator(self) -> QGroupBox:
        self._cal_solution = None
        box = QGroupBox("Calibrate HOME_OFFSET from a known point")
        v = QVBoxLayout(box)
        v.addWidget(QLabel(
            "Home the robot, seat the tool on a jig at a measured robot-frame "
            "(X, Y) mm, then Compute — the app reads the ClearLink commanded "
            "step count (open-loop, zeroed at home) and solves both offsets. "
            "See docs/home_offset_calibration.md."
        ))
        row = QHBoxLayout()
        row.addWidget(QLabel("X:"))
        self.cal_x = QDoubleSpinBox()
        self.cal_x.setRange(-400.0, 400.0)
        self.cal_x.setDecimals(1)
        self.cal_x.setSuffix(" mm")
        row.addWidget(self.cal_x)
        row.addWidget(QLabel("Y:"))
        self.cal_y = QDoubleSpinBox()
        self.cal_y.setRange(0.0, 500.0)
        self.cal_y.setDecimals(1)
        self.cal_y.setValue(250.0)
        self.cal_y.setSuffix(" mm")
        row.addWidget(self.cal_y)
        self.cal_compute_btn = QPushButton("Compute from PLC")
        self.cal_compute_btn.clicked.connect(self._on_offset_compute)
        row.addWidget(self.cal_compute_btn)
        self.cal_apply_btn = QPushButton("Apply to table")
        self.cal_apply_btn.setEnabled(False)
        self.cal_apply_btn.clicked.connect(self._on_offset_apply)
        row.addWidget(self.cal_apply_btn)
        row.addStretch(1)
        v.addLayout(row)
        self.cal_result = QLabel()
        self.cal_result.setWordWrap(True)
        v.addWidget(self.cal_result)
        return box

    def _on_offset_compute(self) -> None:
        client = self._constants_client()
        if client is None:
            self._set_cal_result("No PLC connected.", ok=False)
            return
        try:
            posn_l = int(client.read(_CMD_POSN["HOME_OFFSET_L"]))
            posn_r = int(client.read(_CMD_POSN["HOME_OFFSET_R"]))
        except (PlcError, Exception) as exc:  # noqa: BLE001
            self._set_cal_result(f"Could not read CommandedPosn: {exc}", ok=False)
            return
        try:
            sol = solve_home_offsets(
                self.controller.kin, self.cal_x.value(), self.cal_y.value(),
                posn_l, posn_r)
        except KinematicsError as exc:
            self._set_cal_result(f"Unreachable point — check X/Y: {exc}", ok=False)
            return
        self._cal_solution = sol
        self.cal_apply_btn.setEnabled(True)
        self._set_cal_result(
            f"At ({sol.x:g}, {sol.y:g}) mm:  θL={sol.theta_left_deg:.3f}°  "
            f"θR={sol.theta_right_deg:.3f}°  |  posn L={sol.posn_left} "
            f"R={sol.posn_right}  →  HOME_OFFSET_L={sol.offset_left}, "
            f"HOME_OFFSET_R={sol.offset_right}.  Apply, then Push to PLC.",
            ok=True)

    def _on_offset_apply(self) -> None:
        sol = self._cal_solution
        if sol is None:
            return
        names = [c.name for c in COMMISSIONING_CONSTANTS]
        self.const_table.item(names.index("HOME_OFFSET_L"), 1).setText(
            str(sol.offset_left))
        self.const_table.item(names.index("HOME_OFFSET_R"), 1).setText(
            str(sol.offset_right))
        self._set_const_status(
            "Applied computed offsets to the table — review, then Push to PLC.",
            ok=True)

    def _set_cal_result(self, text: str, *, ok: bool) -> None:
        self.cal_result.setText(text)
        self.cal_result.setStyleSheet(
            "color: #3fb950;" if ok else "color: #f85149;")

    def _seed_constants_table(self) -> None:
        ro = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
        for r, c in enumerate(COMMISSIONING_CONSTANTS):
            name = QTableWidgetItem(c.name)
            name.setFlags(ro)
            name.setToolTip(c.desc)
            val = QTableWidgetItem(self._fmt(c, self.const_store.get(c.name)))
            val.setFlags(ro | Qt.ItemFlag.ItemIsEditable)
            unit = QTableWidgetItem(c.unit)
            unit.setFlags(ro)
            live = QTableWidgetItem("")
            live.setFlags(ro)
            for col, item in enumerate((name, val, unit, live)):
                self.const_table.setItem(r, col, item)

    @staticmethod
    def _fmt(constant, value) -> str:
        return str(int(round(value))) if constant.dtype == "DINT" else f"{float(value):g}"

    def _constants_client(self):
        """The live PLC client if one is connected, else None."""
        client = getattr(self.controller.driver, "client", None)
        if client is not None and client.is_connected:
            return client
        return None

    def _refresh_const_enabled(self) -> None:
        client = self._constants_client()
        for b in (self.const_read_btn, self.const_push_btn):
            b.setEnabled(client is not None)
        if client is None:
            self.const_status.setText("Connect a PLC (simulated or real) to read/push.")
            self.const_status.setStyleSheet("color: #8b949e;")

    def _table_values(self) -> Dict[str, float]:
        """Parse the editable Value column; raises ValueError on a bad cell."""
        out: Dict[str, float] = {}
        for r, c in enumerate(COMMISSIONING_CONSTANTS):
            text = self.const_table.item(r, 1).text().strip()
            try:
                out[c.name] = float(text)
            except ValueError:
                raise ValueError(f"{c.name}: '{text}' is not a number")
        return out

    def _set_live_column(self, values: Dict[str, float]) -> None:
        for r, c in enumerate(COMMISSIONING_CONSTANTS):
            if c.name in values:
                self.const_table.item(r, 3).setText(self._fmt(c, values[c.name]))

    def _on_const_save(self) -> None:
        try:
            values = self._table_values()
        except ValueError as exc:
            self._set_const_status(str(exc), ok=False)
            return
        self.const_store.update(values)
        if self.const_store.path is not None:
            self.const_store.save()
        self._set_const_status(
            f"Saved {len(values)} values to {self.const_store.path}", ok=True)

    def _on_const_read(self) -> None:
        client = self._constants_client()
        if client is None:
            self._set_const_status("No PLC connected.", ok=False)
            return
        values = read_constants(client)
        self._set_live_column(values)
        # A snapshot also seeds the editable values and persists them (backup).
        for r, c in enumerate(COMMISSIONING_CONSTANTS):
            if c.name in values:
                self.const_table.item(r, 1).setText(self._fmt(c, values[c.name]))
        self.const_store.update(values)
        if self.const_store.path is not None:
            self.const_store.save()
        self._set_const_status(
            f"Snapshot: read {len(values)} values from the PLC and saved them.",
            ok=True)

    def _on_const_push(self) -> None:
        client = self._constants_client()
        if client is None:
            self._set_const_status("No PLC connected.", ok=False)
            return
        try:
            values = self._table_values()
        except ValueError as exc:
            self._set_const_status(str(exc), ok=False)
            return
        confirm = QMessageBox.question(
            self,
            "Push constants to PLC",
            f"Write {len(values)} commissioning constants to the PLC?\n\n"
            "This overwrites the live values (HOME_OFFSET, home angles, speeds, "
            "timeouts). Use only to restore a known-good set.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return
        results = push_constants(client, values)
        by_name = {res.name: res for res in results}
        for r, c in enumerate(COMMISSIONING_CONSTANTS):
            res = by_name.get(c.name)
            tint = _OK_TINT if (res and res.ok) else _FAIL_TINT
            self.const_table.item(r, 3).setBackground(tint)
        self._set_live_column(read_constants(client))
        ok = sum(1 for res in results if res.ok)
        failed = [res.name for res in results if not res.ok]
        if failed:
            self._set_const_status(
                f"Pushed {ok}/{len(results)}. Failed: {', '.join(failed)}", ok=False)
        else:
            self._set_const_status(
                f"Pushed all {ok} constants to the PLC.", ok=True)

    def _set_const_status(self, text: str, *, ok: bool) -> None:
        self.const_status.setText(text)
        self.const_status.setStyleSheet(
            "color: #3fb950; font-weight: bold;" if ok
            else "color: #f85149; font-weight: bold;")

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
