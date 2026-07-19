"""Drives tab — EtherCAT motor setup / commissioning page.

Four areas:
  * Connection — the master's network interface (EtherCAT binds a raw NIC, not
    an IP; the field persists in app settings) + connect real / simulated.
  * Per-drive status — CiA 402 state, statusword, mode, and live encoder
    counts (raw counts + shoulder degrees).
  * I/O — statusword bits and the CiA 402 digital inputs (0x60FD: limits +
    home switch) as ISA-101 indicators: neutral when inactive, dark when
    active; FAULT is the only red, WARNING the only amber.
  * Parameters — the settable table (motion limits + drive SDO objects),
    persisted to config/drive_parameters.yaml, applied to the live driver.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController
from ..ethercat import cia402
from ..ethercat.ethercat_driver import EtherCatRobotDriver
from ..ethercat.igh_master import IgHMaster
from ..ethercat.master import MasterError, SimulatedEtherCatMaster
from ..ethercat.parameters import PARAMETERS, ParameterStore
from ..robot.driver import HomingConfig, RobotDriverError
from . import theme

_SW_BITS = [  # (label, mask, kind-when-active)
    ("Ready", cia402.SW_READY_TO_SWITCH_ON, "ok"),
    ("Switched on", cia402.SW_SWITCHED_ON, "ok"),
    ("Op enabled", cia402.SW_OPERATION_ENABLED, "ok"),
    ("Voltage", cia402.SW_VOLTAGE_ENABLED, "ok"),
    ("Quick stop", cia402.SW_QUICK_STOP, "ok"),
    ("Warning", cia402.SW_WARNING, "warn"),
    ("Fault", cia402.SW_FAULT, "bad"),
    ("Target reached", cia402.SW_TARGET_REACHED, "ok"),
    ("Internal limit", cia402.SW_INTERNAL_LIMIT, "warn"),
]
_DI_BITS = [("Neg limit", 1 << 0, "warn"), ("Pos limit", 1 << 1, "warn"),
            ("Home switch", 1 << 2, "ok")]


class _Bit(QLabel):
    """ISA-101 bit indicator: light gray when inactive, filled when active;
    red/amber reserved for fault/warning bits."""

    def __init__(self, label: str, kind: str) -> None:
        super().__init__(label)
        self._kind = kind
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_active(False)

    def set_active(self, on: bool) -> None:
        if not on:
            style = (f"background:#d4d7d8; color:{theme.TEXT_DIM}; "
                     "border-radius:4px; padding:2px 7px;")
        elif self._kind == "bad":
            style = ("background:%s; color:#ffffff; border-radius:4px; "
                     "padding:2px 7px; font-weight:600;" % theme.DANGER)
        elif self._kind == "warn":
            style = ("background:#e8ae1b; color:#22282b; border-radius:4px; "
                     "padding:2px 7px; font-weight:600;")
        else:
            style = (f"background:#cbd0d2; color:{theme.TEXT}; "
                     "border-radius:4px; padding:2px 7px; font-weight:600;")
        self.setStyleSheet(style)


class _StatusPoller(QThread):
    """Snapshots the two drive process images off the GUI thread (~4 Hz)."""

    updated = Signal(object)

    def __init__(self, get_master, interval_ms: int = 250, parent=None) -> None:
        super().__init__(parent)
        self._get_master = get_master
        self._interval_ms = interval_ms
        self._stop = False

    def stop(self) -> None:
        self._stop = True
        self.wait(1500)

    def run(self) -> None:  # pragma: no cover - thread loop
        while not self._stop:
            master = self._get_master()
            snap = None
            if master is not None and master.is_open:
                try:
                    snap = [dict(sw=d.statusword, mode=d.mode_display,
                                 act=d.actual_position, tgt=d.target_position,
                                 di=d.digital_inputs, err=d.error_code,
                                 fe=d.following_error) for d in master.drives]
                except Exception:  # noqa: BLE001 - display poller must not die
                    snap = None
            if self._stop:
                break
            self.updated.emit(snap)
            self.msleep(self._interval_ms)


class _JogWorker(QThread):
    """Runs a blocking jog off the GUI thread so the poller keeps updating."""

    done = Signal(str)   # error message, or "" on success

    def __init__(self, fn, parent=None) -> None:
        super().__init__(parent)
        self._fn = fn

    def run(self) -> None:  # pragma: no cover - exercised via the GUI/hardware
        try:
            self._fn()
            self.done.emit("")
        except Exception as exc:  # noqa: BLE001
            self.done.emit(str(exc))


class EtherCatTab(QWidget):
    connectionChanged = Signal()

    def __init__(self, controller: RobotTestController, settings=None,
                 config_dir: Optional[str | Path] = None,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        cfg_dir = Path(config_dir) if config_dir else Path("config")
        self.store = ParameterStore.load(cfg_dir / "drive_parameters.yaml")
        self._poller: Optional[_StatusPoller] = None
        self._jog_worker: Optional[_JogWorker] = None

        # This is an HMI — the whole page must NOT scroll. Keep the sections
        # compact and let only the (tall) parameter tables scroll internally.
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        top = QHBoxLayout()
        top.addWidget(self._build_connection(), 1)
        root.addLayout(top)
        drives = QHBoxLayout()
        self._drive_panels = [self._build_drive_panel("Drive 0 — left shoulder"),
                              self._build_drive_panel("Drive 1 — right shoulder")]
        for panel, _ in self._drive_panels:
            drives.addWidget(panel, 1)
        root.addLayout(drives)
        motion = QHBoxLayout()
        motion.addWidget(self._build_jog(), 1)
        motion.addWidget(self._build_coordinated(), 1)
        root.addLayout(motion)
        root.addWidget(self._build_parameters(), 1)

        self.status_label = QLabel("Not connected — connect the EtherCAT master, "
                                   "or use the simulated network for bench-off work.")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        root.addWidget(self.status_label)
        self.refresh()

    # --- construction -------------------------------------------------------
    def _build_connection(self) -> QGroupBox:
        box = QGroupBox("Connection — EtherCAT master")
        g = QGridLayout(box)
        g.addWidget(QLabel("Network interface"), 0, 0)
        self.if_edit = QLineEdit()
        self.if_edit.setPlaceholderText("e.g. enp3s0  (EtherCAT binds a raw NIC — no IP)")
        if self.settings is not None:
            self.if_edit.setText(str(self.settings.get("ethercat_ifname", "") or ""))
        g.addWidget(self.if_edit, 0, 1)
        g.addWidget(QLabel("Drives on bus"), 1, 0)
        self.drives_spin = QSpinBox()
        self.drives_spin.setRange(1, 2)
        try:
            n = int(self.settings.get("ethercat_num_drives", 2)) if self.settings else 2
        except (TypeError, ValueError):
            n = 2
        self.drives_spin.setValue(max(1, min(2, n)))
        self.drives_spin.setToolTip(
            "Drives expected on the EtherCAT chain. Set 1 for single-axis bench "
            "bring-up; 2 for the assembled robot.")
        g.addWidget(self.drives_spin, 1, 1, Qt.AlignmentFlag.AlignLeft)
        note = QLabel("Real HW uses the IgH master (NIC set in ethercat.conf; this "
                      "field is informational). Connect launches the RT daemon.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color:{theme.TEXT_DIM};")
        g.addWidget(note, 2, 0, 1, 2)
        row = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_real)
        self.sim_btn = QPushButton("Connect simulated")
        self.sim_btn.clicked.connect(self._on_connect_sim)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        self.reset_btn = QPushButton("Reset fault")
        self.reset_btn.clicked.connect(self._on_reset_fault)
        for b in (self.connect_btn, self.sim_btn, self.disconnect_btn, self.reset_btn):
            row.addWidget(b)
        row.addStretch(1)
        self.conn_label = QLabel("DISCONNECTED")
        self.conn_label.setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")
        row.addWidget(self.conn_label)
        g.addLayout(row, 3, 0, 1, 2)
        return box

    def _build_drive_panel(self, title: str):
        box = QGroupBox(title)
        v = QVBoxLayout(box)
        v.setContentsMargins(8, 6, 8, 6)
        v.setSpacing(3)
        state = QLabel("state: —")
        state.setStyleSheet("font-family:monospace; font-weight:600;")
        counts = QLabel("encoder: — counts   |   — °")
        counts.setStyleSheet("font-family:monospace;")
        detail = QLabel("statusword — · mode — · target —")
        detail.setStyleSheet(f"font-family:monospace; color:{theme.TEXT_DIM};")
        # These carry live numbers. Fixed-width formatting (in _on_snapshot) keeps
        # the text a constant length; Ignored horizontal policy stops any label
        # from driving the panel width, so the two-panel split can't jitter as
        # counts change.
        for lbl in (state, counts, detail):
            lbl.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        v.addWidget(state)
        v.addWidget(counts)
        v.addWidget(detail)
        io = QGridLayout()
        io.setSpacing(3)
        bits = []
        for i, (name, mask, kind) in enumerate(_SW_BITS):
            b = _Bit(name, kind)
            bits.append(("sw", mask, b))
            io.addWidget(b, i // 3, i % 3)
        for j, (name, mask, kind) in enumerate(_DI_BITS):
            b = _Bit(name, kind)
            bits.append(("di", mask, b))
            io.addWidget(b, (len(_SW_BITS) + 2) // 3, j)   # next row, no empty gap
        v.addLayout(io)
        widgets = dict(state=state, counts=counts, detail=detail, bits=bits)
        return box, widgets

    def _build_jog(self) -> QGroupBox:
        box = QGroupBox("Bench jog — single axis  (motion)")
        g = QGridLayout(box)
        g.setContentsMargins(8, 6, 8, 6)
        g.setSpacing(4)
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.clicked.connect(self._on_enable)
        self.disable_btn = QPushButton("Disable")
        self.disable_btn.clicked.connect(self._on_disable)
        g.addWidget(self.enable_btn, 0, 0)
        g.addWidget(self.disable_btn, 0, 1)
        g.addWidget(QLabel("axis"), 0, 2)
        self.jog_axis = QSpinBox()
        self.jog_axis.setRange(0, 1)     # bench max is 2 drives; index validated on jog
        self.jog_axis.setMaximumWidth(48)
        self.jog_axis.setToolTip(
            "Which drive on the bus to jog (0 = first slave, 1 = second). "
            "Jog each after wiring to confirm it maps to the axis you expect.")
        g.addWidget(self.jog_axis, 0, 3)
        # step + speed on the second row so the box stays narrow enough to sit
        # beside the coordinated-move box without overflowing the page width.
        g.addWidget(QLabel("step"), 1, 0)
        self.jog_step = QSpinBox()
        self.jog_step.setRange(1, 200000)
        self.jog_step.setValue(2000)
        self.jog_step.setMaximumWidth(88)
        self.jog_step.setToolTip("Jog distance in raw drive counts.")
        g.addWidget(self.jog_step, 1, 1)
        g.addWidget(QLabel("speed"), 1, 2)
        self.jog_speed = QSpinBox()
        self.jog_speed.setRange(100, 500000)
        self.jog_speed.setValue(20000)
        self.jog_speed.setMaximumWidth(88)
        self.jog_speed.setToolTip("Jog / coordinated-move speed in counts/s.")
        g.addWidget(self.jog_speed, 1, 3)
        self.jog_minus = QPushButton("– Jog")
        self.jog_minus.clicked.connect(lambda: self._on_jog(-1))
        self.jog_plus = QPushButton("Jog +")
        self.jog_plus.clicked.connect(lambda: self._on_jog(+1))
        g.addWidget(self.jog_minus, 0, 4)
        g.addWidget(self.jog_plus, 0, 5)
        warn = QLabel("Motion — E-stop/contactor live, motor secured. Small steps first.")
        warn.setWordWrap(True)
        warn.setStyleSheet(f"color:{theme.WARN}; font-weight:600;")
        g.addWidget(warn, 2, 0, 1, 6)
        return box

    def _build_coordinated(self) -> QGroupBox:
        """Coordinated two-axis move: both drives ramp through one shared time
        profile (start + finish together). Joint-space, so it's safe before the
        arm is in the linkage — this is the first lockstep-streaming test."""
        box = QGroupBox("Coordinated move — both axes  (motion)")
        g = QGridLayout(box)
        g.setContentsMargins(8, 6, 8, 6)
        g.setSpacing(4)
        g.addWidget(QLabel("axis 0 Δ"), 0, 0)
        self.coord_d0 = QSpinBox()
        self.coord_d0.setRange(-200000, 200000)
        self.coord_d0.setValue(2000)
        self.coord_d0.setMaximumWidth(88)
        g.addWidget(self.coord_d0, 0, 1)
        g.addWidget(QLabel("axis 1 Δ"), 0, 2)
        self.coord_d1 = QSpinBox()
        self.coord_d1.setRange(-200000, 200000)
        self.coord_d1.setValue(-2000)
        self.coord_d1.setMaximumWidth(88)
        g.addWidget(self.coord_d1, 0, 3)
        self.coord_btn = QPushButton("Move both")
        self.coord_btn.clicked.connect(self._on_coord_move)
        g.addWidget(self.coord_btn, 1, 0, 1, 2)
        hint = QLabel("Both axes reach target together (shared trapezoid, speed "
                      "above). Opposite signs check direction; small deltas first.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{theme.TEXT_DIM};")
        g.addWidget(hint, 1, 2, 1, 2)
        return box

    def _ec_driver(self):
        drv = self.controller.driver
        return drv if isinstance(drv, EtherCatRobotDriver) else None

    def _on_enable(self) -> None:
        drv = self._ec_driver()
        if drv is None:
            self._status("Connect the drives first.", theme.WARN)
            return
        try:
            drv.enable()
            self._status("Enabled — Operation Enabled. Jog with care.", theme.TEXT)
        except RobotDriverError as exc:
            self._status(f"Enable failed: {exc}", theme.DANGER)

    def _on_disable(self) -> None:
        drv = self._ec_driver()
        if drv is None:
            return
        drv.disable()
        self._status("Disabled — torque off (drive tracks position).", theme.TEXT)

    def _set_motion_enabled(self, on: bool) -> None:
        for b in (self.enable_btn, self.disable_btn, self.jog_minus, self.jog_plus,
                  self.coord_btn):
            b.setEnabled(on)

    def _on_jog(self, sign: int) -> None:
        drv = self._ec_driver()
        if drv is None:
            self._status("Connect + enable the drive first.", theme.WARN)
            return
        if self._jog_worker is not None and self._jog_worker.isRunning():
            return
        axis = int(self.jog_axis.value())
        delta = sign * int(self.jog_step.value())
        speed = float(self.jog_speed.value())
        # Run off the GUI thread so the poller keeps updating (see following error).
        self._set_motion_enabled(False)
        self._status(f"Jogging axis {axis} by {delta:+d} counts…", theme.TEXT)
        self._jog_worker = _JogWorker(
            lambda: drv.jog_counts(axis, delta, speed_counts_s=speed))
        self._jog_worker.done.connect(self._on_jog_done)
        self._jog_worker.start()

    def _on_jog_done(self, err: str) -> None:
        self._set_motion_enabled(True)
        if err:
            self._status(f"Jog failed: {err}", theme.DANGER)
        else:
            self._status("Jog complete.", theme.TEXT)

    def _on_coord_move(self) -> None:
        drv = self._ec_driver()
        if drv is None:
            self._status("Connect + enable the drives first.", theme.WARN)
            return
        if self._jog_worker is not None and self._jog_worker.isRunning():
            return
        n = len(drv.master.drives)
        deltas = [int(self.coord_d0.value()), int(self.coord_d1.value())][:n]
        speed = float(self.jog_speed.value())
        self._set_motion_enabled(False)
        self._status(f"Coordinated move {deltas} counts…", theme.TEXT)
        self._jog_worker = _JogWorker(
            lambda: drv.jog_counts_multi(deltas, speed_counts_s=speed))
        self._jog_worker.done.connect(self._on_jog_done)
        self._jog_worker.start()

    def _build_parameters(self) -> QGroupBox:
        box = QGroupBox("Parameters")
        v = QVBoxLayout(box)
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(6)
        tables = QHBoxLayout()
        tables.addWidget(self._build_motion_params(), 3)
        tables.addWidget(self._build_custom_params(), 4)
        v.addLayout(tables)
        row = QHBoxLayout()
        self.save_btn = QPushButton("Save parameters")
        self.save_btn.clicked.connect(self._on_save_params)
        self.apply_btn = QPushButton("Apply to drives")
        self.apply_btn.setProperty("accent", "primary")
        self.apply_btn.clicked.connect(self._on_apply_params)
        self.refresh_btn = QPushButton("Refresh from drives")
        self.refresh_btn.clicked.connect(self._on_refresh_drives)
        row.addWidget(self.save_btn)
        row.addWidget(self.apply_btn)
        row.addWidget(self.refresh_btn)
        row.addStretch(1)
        v.addLayout(row)
        return box

    def _build_motion_params(self) -> QGroupBox:
        box = QGroupBox("Motion + drive SDO")
        v = QVBoxLayout(box)
        v.setContentsMargins(6, 6, 6, 6)
        self.table = QTableWidget(len(PARAMETERS), 5)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value", "Unit", "Scope", "Description"])
        for r, p in enumerate(PARAMETERS):
            for c, text in ((0, p.name), (2, p.unit),
                            (3, "SDO 0x%04X:%d" % p.sdo if p.sdo else "motion"),
                            (4, p.desc)):
                item = QTableWidgetItem(text)
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(r, c, item)
            self.table.setItem(r, 1, QTableWidgetItem(self._fmt(self.store.get(p.name))))
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        # Fill the container (which the parameters box sizes via stretch) and
        # scroll internally when the rows exceed it — no empty space, no page grow.
        self.table.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        v.addWidget(self.table, 1)
        return box

    def _build_custom_params(self) -> QGroupBox:
        """User-added drive objects (gains/stiffness). Add any tuning parameter
        by its friendly ``Cxx.NN`` address or a raw ``0xINDEX:SUB`` CoE address;
        Apply writes them to the live drive over SDO."""
        box = QGroupBox("Tuning parameters (drive SDO)")
        v = QVBoxLayout(box)
        v.setContentsMargins(6, 6, 6, 6)
        v.setSpacing(4)
        hint = QLabel("Edit Value → Apply writes both drives; Refresh reads each "
                      "back. Verify Cxx.NN vs the manual. Hover a name for detail.")
        hint.setWordWrap(True)
        hint.setStyleSheet(f"color:{theme.TEXT_DIM}; font-size:11px;")
        v.addWidget(hint)
        self.custom_table = QTableWidget(0, 5)
        self.custom_table.setHorizontalHeaderLabels(
            ["Parameter", "Address", "Value", "Drive 1", "Drive 2"])
        self.custom_table.horizontalHeader().setStretchLastSection(True)
        self.custom_table.verticalHeader().setVisible(False)
        self.custom_table.setSizePolicy(QSizePolicy.Policy.Expanding,
                                        QSizePolicy.Policy.Expanding)
        v.addWidget(self.custom_table, 1)
        add = QHBoxLayout()
        self.cp_name = QLineEdit()
        self.cp_name.setPlaceholderText("name (e.g. rigidity)")
        self.cp_addr = QLineEdit()
        self.cp_addr.setPlaceholderText("Cxx.NN or 0x20xx:NN")
        self.cp_val = QLineEdit()
        self.cp_val.setPlaceholderText("value")
        self.cp_val.setMaximumWidth(90)
        self.cp_type = QComboBox()
        self.cp_type.addItems(["int", "int16", "int8", "float"])
        add_btn = QPushButton("Add")
        add_btn.clicked.connect(self._on_add_custom)
        rm_btn = QPushButton("Remove selected")
        rm_btn.clicked.connect(self._on_remove_custom)
        for w in (self.cp_name, self.cp_addr, self.cp_val, self.cp_type,
                  add_btn, rm_btn):
            add.addWidget(w)
        v.addLayout(add)
        self._refresh_custom_table()
        return box

    def _refresh_custom_table(self) -> None:
        cps = self.store.custom_parameters()
        rb = getattr(self, "_drive_readback", {})
        self.custom_table.setRowCount(len(cps))
        for r, c in enumerate(cps):
            name_item = QTableWidgetItem(c.name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            if c.desc:
                name_item.setToolTip(c.desc)
            self.custom_table.setItem(r, 0, name_item)
            addr_item = QTableWidgetItem(c.address)
            addr_item.setFlags(addr_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.custom_table.setItem(r, 1, addr_item)
            self.custom_table.setItem(r, 2, QTableWidgetItem(self._fmt(c.value)))  # editable
            drive_vals = rb.get(c.name, [])
            for di in range(2):
                v = drive_vals[di] if di < len(drive_vals) else None
                it = QTableWidgetItem("—" if v is None else self._fmt(v))
                it.setFlags(it.flags() & ~Qt.ItemFlag.ItemIsEditable)
                # Flag a drive whose actual doesn't match the setpoint (helps
                # confirm whether Apply actually landed).
                if v is not None and int(v) != int(c.value):
                    it.setForeground(QColor(theme.WARN))
                self.custom_table.setItem(r, 3 + di, it)
        self.custom_table.resizeColumnsToContents()
        self.custom_table.horizontalHeader().setStretchLastSection(True)

    def _on_refresh_drives(self) -> None:
        drv = self.controller.driver
        if not isinstance(drv, EtherCatRobotDriver):
            self._status("Connect the drives first to read their values.", theme.WARN)
            return
        try:
            self._drive_readback = self.store.read_custom_from_drives(drv)
        except Exception as exc:  # noqa: BLE001
            self._status(f"Read failed: {exc}", theme.DANGER)
            return
        self._refresh_custom_table()
        self._status("Read tuning values back from the drives — mismatches shown amber.",
                     theme.TEXT)

    def _on_add_custom(self) -> None:
        name = self.cp_name.text().strip()
        addr = self.cp_addr.text().strip()
        if not name or not addr:
            self._status("Custom parameter needs a name and an address.", theme.WARN)
            return
        try:
            self.store.add_custom(name, addr, self.cp_val.text() or 0,
                                  self.cp_type.currentText())
        except ValueError as exc:
            self._status(f"Bad address: {exc}", theme.WARN)
            return
        self.store.save()
        self.cp_name.clear()
        self.cp_addr.clear()
        self.cp_val.clear()
        self._refresh_custom_table()
        self._status(f"Added custom parameter '{name}'.", theme.TEXT)

    def _on_remove_custom(self) -> None:
        row = self.custom_table.currentRow()
        if row < 0:
            self._status("Select a custom parameter row to remove.", theme.WARN)
            return
        name_item = self.custom_table.item(row, 0)
        if name_item is None:
            return
        self.store.remove_custom(name_item.text())
        self.store.save()
        self._refresh_custom_table()
        self._status("Removed custom parameter.", theme.TEXT)

    def _read_custom_table(self) -> bool:
        for r, c in enumerate(self.store.custom_parameters()):
            item = self.custom_table.item(r, 2)
            if item is None:
                continue
            try:
                self.store.set_custom_value(c.name, float(item.text()))
            except (TypeError, ValueError):
                self._status(f"'{c.name}' value is not a number - fix it and retry.",
                             theme.WARN)
                return False
        return True

    @staticmethod
    def _fmt(v: float) -> str:
        return f"{v:g}"

    # --- connection ----------------------------------------------------------
    def _master(self):
        drv = self.controller.driver
        return drv.master if isinstance(drv, EtherCatRobotDriver) else None

    def _adopt(self, master) -> None:
        homing = HomingConfig()
        driver = EtherCatRobotDriver(
            master, self.controller.kin, self.controller.validator,
            home_angles=homing.home_angles,
            limits=self.store.trajectory_limits(),
        ).connect()
        self.controller.set_driver(driver)
        self._start_poller()
        self.connectionChanged.emit()
        self.refresh()

    def _on_connect_real(self) -> None:
        n_drives = self.drives_spin.value()
        if self.settings is not None:
            self.settings.set("ethercat_num_drives", n_drives)
        # Real hardware runs on the IgH master (the AS715N is DC-SYNC0-only and
        # pysoem can't generate SYNC0); IgHMaster launches/maps the C RT daemon.
        try:
            master = IgHMaster(num_drives=n_drives,
                               cycle_dt_s=self.store.get("cycle_dt_s")).open()
        except MasterError as exc:
            self._status(f"Connect failed: {exc}", theme.DANGER)
            return
        self._adopt(master)  # pragma: no cover - real hardware path
        bench = "  [single-axis bench]" if n_drives == 1 else ""
        self._status(f"Connected — IgH EtherCAT master.{bench}", theme.TEXT)

    def _on_connect_sim(self) -> None:
        self._adopt(SimulatedEtherCatMaster(
            num_drives=self.drives_spin.value(),
            cycle_dt_s=self.store.get("cycle_dt_s")).open())
        self._status("Connected to the SIMULATED drive network.", theme.TEXT)

    def _on_reset_fault(self) -> None:
        """Clear a latched drive fault (CiA 402 fault-reset edge). Lets us tell a
        stale latched Er741 from one that re-trips immediately."""
        drv = self.controller.driver
        if not isinstance(drv, EtherCatRobotDriver):
            self._status("Not connected — nothing to reset.", theme.WARN)
            return
        try:
            drv.reset()
            self._status("Fault reset sent — watch whether it clears or re-trips.",
                         theme.TEXT)
        except Exception as exc:  # noqa: BLE001
            self._status(f"Reset failed: {exc}", theme.DANGER)

    def _stop_jog(self) -> None:
        if self._jog_worker is not None:
            self._jog_worker.wait(3000)
            self._jog_worker = None

    def _teardown_master(self) -> None:
        """Disable the drives (torque off) and stop the master/daemon. Safe to
        call when not connected."""
        self._stop_jog()
        self._stop_poller()
        drv = self._ec_driver()
        if drv is not None:
            try:
                drv.close()            # disable() then master.close() (stops daemon)
            except Exception:  # noqa: BLE001 - shutdown must not raise
                pass

    def shutdown(self) -> None:
        """App is closing: always leave the drives disabled and the master down."""
        self._teardown_master()

    def _on_disconnect(self) -> None:
        self._teardown_master()
        from ..robot.driver import DryRunRobotDriver

        self.controller.set_driver(DryRunRobotDriver())
        self.connectionChanged.emit()
        self.refresh()
        self._status("Disconnected — drives disabled, master stopped.", theme.TEXT)

    # --- parameters -----------------------------------------------------------
    def _read_table(self) -> bool:
        for r, p in enumerate(PARAMETERS):
            item = self.table.item(r, 1)
            try:
                self.store.set(p.name, float(item.text()))
            except (TypeError, ValueError):
                self._status(f"'{p.name}' is not a number - fix it and retry.", theme.WARN)
                return False
        return True

    def _on_save_params(self) -> None:
        if not self._read_table() or not self._read_custom_table():
            return
        path = self.store.save()
        self._status(f"Parameters saved -> {path}", theme.TEXT)

    def _on_apply_params(self) -> None:
        if not self._read_table() or not self._read_custom_table():
            return
        drv = self.controller.driver
        if not isinstance(drv, EtherCatRobotDriver):
            self._status("Not connected - parameters saved locally only.", theme.WARN)
            self.store.save()
            return
        notes = self.store.apply(drv)
        self.store.save()
        # Read straight back so the Drive columns confirm what actually landed.
        try:
            self._drive_readback = self.store.read_custom_from_drives(drv)
            self._refresh_custom_table()
        except Exception:  # noqa: BLE001 - readback is best-effort
            pass
        self._status("Applied to drives + read back: "
                     + "; ".join(notes[:2]) + (" …" if len(notes) > 2 else ""),
                     theme.TEXT)

    # --- live status ----------------------------------------------------------
    def _start_poller(self) -> None:
        self._stop_poller()
        self._poller = _StatusPoller(self._master, interval_ms=100)
        self._poller.updated.connect(self._on_snapshot)
        self._poller.start()

    def _stop_poller(self) -> None:
        if self._poller is not None:
            self._poller.stop()
            self._poller = None

    def _on_snapshot(self, snap) -> None:
        connected = snap is not None
        self.conn_label.setText("CONNECTED" if connected else "DISCONNECTED")
        self.conn_label.setStyleSheet(
            f"color:{theme.TEXT}; font-weight:600;" if connected
            else f"color:{theme.TEXT_DIM}; font-weight:600;")
        if not connected:
            return
        drv = self.controller.driver
        ppd = self.controller.kin.config.pulses_per_degree
        home_counts = getattr(drv, "_home_counts", [0, 0])
        for i, (panel, w) in enumerate(self._drive_panels):
            if i >= len(snap):
                # Fewer drives on the bus than panels (single-axis bench): mark absent.
                w["state"].setText("state: — (not on bus)")
                w["state"].setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")
                w["counts"].setText("encoder: —")
                w["detail"].setText("—")
                for _src, _mask, bit in w["bits"]:
                    bit.set_active(False)
                continue
            d = snap[i]
            home = home_counts[i] if i < len(home_counts) else 0
            state = cia402.decode_state(d["sw"])
            # Pad to a constant width so the label never changes length as the
            # live values change (which would jitter the layout).
            w["state"].setText(f"state: {state.value.replace('_', ' ').upper():<22}")
            w["state"].setStyleSheet(
                f"font-family:monospace; color:{theme.DANGER}; font-weight:600;"
                if cia402.is_fault(d["sw"])
                else "font-family:monospace; font-weight:600;")
            deg = (d["act"] + home) / ppd
            w["counts"].setText(f"encoder: {d['act']:>+10d} counts   |   {deg:>+9.3f} °")
            w["detail"].setText(
                f"statusword 0x{d['sw']:04X} · err 0x{d.get('err', 0):04X} · "
                f"foll.err {d.get('fe', 0):>+7d} · mode {d['mode']:>2d} · "
                f"target {d['tgt']:>+10d}")
            for src, mask, bit in w["bits"]:
                bit.set_active(bool((d["sw"] if src == "sw" else d["di"]) & mask))

    def refresh(self) -> None:
        connected = self._master() is not None
        self.disconnect_btn.setEnabled(connected)
        if connected and (self._poller is None or not self._poller.isRunning()):
            self._start_poller()
        if not connected:
            self.conn_label.setText("DISCONNECTED")
            self.conn_label.setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")

    def _status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        weight = "font-weight:600;" if color in (theme.DANGER, theme.WARN) else ""
        self.status_label.setStyleSheet(f"color:{color}; {weight}")

    # --- lifecycle ------------------------------------------------------------
    def hideEvent(self, event) -> None:  # noqa: N802
        super().hideEvent(event)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._teardown_master()          # disable drives + stop the daemon
        super().closeEvent(event)
