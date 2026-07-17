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
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController
from ..ethercat import cia402
from ..ethercat.ethercat_driver import EtherCatRobotDriver
from ..ethercat.master import MasterError, PysoemMaster, SimulatedEtherCatMaster
from ..ethercat.parameters import PARAMETERS, ParameterStore
from ..robot.driver import HomingConfig
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
                     "border-radius:4px; padding:3px 8px;")
        elif self._kind == "bad":
            style = ("background:%s; color:#ffffff; border-radius:4px; "
                     "padding:3px 8px; font-weight:600;" % theme.DANGER)
        elif self._kind == "warn":
            style = ("background:#e8ae1b; color:#22282b; border-radius:4px; "
                     "padding:3px 8px; font-weight:600;")
        else:
            style = (f"background:#cbd0d2; color:{theme.TEXT}; "
                     "border-radius:4px; padding:3px 8px; font-weight:600;")
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
                                 di=d.digital_inputs) for d in master.drives]
                except Exception:  # noqa: BLE001 - display poller must not die
                    snap = None
            if self._stop:
                break
            self.updated.emit(snap)
            self.msleep(self._interval_ms)


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

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addWidget(self._build_connection(), 1)
        root.addLayout(top)
        drives = QHBoxLayout()
        self._drive_panels = [self._build_drive_panel("Drive 0 — left shoulder"),
                              self._build_drive_panel("Drive 1 — right shoulder")]
        for panel, _ in self._drive_panels:
            drives.addWidget(panel, 1)
        root.addLayout(drives)
        root.addWidget(self._build_parameters(), 1)
        self.status_label = QLabel("Not connected — connect the EtherCAT master, "
                                   "or use the simulated network for bench-off work.")
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
        note = QLabel("EtherCAT is not IP-based: the master owns this interface "
                      "directly (raw frames). Dedicate a NIC to the drive chain.")
        note.setWordWrap(True)
        note.setStyleSheet(f"color:{theme.TEXT_DIM};")
        g.addWidget(note, 1, 0, 1, 2)
        row = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_real)
        self.sim_btn = QPushButton("Connect simulated")
        self.sim_btn.clicked.connect(self._on_connect_sim)
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.clicked.connect(self._on_disconnect)
        for b in (self.connect_btn, self.sim_btn, self.disconnect_btn):
            row.addWidget(b)
        row.addStretch(1)
        self.conn_label = QLabel("DISCONNECTED")
        self.conn_label.setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")
        row.addWidget(self.conn_label)
        g.addLayout(row, 2, 0, 1, 2)
        return box

    def _build_drive_panel(self, title: str):
        box = QGroupBox(title)
        v = QVBoxLayout(box)
        state = QLabel("state: —")
        state.setStyleSheet("font-weight:600;")
        counts = QLabel("encoder: — counts   |   — °")
        counts.setStyleSheet("font-family:monospace;")
        detail = QLabel("statusword — · mode — · target —")
        detail.setStyleSheet(f"font-family:monospace; color:{theme.TEXT_DIM};")
        v.addWidget(state)
        v.addWidget(counts)
        v.addWidget(detail)
        io = QGridLayout()
        bits = []
        for i, (name, mask, kind) in enumerate(_SW_BITS):
            b = _Bit(name, kind)
            bits.append(("sw", mask, b))
            io.addWidget(b, i // 3, i % 3)
        for j, (name, mask, kind) in enumerate(_DI_BITS):
            b = _Bit(name, kind)
            bits.append(("di", mask, b))
            io.addWidget(b, len(_SW_BITS) // 3 + 1, j)
        v.addLayout(io)
        widgets = dict(state=state, counts=counts, detail=detail, bits=bits)
        return box, widgets

    def _build_parameters(self) -> QGroupBox:
        box = QGroupBox("Parameters (motion + drive SDO)")
        v = QVBoxLayout(box)
        self.table = QTableWidget(len(PARAMETERS), 5)
        self.table.setHorizontalHeaderLabels(["Parameter", "Value", "Unit", "Scope", "Description"])
        for r, p in enumerate(PARAMETERS):
            for c, text in ((0, p.name), (2, p.unit),
                            (3, "drive SDO 0x%04X:%d" % p.sdo if p.sdo else "motion (PC)"),
                            (4, p.desc)):
                item = QTableWidgetItem(text)
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                self.table.setItem(r, c, item)
            self.table.setItem(r, 1, QTableWidgetItem(self._fmt(self.store.get(p.name))))
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setStretchLastSection(True)
        v.addWidget(self.table)
        row = QHBoxLayout()
        self.save_btn = QPushButton("Save parameters")
        self.save_btn.clicked.connect(self._on_save_params)
        self.apply_btn = QPushButton("Apply to drives")
        self.apply_btn.setProperty("accent", "primary")
        self.apply_btn.clicked.connect(self._on_apply_params)
        row.addWidget(self.save_btn)
        row.addWidget(self.apply_btn)
        row.addStretch(1)
        v.addLayout(row)
        return box

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
        ifname = self.if_edit.text().strip()
        if not ifname:
            self._status("Enter the EtherCAT network interface first.", theme.WARN)
            return
        if self.settings is not None:
            self.settings.set("ethercat_ifname", ifname)
        try:
            master = PysoemMaster(ifname=ifname,
                                  cycle_dt_s=self.store.get("cycle_dt_s")).open()
        except MasterError as exc:
            self._status(f"Connect failed: {exc}", theme.DANGER)
            return
        self._adopt(master)  # pragma: no cover - real hardware path
        self._status(f"Connected — EtherCAT on {ifname}.", theme.TEXT)

    def _on_connect_sim(self) -> None:
        self._adopt(SimulatedEtherCatMaster(
            cycle_dt_s=self.store.get("cycle_dt_s")).open())
        self._status("Connected to the SIMULATED drive network.", theme.TEXT)

    def _on_disconnect(self) -> None:
        self._stop_poller()
        from ..robot.driver import DryRunRobotDriver

        self.controller.set_driver(DryRunRobotDriver())
        self.connectionChanged.emit()
        self.refresh()
        self._status("Disconnected — back on the dry-run driver.", theme.TEXT)

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
        if not self._read_table():
            return
        path = self.store.save()
        self._status(f"Parameters saved -> {path}", theme.TEXT)

    def _on_apply_params(self) -> None:
        if not self._read_table():
            return
        drv = self.controller.driver
        if not isinstance(drv, EtherCatRobotDriver):
            self._status("Not connected - parameters saved locally only.", theme.WARN)
            self.store.save()
            return
        notes = self.store.apply(drv)
        self.store.save()
        self._status("Applied: " + "; ".join(notes[:3]) + (" …" if len(notes) > 3 else ""),
                     theme.TEXT)

    # --- live status ----------------------------------------------------------
    def _start_poller(self) -> None:
        self._stop_poller()
        self._poller = _StatusPoller(self._master)
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
        for (panel, w), d, home in zip(self._drive_panels, snap,
                                       getattr(drv, "_home_counts", [0, 0])):
            state = cia402.decode_state(d["sw"])
            w["state"].setText(f"state: {state.value.replace('_', ' ').upper()}")
            w["state"].setStyleSheet(
                f"color:{theme.DANGER}; font-weight:600;" if cia402.is_fault(d["sw"])
                else "font-weight:600;")
            deg = (d["act"] + home) / ppd
            w["counts"].setText(f"encoder: {d['act']:>9d} counts   |   {deg:8.3f} °")
            w["detail"].setText(
                f"statusword 0x{d['sw']:04X} · mode {d['mode']} · target {d['tgt']}")
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
        self._stop_poller()
        super().closeEvent(event)
