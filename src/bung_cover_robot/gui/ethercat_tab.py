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

import time
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
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

from ..app.cycle_manager import PickSequence, demo_pick_and_place_targets
from ..app.robot_test_controller import RobotTestController
from ..ethercat import cia402
from ..ethercat.ethercat_driver import EtherCatRobotDriver
from ..ethercat.igh_master import IgHMaster
from ..ethercat.master import MasterError, SimulatedEtherCatMaster
from ..ethercat.parameters import PARAMETERS, ParameterStore
from ..robot.driver import HomingConfig, RobotDriverError
from . import theme

_SW_BITS = [  # (label, statusword mask)
    ("Ready", cia402.SW_READY_TO_SWITCH_ON),
    ("Switched on", cia402.SW_SWITCHED_ON),
    ("Op enabled", cia402.SW_OPERATION_ENABLED),
    ("Voltage", cia402.SW_VOLTAGE_ENABLED),
    ("Quick stop", cia402.SW_QUICK_STOP),
    ("Warning", cia402.SW_WARNING),
    ("Fault", cia402.SW_FAULT),
    ("Target reached", cia402.SW_TARGET_REACHED),
    ("Internal limit", cia402.SW_INTERNAL_LIMIT),
]
_DI_BITS = [("Neg limit", 1 << 0), ("Pos limit", 1 << 1), ("Home switch", 1 << 2)]

# Rows of the single per-drive status table: (label, kind). kind is a scalar key
# ("state"/"counts"/"angle"/"err"/"fe") or ("sw"|"di", mask) for an on/off bit.
_STATUS_ROWS = (
    [("State", "state"), ("Encoder (counts)", "counts"), ("Angle (deg)", "angle"),
     ("Error code", "err"), ("Following error", "fe"),
     ("Link errors (CRC)", "link")]
    + [(lbl, ("sw", mask)) for lbl, mask in _SW_BITS]
    + [(lbl, ("di", mask)) for lbl, mask in _DI_BITS]
)


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
                                 fe=d.following_error,
                                 link=getattr(d, "link_errors", None))
                            for d in master.drives]
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


class _DemoWorker(QThread):
    """Runs the sample pick&place demo off the GUI thread: pick from a fixed
    nest, drop into each hole of a variably-placed 6-hole cover row, actuating
    the vacuum/cylinder along the way. Loops until stopped when ``loop`` is set;
    re-randomises the cover row each pass."""

    step = Signal(str)          # per-hole progress line
    stats = Signal(float, int)  # (rolling cycles/min, total placed)
    done = Signal(str)          # final message (prefixed 'FAIL:' on failure)

    def __init__(self, controller, make_targets, loop: bool,
                 pick_sequence=None, move_speed_mm_s=None, parent=None) -> None:
        super().__init__(parent)
        self._controller = controller
        self._make_targets = make_targets
        self._loop = loop
        self._pick_sequence = pick_sequence
        self._move_speed_mm_s = move_speed_mm_s
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:  # pragma: no cover - GUI/hardware thread
        from ..app.cycle_manager import CycleRateTracker, run_demo_cycle

        rate = CycleRateTracker()
        try:
            passes = 0
            while not self._stop:
                pick, drops = self._make_targets()

                def _on_step(s):
                    if s.ok:                       # one placed cover = one cycle
                        now = time.perf_counter()
                        rate.record(now)
                        self.stats.emit(rate.per_minute(now), rate.total)
                    tag = "placed" if s.ok else "skipped"
                    self.step.emit(
                        f"pass {passes + 1}: hole {s.hole_index + 1}/{len(drops)} "
                        f"{tag} — {s.reason}")

                res = run_demo_cycle(
                    self._controller, pick, drops,
                    pick_sequence=self._pick_sequence,
                    move_speed_mm_s=self._move_speed_mm_s,
                    should_stop=lambda: self._stop, on_step=_on_step,
                )
                passes += 1
                if not res.ok and "stopped" not in res.reason:
                    self.done.emit(f"FAIL:{res.reason}")
                    return
                if not self._loop or self._stop:
                    self.done.emit(f"Demo finished — {res.reason} "
                                   f"[{passes} pass(es)]")
                    return
        except Exception as exc:  # noqa: BLE001
            self.done.emit(f"FAIL:{exc}")


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
        self._demo_worker: Optional[_DemoWorker] = None
        # Pick-head dwell timing for the demo. None = the driver's real-hardware
        # defaults; tests set a zero-dwell PickSequence to run instantly.
        self._demo_sequence: Optional[PickSequence] = None

        # This is an HMI — the whole page must NOT scroll. Keep the sections
        # compact and let only the (tall) parameter tables scroll internally.
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)
        top = QHBoxLayout()
        top.addWidget(self._build_connection(), 1)
        root.addLayout(top)
        # One status table (rows = signals, columns = Drive 0 / Drive 1) with the
        # jog pad to its right.
        mid = QHBoxLayout()
        mid.addWidget(self._build_status_table(), 3)
        mid.addWidget(self._build_jog(), 2)
        root.addLayout(mid)
        # Parameters take the remaining height (both tables expand vertically).
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
        self.crc_btn = QPushButton("Zero CRC ctrs")
        self.crc_btn.setToolTip(
            "Zero every slave's EtherCAT link/CRC error counters. Zero them, run "
            "a speed trial, and watch the 'Link errors (CRC)' row: anything that "
            "climbs under load is cable/connector/EMI (a bad link kicks a drive "
            "out of OP - the silent no-fault disable).")
        self.crc_btn.clicked.connect(self._on_zero_crc)
        for b in (self.connect_btn, self.sim_btn, self.disconnect_btn,
                  self.reset_btn, self.crc_btn):
            row.addWidget(b)
        row.addStretch(1)
        self.conn_label = QLabel("DISCONNECTED")
        self.conn_label.setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")
        row.addWidget(self.conn_label)
        g.addLayout(row, 3, 0, 1, 2)
        return box

    def _build_status_table(self) -> QGroupBox:
        """One table: rows are the per-drive signals (state, encoder, angle, err,
        following error, then each statusword/DI bit as plain ON/OFF), columns
        are Drive 0 and Drive 1."""
        box = QGroupBox("Drive status")
        v = QVBoxLayout(box)
        v.setContentsMargins(6, 6, 6, 6)
        t = QTableWidget(len(_STATUS_ROWS), 3)
        t.setHorizontalHeaderLabels(["Signal", "Drive 0", "Drive 1"])
        t.verticalHeader().setVisible(False)
        t.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        t.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        row_h = 19        # 18 rows (incl. the link/CRC row) in the old footprint
        t.verticalHeader().setDefaultSectionSize(row_h)
        # Show every row at once — no internal scroll.
        t.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        t.setMinimumHeight(row_h * len(_STATUS_ROWS) + 28)
        for r, (label, _kind) in enumerate(_STATUS_ROWS):
            t.setItem(r, 0, QTableWidgetItem(label))
            for c in (1, 2):
                cell = QTableWidgetItem("—")
                cell.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                t.setItem(r, c, cell)
        hdr = t.horizontalHeader()
        hdr.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.status_table = t
        v.addWidget(t)
        return box

    def _set_status_cell(self, row: int, col: int, text: str, danger: bool,
                         tip: str = "") -> None:
        item = self.status_table.item(row, col)
        if item is None:
            return
        item.setText(text)
        item.setForeground(QColor(theme.DANGER) if danger else QColor(theme.TEXT))
        item.setToolTip(tip)

    @staticmethod
    def _link_tooltip(lc) -> str:
        """Non-verdict detail for the link/CRC cell. Forwarded RX errors point
        at an UPSTREAM segment (an error another slave already flagged); PU/PDI
        aren't cable-CRC at all and read 0xFF (unimplemented) on some ESCs -
        which is why none of them count toward the headline verdict."""
        if not lc:
            return ""
        fwd = ", ".join(f"p{i}={p['forwarded']}" for i, p in enumerate(lc["ports"]))
        return ("forwarded RX (upstream): " + fwd
                + f"\nprocessing-unit err: {lc['pu_error']}"
                + f"\nPDI err: {lc['pdi_error']}"
                + "\n(forwarded points upstream; PU/PDI aren't cable-CRC and"
                  " read 0xFF = unimplemented on some drives)")

    def _build_jog(self) -> QGroupBox:
        """Cartesian TCP jog: X/Y +/- pad, increment in mm, speed in mm/s. Moves
        the tool in a validated straight line via the 5-bar kinematics; needs the
        robot referenced (home, or 'Reference here' on the bench)."""
        box = QGroupBox("Jog — Cartesian TCP  (motion)")
        g = QGridLayout(box)
        g.setContentsMargins(8, 6, 8, 6)
        g.setSpacing(4)
        self.enable_btn = QPushButton("Enable")
        self.enable_btn.clicked.connect(self._on_enable)
        self.disable_btn = QPushButton("Disable")
        self.disable_btn.clicked.connect(self._on_disable)
        self.ref_btn = QPushButton("Set Home")
        self.ref_btn.setToolTip("Drive each axis to its hard-stop home, then Set "
                                "Home to make this pose the datum (no switches). "
                                "Configured home_angles must match the hard stop.")
        self.ref_btn.clicked.connect(self._on_set_home)
        g.addWidget(self.enable_btn, 0, 0)
        g.addWidget(self.disable_btn, 0, 1)
        g.addWidget(self.ref_btn, 0, 2, 1, 2)
        # X/Y pad — Y+ up, Y- down, X- left, X+ right
        self.yplus_btn = QPushButton("Y +  ▲")
        self.yplus_btn.clicked.connect(lambda: self._on_cart_jog(0.0, +1.0))
        self.yminus_btn = QPushButton("Y −  ▼")
        self.yminus_btn.clicked.connect(lambda: self._on_cart_jog(0.0, -1.0))
        self.xminus_btn = QPushButton("◀  X −")
        self.xminus_btn.clicked.connect(lambda: self._on_cart_jog(-1.0, 0.0))
        self.xplus_btn = QPushButton("X +  ▶")
        self.xplus_btn.clicked.connect(lambda: self._on_cart_jog(+1.0, 0.0))
        g.addWidget(self.yplus_btn, 1, 1)
        g.addWidget(self.xminus_btn, 2, 0)
        g.addWidget(self.xplus_btn, 2, 2)
        g.addWidget(self.yminus_btn, 3, 1)
        g.addWidget(QLabel("increment (mm)"), 1, 3)
        self.jog_incr = QDoubleSpinBox()
        self.jog_incr.setRange(0.1, 100.0)
        self.jog_incr.setValue(5.0)
        self.jog_incr.setDecimals(1)
        self.jog_incr.setMaximumWidth(90)
        g.addWidget(self.jog_incr, 1, 4)
        g.addWidget(QLabel("speed (mm/s)"), 2, 3)
        self.jog_speed_mm = QDoubleSpinBox()
        # Also the demo's travel speed — uncapped for speed trials (the real
        # limiter at high settings becomes accel_mm_s2 in the Parameters table).
        self.jog_speed_mm.setRange(1.0, 5000.0)
        self.jog_speed_mm.setValue(50.0)
        self.jog_speed_mm.setMaximumWidth(90)
        g.addWidget(self.jog_speed_mm, 2, 4)
        warn = QLabel("Motion — E-stop/contactor live, robot referenced. Small increments first.")
        warn.setWordWrap(True)
        warn.setStyleSheet(f"color:{theme.WARN}; font-weight:600;")
        g.addWidget(warn, 4, 0, 1, 5)
        # Sample pick&place demo (vision bypass): pick a fixed nest, drop into a
        # variably-placed 6-hole cover row. Exercises the arm + pick head before
        # vision calibration exists. Needs the drives enabled + Set Home.
        self.sim_demo_btn = QPushButton("Simulate pick && place")
        self.sim_demo_btn.setToolTip(
            "Run a sample pick&place: pick from a fixed nest and drop into the six "
            "holes of a randomly-placed bung cover. Needs Enable + Set Home. "
            "Vision bypass — no camera/calibration required.")
        self.sim_demo_btn.clicked.connect(self._on_simulate)
        self.demo_loop_chk = QCheckBox("loop")
        self.demo_loop_chk.setToolTip("Keep repeating the demo (re-placing the "
                                      "cover each pass) until stopped.")
        g.addWidget(self.sim_demo_btn, 5, 0, 1, 3)
        g.addWidget(self.demo_loop_chk, 5, 3, 1, 2, Qt.AlignmentFlag.AlignLeft)
        # Rolling throughput readout (cycles/min over a trailing minute).
        self.demo_rate_label = QLabel("cycles/min: —")
        self.demo_rate_label.setToolTip("Rolling pick&place throughput — completed "
                                        "cover placements per minute over the last "
                                        "60 s.")
        g.addWidget(self.demo_rate_label, 6, 0, 1, 5)
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
        for b in (self.enable_btn, self.disable_btn, self.ref_btn,
                  self.xplus_btn, self.xminus_btn, self.yplus_btn, self.yminus_btn):
            b.setEnabled(on)

    def _on_set_home(self) -> None:
        drv = self._ec_driver()
        if drv is None:
            self._status("Connect the drives first.", theme.WARN)
            return
        try:
            drv.set_home()
            self._status("Home set at current pose — Cartesian jog enabled.", theme.TEXT)
        except Exception as exc:  # noqa: BLE001
            self._status(f"Set Home failed: {exc}", theme.DANGER)

    def _on_cart_jog(self, sx: float, sy: float) -> None:
        drv = self._ec_driver()
        if drv is None:
            self._status("Connect + enable the drives first.", theme.WARN)
            return
        if self._jog_worker is not None and self._jog_worker.isRunning():
            return
        incr = float(self.jog_incr.value())
        dx, dy = sx * incr, sy * incr
        speed = float(self.jog_speed_mm.value())
        # Run off the GUI thread so the poller keeps updating (following error).
        self._set_motion_enabled(False)
        self._status(f"Jogging TCP by ({dx:+.1f}, {dy:+.1f}) mm…", theme.TEXT)
        self._jog_worker = _JogWorker(
            lambda: drv.jog_cartesian(dx, dy, speed_mm_s=speed))
        self._jog_worker.done.connect(self._on_jog_done)
        self._jog_worker.start()

    def _on_jog_done(self, err: str) -> None:
        self._set_motion_enabled(True)
        if err:
            self._status(f"Jog failed: {err}", theme.DANGER)
        else:
            self._status("Jog complete.", theme.TEXT)

    # --- demo (sample pick & place) -----------------------------------------
    def _on_simulate(self) -> None:
        # Second press = stop a running demo.
        if self._demo_worker is not None and self._demo_worker.isRunning():
            self._demo_worker.stop()
            self.sim_demo_btn.setText("Stopping…")
            self.sim_demo_btn.setEnabled(False)
            return
        if self._jog_worker is not None and self._jog_worker.isRunning():
            self._status("Wait for the jog to finish before running the demo.",
                         theme.WARN)
            return
        ctrl = self.controller
        drv = ctrl.driver
        if not drv.is_enabled:
            self._status("Enable the drives before running the demo.", theme.WARN)
            return
        if not drv.is_referenced:
            self._status("Set Home before running the demo.", theme.WARN)
            return

        def make_targets():
            return demo_pick_and_place_targets(ctrl.validator, ctrl.home_xy)

        loop = self.demo_loop_chk.isChecked()
        # Travel at the (gentle) jog speed the operator has already dialled in, so
        # a big demo move can't outrun the servo and trip a position-deviation
        # alarm. The pick head still uses the real dwell timing.
        speed = float(self.jog_speed_mm.value())
        self._set_motion_enabled(False)          # no jog/enable while it runs
        self.sim_demo_btn.setText("Stop demo")
        self.demo_rate_label.setText("cycles/min: …")
        self._status(f"Running sample pick & place at {speed:.0f} mm/s…", theme.TEXT)
        self._demo_worker = _DemoWorker(ctrl, make_targets, loop,
                                        pick_sequence=self._demo_sequence,
                                        move_speed_mm_s=speed)
        self._demo_worker.step.connect(lambda m: self._status(m, theme.TEXT))
        self._demo_worker.stats.connect(self._on_demo_stats)
        self._demo_worker.done.connect(self._on_demo_done)
        self._demo_worker.start()

    def _on_demo_stats(self, cpm: float, total: int) -> None:
        self.demo_rate_label.setText(f"cycles/min: {cpm:.1f}   (placed {total})")

    def _on_demo_done(self, msg: str) -> None:
        self._set_motion_enabled(True)
        self.sim_demo_btn.setText("Simulate pick && place")
        self.sim_demo_btn.setEnabled(True)
        if msg.startswith("FAIL:"):
            self._status(f"Demo failed: {msg[5:]}", theme.DANGER)
        else:
            self._status(msg, theme.TEXT)

    def _stop_demo(self) -> None:
        if self._demo_worker is not None:
            self._demo_worker.stop()
            self._demo_worker.wait(5000)
            self._demo_worker = None
            # Restore the controls in case we stopped mid-run (e.g. disconnect).
            self.sim_demo_btn.setText("Simulate pick && place")
            self.sim_demo_btn.setEnabled(True)
            self._set_motion_enabled(True)

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
        self.custom_table = QTableWidget(0, 6)
        self.custom_table.setHorizontalHeaderLabels(
            ["Parameter", "Address", "Value", "Drive 0", "Drive 1", "Description"])
        # Description stretches to fill; the value/drive columns size to content
        # (so 'Drive 1' isn't wildly wide).
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
                    it.setToolTip(f"drive reads {self._fmt(v)}, setpoint {self._fmt(c.value)} — "
                                  "write didn't take (object may be read-only, state-gated, "
                                  "or overwritten by auto-tune). Verify the Cxx.NN address.")
                self.custom_table.setItem(r, 3 + di, it)
            desc_item = QTableWidgetItem(c.desc)
            desc_item.setFlags(desc_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            if c.desc:
                desc_item.setToolTip(c.desc)
            self.custom_table.setItem(r, 5, desc_item)
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
        # Build with the PERSISTED motion parameters — previously the stored
        # position tolerance only took effect after pressing Apply, so every
        # fresh Connect silently ran with the code default.
        driver = EtherCatRobotDriver(
            master, self.controller.kin, self.controller.validator,
            home_angles=homing.home_angles,
            limits=self.store.trajectory_limits(),
            position_tol_counts=int(self.store.get("position_tol_counts")),
            settle_timeout_s=float(self.store.get("settle_timeout_s")),
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

    def _on_zero_crc(self) -> None:
        master = self._master()
        reset = getattr(master, "reset_link_counters", None)
        if not callable(reset):
            self._status("Link counters need the real (IgH) master.", theme.WARN)
            return
        reset()
        self._status("Link/CRC counters zeroed - run a trial and watch the "
                     "'Link errors (CRC)' row.", theme.TEXT)

    def _stop_jog(self) -> None:
        if self._jog_worker is not None:
            self._jog_worker.wait(3000)
            self._jog_worker = None

    def _teardown_master(self) -> None:
        """Disable the drives (torque off) and stop the master/daemon. Safe to
        call when not connected."""
        self._stop_demo()
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
        # The last note is the "N written, M unchanged[, ignored/aborted]" summary.
        summary = notes[-1] if notes else "nothing to apply"
        bad = ("ignored" in summary) or ("aborted" in summary)
        if bad:
            # Surface the first failing detail (carries the CoE abort code).
            detail = next((n for n in notes if "ABORTED" in n or "ignored by drive" in n), "")
            self._status(f"Applied: {summary} — {detail}", theme.DANGER)
        else:
            self._status(f"Applied: {summary}", theme.TEXT)

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

    @staticmethod
    def _status_value(kind, d, deg, faulted):
        """(text, danger) for one signal row of one drive."""
        if kind == "state":
            return cia402.decode_state(d["sw"]).value.replace("_", " ").upper(), faulted
        if kind == "counts":
            return f"{d['act']:+d}", False
        if kind == "angle":
            return f"{deg:+.3f}", False
        if kind == "err":
            return f"0x{d.get('err', 0):04X}", False
        if kind == "fe":
            return f"{d.get('fe', 0):+d}", False
        if kind == "link":
            lc = d.get("link")
            if not lc:
                return "\u2014", False
            rx = sum(p["rx_error"] for p in lc["ports"])
            inv = sum(p["invalid_frame"] for p in lc["ports"])
            lost = sum(p["lost_link"] for p in lc["ports"])
            # rx-error + invalid-frame are THIS segment's CRC / physical-layer
            # counters - the ones that climb with a bad cable, so they alone set
            # the verdict. lost-link is shown for context (a couple at power-up /
            # connect is normal, not a fault). forwarded / PU / PDI live in the
            # cell tooltip: they point upstream or aren't cable-CRC at all, and
            # read 0xFF (unimplemented) on some drives - counting them made a
            # clean link show a bogus 510+ total and never reach "0 (clean)".
            crc = rx + inv
            if crc == 0 and lost == 0:
                return "0 (clean)", False
            return f"rx{rx} inv{inv} lost{lost}", crc > 0
        src, mask = kind
        on = bool((d["sw"] if src == "sw" else d["di"]) & mask)
        return ("ON" if on else "OFF"), False

    def _on_snapshot(self, snap) -> None:
        connected = snap is not None
        self.conn_label.setText("CONNECTED" if connected else "DISCONNECTED")
        self.conn_label.setStyleSheet(
            f"color:{theme.TEXT}; font-weight:600;" if connected
            else f"color:{theme.TEXT_DIM}; font-weight:600;")
        ppd = self.controller.kin.config.pulses_per_degree
        home_counts = getattr(self.controller.driver, "_home_counts", [0, 0])
        for i in range(2):                       # columns: Drive 0, Drive 1
            col = 1 + i
            if not connected or i >= len(snap):
                for r in range(len(_STATUS_ROWS)):
                    self._set_status_cell(r, col, "—", False)
                continue
            d = snap[i]
            home = home_counts[i] if i < len(home_counts) else 0
            deg = (d["act"] + home) / ppd
            faulted = cia402.is_fault(d["sw"])
            for r, (_label, kind) in enumerate(_STATUS_ROWS):
                text, danger = self._status_value(kind, d, deg, faulted)
                tip = self._link_tooltip(d.get("link")) if kind == "link" else ""
                self._set_status_cell(r, col, text, danger, tip)

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
