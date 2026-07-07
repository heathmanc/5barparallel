"""Bypass tab — bench-test overrides for a machine that isn't fully built.

Lets you feed the PLC the inputs the missing hardware would provide, so you can
enable/home/jog the motors on the table with no safety wiring, no home prox, and
no Z/vacuum sensors:

  * Safeties  — force the safety INPUT tags to their safe state (EStop_OK=1,
    Guard_Closed=1, limits/alarms=0). The safety *logic* stays intact; once you
    wire real safety I/O and alias those tags, the physical inputs override.
  * Homing    — set Bypass_Homing so R30 marks referenced instantly (no prox).
  * Vision    — set Bypass_Vision so R50 auto-satisfies the Z reed switches and
    vacuum sensor, letting the pick/place motion run open-loop.

Writes go to the connected PLC client (real CompactLogix or the simulated PLC).
BENCH ONLY — never run with people near the machine.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController

# Safety INPUT tags and their safe (bench) values — these are the base BOOLs the
# ladder reads; forcing them simulates healthy safety hardware.
SAFE_INPUTS = [
    ("EStop_OK", True),
    ("Guard_Closed", True),
    ("EStop_Pressed", False),
    ("Ax0_LimitMin", False),
    ("Ax0_LimitMax", False),
    ("Ax1_LimitMin", False),
    ("Ax1_LimitMax", False),
    ("EM806_0_ALM", False),
    ("EM806_1_ALM", False),
]


class BypassTab(QWidget):
    def __init__(self, controller: RobotTestController, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.controller = controller

        root = QVBoxLayout(self)

        warn = QLabel(
            "⚠  BENCH TEST ONLY — these bypasses defeat missing safety/sensor "
            "hardware. Never run with anyone near the machine, and clear all "
            "bypasses before commissioning the real cell."
        )
        warn.setWordWrap(True)
        warn.setStyleSheet(
            "color: #f85149; font-weight: bold; border: 1px solid #f85149; "
            "border-radius: 6px; padding: 8px;"
        )
        root.addWidget(warn)

        root.addWidget(self._build_safety_group())
        root.addWidget(self._build_homing_group())
        root.addWidget(self._build_vision_group())

        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)
        root.addStretch(1)

        self.refresh()

    # --- groups -------------------------------------------------------------
    def _build_safety_group(self) -> QGroupBox:
        box = QGroupBox("Safeties (force inputs SAFE)")
        v = QVBoxLayout(box)
        v.addWidget(QLabel(
            "Writes EStop_OK=1, Guard_Closed=1 and all limit/alarm inputs=0 so "
            "SafetyOK is true and the drives enable with no safety wiring."
        ))
        self.force_safe_btn = QPushButton("Force safeties SAFE")
        self.force_safe_btn.clicked.connect(self._on_force_safe)
        v.addWidget(self.force_safe_btn)
        return box

    def _build_homing_group(self) -> QGroupBox:
        box = QGroupBox("Homing")
        v = QVBoxLayout(box)
        self.homing_chk = QCheckBox("Skip homing (Bypass_Homing)")
        self.homing_chk.setToolTip(
            "R30 marks the robot referenced instantly on Home (find ref) — no home "
            "prox needed. Publishes the nominal home angles so jogging is allowed."
        )
        self.homing_chk.toggled.connect(self._on_homing_toggled)
        v.addWidget(self.homing_chk)
        return box

    def _build_vision_group(self) -> QGroupBox:
        box = QGroupBox("Vision / sensors")
        v = QVBoxLayout(box)
        self.vision_chk = QCheckBox("Bypass vision + Z / vacuum sensors (Bypass_Vision)")
        self.vision_chk.setToolTip(
            "R50 auto-satisfies the Z reed switches (PickDown/Up, DropDown/Up) and "
            "the vacuum sensor, so the automatic pick/place motion runs open-loop."
        )
        self.vision_chk.toggled.connect(self._on_vision_toggled)
        v.addWidget(self.vision_chk)
        return box

    # --- PLC client ---------------------------------------------------------
    def _client(self):
        """The connected PLC client, or None on the dry-run driver."""
        return getattr(self.controller.driver, "client", None)

    def _write(self, tag: str, value) -> bool:
        client = self._client()
        if client is None:
            return False
        try:
            client.write(tag, value)
            return True
        except Exception as exc:  # PlcError or comms drop
            self._set_status(f"Write {tag} failed: {exc}", ok=False)
            return False

    # --- handlers -----------------------------------------------------------
    def _on_force_safe(self) -> None:
        if self._client() is None:
            self._set_status("No PLC connected (PLC tab -> Connect).", ok=False)
            return
        failed = [tag for tag, val in SAFE_INPUTS if not self._write(tag, val)]
        if failed:
            self._set_status(f"Some safety writes failed: {', '.join(failed)}", ok=False)
        else:
            self._set_status(
                "Safeties forced SAFE — enable / home / jog now work with no safety "
                "I/O. Wire real safety hardware before production.",
                ok=True,
            )

    def _on_homing_toggled(self, checked: bool) -> None:
        if self._write("Bypass_Homing", bool(checked)):
            self._set_status(
                "Bypass_Homing ON — Home (find ref) marks referenced instantly."
                if checked else "Bypass_Homing OFF — real prox homing restored.",
                ok=checked,
            )

    def _on_vision_toggled(self, checked: bool) -> None:
        if self._write("Bypass_Vision", bool(checked)):
            self._set_status(
                "Bypass_Vision ON — auto-cycle Z/vacuum sensors auto-satisfied."
                if checked else "Bypass_Vision OFF — real sensors required.",
                ok=checked,
            )

    # --- refresh ------------------------------------------------------------
    def refresh(self) -> None:
        """Re-check the PLC connection (call on connectionChanged) and sync the
        toggles to the live tag values."""
        client = self._client()
        connected = client is not None
        for w in (self.force_safe_btn, self.homing_chk, self.vision_chk):
            w.setEnabled(connected)
        if not connected:
            self._set_status(
                "No PLC connected. Connect a real or simulated PLC on the PLC tab "
                "to use bench bypass.",
                ok=False,
            )
            return
        for chk, tag in ((self.homing_chk, "Bypass_Homing"),
                         (self.vision_chk, "Bypass_Vision")):
            try:
                chk.blockSignals(True)
                chk.setChecked(bool(client.read(tag)))
            except Exception:
                pass
            finally:
                chk.blockSignals(False)
        self._set_status(
            "Connected — bench bypass available. Use ONLY with no one near the machine.",
            ok=True,
        )

    def _set_status(self, text: str, *, ok: bool) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(
            "color: #3fb950;" if ok else "color: #f85149; font-weight: bold;"
        )
