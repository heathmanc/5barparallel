"""Vision tab — the main operational screen.

Large overhead-camera view on the left; a live status panel and run controls on
the right. Detection overlays and the automatic cycle hook in here once
vision/detect_* and app/cycle_manager are built.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..app.robot_test_controller import RobotTestController
from ..robot.driver import DryRunRobotDriver
from ..vision.camera import Camera, CameraError
from . import theme
from .imaging import ndarray_to_qpixmap
from .widgets import ImageView, StatusPill


class VisionTab(QWidget):
    def __init__(
        self,
        controller: RobotTestController,
        camera: Camera,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.camera = camera

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        top.addLayout(self._build_view(), 1)
        top.addWidget(self._build_sidebar())
        root.addLayout(top, 1)

        self.status_label = QLabel("Ready.")
        self.status_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        root.addWidget(self.status_label)

        self._capture()
        self.refresh()

    # --- layout -------------------------------------------------------------
    def _build_view(self) -> QVBoxLayout:
        col = QVBoxLayout()
        header = QLabel("Overhead camera")
        header.setStyleSheet("font-size:15px; font-weight:600;")
        self.view = ImageView()
        col.addWidget(header)
        col.addWidget(self.view, 1)
        return col

    def _build_sidebar(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(260)
        v = QVBoxLayout(panel)
        v.setContentsMargins(0, 0, 0, 0)

        status_box = QGroupBox("System status")
        grid = QGridLayout(status_box)
        self.pill_drives = StatusPill()
        self.pill_home = StatusPill()
        self.pill_plc = StatusPill()
        self.pill_camera = StatusPill()
        for row, (name, pill) in enumerate(
            [("Drives", self.pill_drives), ("Home", self.pill_home),
             ("PLC", self.pill_plc), ("Camera", self.pill_camera)]
        ):
            lbl = QLabel(name)
            lbl.setStyleSheet(f"color:{theme.TEXT_DIM};")
            grid.addWidget(lbl, row, 0)
            grid.addWidget(pill, row, 1, Qt.AlignmentFlag.AlignRight)
        v.addWidget(status_box)

        run_box = QGroupBox("Run")
        rb = QVBoxLayout(run_box)
        self.capture_btn = QPushButton("Capture frame")
        self.capture_btn.clicked.connect(self._on_capture)
        self.start_btn = QPushButton("Start cycle")
        self.start_btn.setProperty("accent", "primary")
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setProperty("accent", "danger")
        self.stop_btn.clicked.connect(self._on_stop)
        for b in (self.capture_btn, self.start_btn, self.stop_btn):
            rb.addWidget(b)
        v.addWidget(run_box)
        v.addStretch(1)
        return panel

    # --- camera -------------------------------------------------------------
    def set_camera(self, camera: Camera) -> None:
        self.camera = camera
        self._capture()
        self.refresh()

    def _on_capture(self) -> None:
        self._capture()
        self._set_status("Frame captured.", theme.SUCCESS)

    def _capture(self) -> None:
        try:
            frame = self.camera.grab()
        except CameraError as exc:
            self._set_status(f"Capture failed: {exc}", theme.DANGER)
            return
        self.view.set_pixmap(ndarray_to_qpixmap(frame))

    # --- run (placeholder until cycle_manager exists) -----------------------
    def _on_start(self) -> None:
        self._set_status(
            "Automatic cycle not yet wired (app/cycle_manager + vision detection "
            "are still to build).",
            theme.WARN,
        )

    def _on_stop(self) -> None:
        self._set_status("Stopped.", theme.TEXT_DIM)

    # --- status -------------------------------------------------------------
    def refresh(self) -> None:
        enabled = self.controller.is_enabled
        referenced = self.controller.is_referenced
        self.pill_drives.set_state(
            "ENABLED" if enabled else "DISABLED", "ok" if enabled else "idle"
        )
        self.pill_home.set_state(
            "REFERENCED" if referenced else "NOT REFERENCED",
            "ok" if referenced else "bad",
        )
        drv = self.controller.driver
        if isinstance(drv, DryRunRobotDriver):
            self.pill_plc.set_state("DRY-RUN", "idle")
        else:
            self.pill_plc.set_state("CONNECTED", "info")
        self.pill_camera.set_state(
            "ONLINE" if self.camera.is_open else "OFFLINE",
            "ok" if self.camera.is_open else "bad",
        )

    def _set_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color:{color};")
