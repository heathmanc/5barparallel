"""Vision tab — the main operational screen.

Large overhead-camera view on the left; a live status panel and run controls on
the right. Detection overlays and the automatic cycle hook in here once
vision/detect_* and app/cycle_manager are built.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, QThread
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from ..app.cycle_manager import CycleManager
from ..app.robot_test_controller import RobotTestController
from ..robot.driver import DryRunRobotDriver
from ..vision.camera import Camera, CameraError
from ..vision.detect_covers import CoverDetector
from ..vision.detect_holes import HoleDetector
from ..vision.detection import annotate
from . import theme
from .cycle_worker import CycleWorker
from .imaging import ndarray_to_qpixmap
from .widgets import ImageView, StatusPill


class VisionTab(QWidget):
    def __init__(
        self,
        controller: RobotTestController,
        camera: Camera,
        calibration=None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.camera = camera
        self.calibration = calibration  # HomographyTransform (pixel->robot) or None
        self.hole_detector = HoleDetector()
        self.cover_detector = CoverDetector()
        self._frame = None
        self._running = False
        self._thread: Optional[QThread] = None
        self._worker: Optional[CycleWorker] = None

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
        self.detect_btn = QPushButton("Detect")
        self.detect_btn.clicked.connect(self._on_detect)
        self.start_btn = QPushButton("Start cycle")
        self.start_btn.setProperty("accent", "primary")
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setProperty("accent", "danger")
        self.stop_btn.clicked.connect(self._on_stop)
        for b in (self.capture_btn, self.detect_btn, self.start_btn, self.stop_btn):
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
        self._frame = frame
        self.view.set_pixmap(ndarray_to_qpixmap(frame))

    def _on_detect(self) -> None:
        if self._frame is None:
            self._capture()
        if self._frame is None:
            return
        holes = self.hole_detector.detect(self._frame)
        # With a calibration, covers are also filtered by real workspace
        # reachability (pixel -> robot -> WorkspaceValidator).
        to_robot = self.calibration.pixel_to_robot if self.calibration else None
        validator = self.controller.validator if self.calibration else None
        covers = self.cover_detector.detect(self._frame, to_robot, validator)
        overlay = annotate(self._frame, holes.holes, covers.covers)
        self.view.set_pixmap(ndarray_to_qpixmap(overlay))
        collinear = "collinear ✓" if holes.ok else holes.reason
        reach = " reachable" if self.calibration else " pickable"
        self._set_status(
            f"{holes.count} holes ({collinear}) · {covers.count} covers, "
            f"{len(covers.accepted)}{reach}.",
            theme.SUCCESS if holes.ok else theme.WARN,
        )

    def set_calibration(self, calibration) -> None:
        self.calibration = calibration

    # --- automatic cycle ----------------------------------------------------
    def _on_start(self) -> None:
        if self._running:
            return
        self.refresh()
        manager = CycleManager(
            self.controller, self.camera, self.calibration,
            hole_detector=self.hole_detector, cover_detector=self.cover_detector,
        )
        block = manager.preflight()
        if block is not None:
            self._set_status(f"Cannot start: {block}", theme.WARN)
            return

        # Run off the GUI thread — a real PLC handshake takes seconds per hole.
        self._running = True
        self.start_btn.setEnabled(False)
        self._set_status("Running automatic cycle…", theme.INFO)

        self._thread = QThread(self)
        self._worker = CycleWorker(manager)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.stepDone.connect(self._on_cycle_step)
        self._worker.finished.connect(self._on_cycle_finished)
        self._thread.start()

    def _on_cycle_step(self, step) -> None:
        where = f"hole {step.hole_index}"
        if step.ok:
            self._set_status(f"Placed cover in {where}…", theme.INFO)
        else:
            self._set_status(f"{where}: {step.reason}", theme.WARN)

    def _on_cycle_finished(self, result) -> None:
        self._teardown_thread()
        self._running = False
        self.start_btn.setEnabled(True)
        self._show_overlay()
        self.refresh()
        placed = len(result.placed)
        color = theme.SUCCESS if result.ok else theme.WARN
        self._set_status(
            f"Cycle done — placed {placed} cover(s). {result.reason}", color
        )

    def _on_stop(self) -> None:
        if self._running and self._worker is not None:
            self._worker.request_stop()
            self.controller.stop()
            self._set_status("Stopping after the current pick…", theme.WARN)
        else:
            self._set_status("Stopped.", theme.TEXT_DIM)

    def _teardown_thread(self) -> None:
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait()
            self._worker.deleteLater()
            self._thread.deleteLater()
            self._thread = None
            self._worker = None

    def _show_overlay(self) -> None:
        """Redraw the current scene with detected holes + covers."""
        try:
            frame = self.camera.grab()
        except CameraError:
            return
        self._frame = frame
        holes = self.hole_detector.detect(frame)
        to_robot = self.calibration.pixel_to_robot if self.calibration else None
        validator = self.controller.validator if self.calibration else None
        covers = self.cover_detector.detect(frame, to_robot, validator)
        overlay = annotate(frame, holes.holes, covers.covers)
        self.view.set_pixmap(ndarray_to_qpixmap(overlay))

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
