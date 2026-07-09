"""Vision tab — the main operational screen.

Large overhead-camera view on the left; a live status panel and run controls on
the right. Detection overlays and the automatic cycle hook in here once
vision/detect_* and app/cycle_manager are built.
"""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..app.cycle_manager import (
    CycleConfig,
    CycleManager,
    ScriptedTargetSource,
    default_scripted_targets,
)
from ..app.recipes import RecipeStore
from ..app.robot_test_controller import RobotTestController
from ..robot.driver import DryRunRobotDriver
from ..vision.camera import Camera, CameraError
from ..vision.detect_covers import CoverDetector, CoverDetectorConfig
from ..vision.detect_holes import HoleDetector, HoleDetectorConfig
from ..vision.detection import (
    annotate,
    draw_reachable_zone,
    draw_robot_grid,
    reachable_zone_contours,
)
from . import theme
from .cycle_worker import CycleWorker
from .imaging import ndarray_to_qpixmap
from .widgets import RoiImageView, StatusPill


class VisionTab(QWidget):
    # emitted when the operator changes the active recipe (changeover)
    recipeChanged = Signal(str)

    def __init__(
        self,
        controller: RobotTestController,
        camera: Camera,
        calibration=None,
        recipes: Optional[RecipeStore] = None,
        calibration_manager=None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.controller = controller
        self.camera = camera
        self.calibration = calibration  # HomographyTransform (pixel->robot) or None
        self.recipes = recipes
        self.calibration_manager = calibration_manager  # for per-recipe pick ROI
        self.hole_detector = HoleDetector()
        # Covers are detected with Hough circles (robust for a round cover on
        # grainy wood); size defaults fit a large cover. Tunable live in the sidebar.
        self.cover_detector = CoverDetector(
            CoverDetectorConfig(method="hough", min_diameter_px=250, max_diameter_px=400,
                                reject_crowded=False))   # chute feeds one cover at a time
        self._pick_roi = None           # (x, y, w, h) px — covers must be inside
        self._frame = None
        self._display = None            # last rendered image (raw or overlay), for Save
        self._last_covers = None        # last CoverDetectionResult (for diagnostics)
        self._last_holes = None         # last HoleDetectionResult (for diagnostics)
        self._reach_cache = None        # safe-zone outline in robot mm (computed once)
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
        self.view = RoiImageView()
        self.view.roiChanged.connect(self._on_roi_changed)
        self.view.roiCleared.connect(self._on_roi_cleared)
        col.addWidget(header)
        col.addWidget(self.view, 1)
        return col

    def _build_sidebar(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(260)
        v = QVBoxLayout(panel)
        v.setContentsMargins(0, 0, 0, 0)

        recipe_box = QGroupBox("Recipe (changeover)")
        rv = QVBoxLayout(recipe_box)
        self.recipe_combo = QComboBox()
        if self.recipes is not None:
            for r in self.recipes.list():
                self.recipe_combo.addItem(r.name, r.key)
        self.recipe_combo.currentIndexChanged.connect(self._on_recipe_changed)
        self.recipe_combo.setEnabled(self.recipes is not None)
        rv.addWidget(self.recipe_combo)
        v.addWidget(recipe_box)

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
        self.save_frame_btn = QPushButton("Save frame…")
        self.save_frame_btn.setToolTip(
            "Write the current raw frame to a PNG (for tuning / sharing)."
        )
        self.save_frame_btn.clicked.connect(self._on_save_frame)
        self.save_diag_btn = QPushButton("Save diagnostics…")
        self.save_diag_btn.setToolTip(
            "Write the annotated overlay PNG plus a text report — pick region, "
            "calibration, and every detected cover/hole with pixel + robot (x,y) "
            "coordinates and its accept/reject reason. Run Detect first."
        )
        self.save_diag_btn.clicked.connect(self._on_save_diag)
        self.detect_btn = QPushButton("Detect")
        self.detect_btn.clicked.connect(self._on_detect)
        self.start_btn = QPushButton("Start cycle")
        self.start_btn.setProperty("accent", "primary")
        self.start_btn.clicked.connect(self._on_start)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setProperty("accent", "danger")
        self.stop_btn.clicked.connect(self._on_stop)
        for b in (self.capture_btn, self.save_frame_btn, self.save_diag_btn,
                  self.detect_btn, self.start_btn, self.stop_btn):
            rb.addWidget(b)
        self.bypass_chk = QCheckBox("Bypass vision (scripted)")
        self.bypass_chk.setToolTip(
            "Run the cycle against fixed, reachable robot-frame targets — no "
            "camera, detection, or calibration. Tests plan → PLC handshake."
        )
        rb.addWidget(self.bypass_chk)
        self.single_step_chk = QCheckBox("Single step (1 hole)")
        self.single_step_chk.setToolTip(
            "Run exactly one pick/place then stop — for safely shaking out the "
            "cycle on real hardware before letting it loop."
        )
        rb.addWidget(self.single_step_chk)
        v.addWidget(run_box)

        # Pick region — only covers inside the drawn box are picked. Enabled once
        # the recipe is calibrated (the scene is fixed after calibration).
        roi_box = QGroupBox("Pick region")
        rob = QVBoxLayout(roi_box)
        self.draw_roi_btn = QPushButton("Draw pick region")
        self.draw_roi_btn.setToolTip(
            "Drag a rectangle on the image around where the covers are. Covers "
            "outside it are ignored. Calibrate the recipe first."
        )
        self.draw_roi_btn.clicked.connect(self._on_draw_roi)
        self.clear_roi_btn = QPushButton("Clear pick region")
        self.clear_roi_btn.clicked.connect(self._on_clear_roi)
        rob.addWidget(self.draw_roi_btn)
        rob.addWidget(self.clear_roi_btn)
        v.addWidget(roi_box)

        v.addWidget(self._build_tuning_group())

        v.addStretch(1)
        return panel

    # --- live Hough tuning --------------------------------------------------
    def _build_tuning_group(self) -> QGroupBox:
        box = QGroupBox("Cover detection (Hough)")
        box.setToolTip(
            "Live-tune the Hough circle detector on the current frame. Set Min/Max "
            "Ø to your cover's pixel size; each change re-detects immediately."
        )
        grid = QGridLayout(box)
        cfg = self.cover_detector.config
        self.tune_min = self._tune_row(grid, 0, "Min Ø px", 20, 800, int(cfg.min_diameter_px))
        self.tune_max = self._tune_row(grid, 1, "Max Ø px", 20, 800, int(cfg.max_diameter_px))
        self.tune_edge = self._tune_row(grid, 2, "Edge sens", 0, 100, 70)
        self.tune_votes = self._tune_row(grid, 3, "Votes", 20, 90, int(cfg.hough_param2))
        for s in (self.tune_min, self.tune_max, self.tune_edge, self.tune_votes):
            s.valueChanged.connect(self._on_tuning_changed)
        self.show_holes_chk = QCheckBox("Show drop holes")
        self.show_holes_chk.setChecked(True)
        self.show_holes_chk.setToolTip(
            "Overlay the detected drop holes (where the bungs go) — gated on the "
            "recipe's hole diameter, separate from the cover size."
        )
        self.show_holes_chk.toggled.connect(self._on_tuning_changed)
        grid.addWidget(self.show_holes_chk, 4, 0, 1, 3)
        # Drop holes are a SEPARATE (smaller) size from the covers, so they get
        # their own pixel-Ø window — the cover window (250-400) would miss them.
        self.tune_hole_min = self._tune_row(grid, 5, "Hole min Ø", 15, 500, 30)
        self.tune_hole_max = self._tune_row(grid, 6, "Hole max Ø", 15, 500, 220)
        for s in (self.tune_hole_min, self.tune_hole_max):
            s.valueChanged.connect(self._on_tuning_changed)
        return box

    def _tune_row(self, grid: QGridLayout, row: int, label: str,
                  lo: int, hi: int, val: int) -> QSlider:
        grid.addWidget(QLabel(label), row, 0)
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(lo, hi)
        slider.setValue(val)
        readout = QLabel(str(val))
        readout.setFixedWidth(32)
        slider.valueChanged.connect(lambda v, r=readout: r.setText(str(v)))
        grid.addWidget(slider, row, 1)
        grid.addWidget(readout, row, 2)
        return slider

    def _apply_tuning(self) -> None:
        """Push the slider values into the Hough cover-detector config."""
        cfg = self.cover_detector.config
        cfg.method = "hough"
        cfg.min_diameter_px = float(self.tune_min.value())
        cfg.max_diameter_px = float(max(self.tune_max.value(), self.tune_min.value() + 1))
        s = self.tune_edge.value()               # 0 (insensitive) .. 100 (sensitive)
        # Edge sens softens only the edge detector; selectivity is a SEPARATE knob
        # (Votes) so cranking sensitivity for a soft edge can't flood the image
        # with weak grain circles.
        cfg.hough_param1 = max(20.0, 170.0 - 1.4 * s)   # more sens -> softer edges
        cfg.hough_param2 = float(self.tune_votes.value())

    def _hole_size_window(self) -> tuple:
        """Drop-hole (min, max) pixel Ø from the sliders, or a broad default."""
        if not hasattr(self, "tune_hole_min"):
            return 15.0, 220.0
        lo = float(self.tune_hole_min.value())
        hi = float(max(self.tune_hole_max.value(), lo + 1))
        return lo, hi

    def _apply_hole_tuning(self) -> None:
        """Push the drop-hole pixel-Ø window into the current hole detector."""
        lo, hi = self._hole_size_window()
        self.hole_detector.config.min_diameter_px = lo
        self.hole_detector.config.max_diameter_px = hi

    def _on_tuning_changed(self) -> None:
        self._apply_tuning()
        self._apply_hole_tuning()
        if self._frame is not None:
            self._on_detect()

    def _on_save_frame(self) -> None:
        if self._frame is None:
            self._capture()
        img = self._display if self._display is not None else self._frame
        if img is None:
            self._set_status("No frame to save (connect the camera).", theme.WARN)
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save image", "frame.png", "PNG (*.png)")
        if not path:
            return
        import cv2
        cv2.imwrite(path, img)
        self._set_status(f"Saved to {path}", theme.SUCCESS)

    def _on_save_diag(self) -> None:
        """Save the annotated overlay + a text report of the current detection, so
        the whole picture (pick region, calibration, per-object robot coords and
        reject reasons) can be reviewed from one bundle."""
        if self._frame is None:
            self._capture()
        img = self._display if self._display is not None else self._frame
        if img is None:
            self._set_status("No frame to save (connect the camera).", theme.WARN)
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save diagnostics", "diagnostics.png", "PNG (*.png)")
        if not path:
            return
        import cv2
        cv2.imwrite(path, img)
        txt_path = (path[:-4] if path.lower().endswith(".png") else path) + ".txt"
        try:
            with open(txt_path, "w", encoding="utf-8") as fh:
                fh.write(self._build_diag_report())
        except OSError as exc:
            self._set_status(f"Saved image; report failed: {exc}", theme.WARN)
            return
        self._set_status(f"Saved {path} + {txt_path}", theme.SUCCESS)

    def _build_diag_report(self) -> str:
        """Human-readable snapshot of the last Detect: settings + every object."""
        lines = ["5-Bar vision diagnostics", "=" * 44]
        lines.append(f"Recipe: {self.active_recipe_key()}  ({self.recipe_combo.currentText()})")
        cal = self.calibration
        lines.append(f"Calibration: {'present' if cal is not None else 'NONE'}"
                     f" ({type(cal).__name__ if cal is not None else '-'})")
        lines.append(f"Pick region (x,y,w,h px): {self._pick_roi}")

        cc = self.cover_detector.config
        lines += ["", "Cover detector:",
                  f"  method={cc.method}  Ø {cc.min_diameter_px:.0f}-{cc.max_diameter_px:.0f} px"
                  f"  param1={cc.hough_param1:.0f} param2={cc.hough_param2:.0f}",
                  f"  expected {cc.expected_diameter_mm:.1f} mm ± {cc.diameter_tolerance*100:.0f}%"
                  f"  reject_crowded={cc.reject_crowded}"]

        hc = self.hole_detector.config
        lines += ["Hole detector:",
                  f"  method={hc.method}  Ø {hc.min_diameter_px:.0f}-{hc.max_diameter_px:.0f} px"
                  f"  expected_count={hc.expected_count}"
                  f"  expected {hc.expected_diameter_mm:.1f} mm ± {hc.diameter_tolerance*100:.0f}%",
                  f"  exclude_roi(px)={hc.exclude_roi}  subset={hc.select_collinear_subset}"
                  f"  auto_battery_roi={hc.auto_battery_roi}"]

        covers = self._last_covers
        lines.append("")
        if covers is not None:
            lines.append(f"Covers: {covers.count} detected, {len(covers.accepted)} accepted")
            for i, cd in enumerate(covers.covers):
                c = cd.circle
                rob = (f"robot({cd.robot_xy[0]:.1f},{cd.robot_xy[1]:.1f})mm"
                       if cd.robot_xy is not None else "robot(--)")
                lines.append(f"  #{i} px({c.cx:.0f},{c.cy:.0f}) Ø{c.diameter:.0f} {rob}"
                             f"  {'ACCEPT' if cd.accepted else 'reject'} :: {cd.reason}")
        else:
            lines.append("Covers: (run Detect first)")

        holes = self._last_holes
        lines.append("")
        if holes is not None:
            lines.append(f"Holes: {holes.count} detected, ok={holes.ok}"
                         f" residual={holes.max_residual_px:.1f}px :: {holes.reason}")
            to_robot = cal.pixel_to_robot if cal is not None else None
            for i, h in enumerate(holes.holes):
                if to_robot is not None:
                    rx, ry = to_robot(h.cx, h.cy)
                    rob = f"robot({rx:.1f},{ry:.1f})mm"
                else:
                    rob = "robot(--)"
                lines.append(f"  #{i} px({h.cx:.0f},{h.cy:.0f}) Ø{h.diameter:.0f} {rob}")
        else:
            lines.append("Holes: (drop-hole overlay off, or Detect not yet run)")
        return "\n".join(lines) + "\n"

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
        self._display = frame
        self.view.set_pixmap(ndarray_to_qpixmap(frame))
        self._update_roi_buttons()      # a frame is enough to draw a pick region

    def _on_detect(self) -> None:
        if self._frame is None:
            self._capture()
        if self._frame is None:
            return
        # With a calibration, covers are also filtered by real workspace
        # reachability (pixel -> robot -> WorkspaceValidator).
        to_robot = self.calibration.pixel_to_robot if self.calibration else None
        validator = self.controller.validator if self.calibration else None
        covers = self.cover_detector.detect(self._frame, to_robot, validator)
        show_holes = getattr(self, "show_holes_chk", None) and self.show_holes_chk.isChecked()
        holes = self.hole_detector.detect(self._frame, to_robot) if show_holes else None
        self._last_covers = covers
        self._last_holes = holes
        base = self._frame
        if self.calibration is not None:
            def _r2p(x, y):
                p = self.calibration.robot_to_pixel_many([[x, y]])[0]
                return (float(p[0]), float(p[1]))
            base = draw_robot_grid(
                self._frame, self.calibration.pixel_to_robot, _r2p, 25.0)
            base = draw_reachable_zone(base, self._reachable_contours(), _r2p)
        overlay = annotate(base, holes.holes if holes else None, covers.covers)
        self._display = overlay
        self.view.set_pixmap(ndarray_to_qpixmap(overlay))
        reach = " reachable" if self.calibration else " pickable"
        why = ""
        if covers.count and not covers.accepted:
            why = f" — rejected: {covers.covers[0].reason}"
        holes_txt = f"{holes.count} holes · " if holes else ""
        self._set_status(
            f"{holes_txt}{covers.count} covers, {len(covers.accepted)}{reach}{why}.",
            theme.SUCCESS if covers.accepted else theme.WARN,
        )

    def _reachable_contours(self):
        """Safe-zone outline in robot mm — computed once (geometry is fixed)."""
        if self._reach_cache is None:
            self._reach_cache = reachable_zone_contours(
                self.controller.validator.is_safe, -300.0, 300.0, 40.0, 430.0, 4.0)
        return self._reach_cache

    def set_calibration(self, calibration) -> None:
        self.calibration = calibration
        self._load_pick_roi()          # each recipe owns its pick region
        self._update_roi_buttons()

    # --- pick region --------------------------------------------------------
    def _apply_pick_roi(self, roi) -> None:
        self._pick_roi = roi
        self.cover_detector.config.pick_roi = roi
        # The pick region is the cover chute — exclude it from drop-hole search so
        # everything *outside* it is considered a drop target.
        self.hole_detector.config.exclude_roi = roi

    def _load_pick_roi(self) -> None:
        """Load the active recipe's saved pick region (if any) into the detector
        and the view — silently, without re-persisting."""
        key = self.active_recipe_key()
        roi = None
        if self.calibration_manager is not None and key:
            roi = self.calibration_manager.get_roi(key)
        self._apply_pick_roi(roi)
        self.view.set_roi(roi)

    def _on_draw_roi(self) -> None:
        # Drawing a pixel box needs a frame, not a calibration — a pick region is
        # useful (and now the color-agnostic cover search area) with no cal at all.
        if self._frame is None:
            self._capture()
        if self._frame is None:
            self._set_status("Grab a frame first (connect the camera).", theme.WARN)
            return
        self.view.set_draw_enabled(True)
        self._set_status("Drag a rectangle around the covers to pick.", theme.INFO)

    def _on_clear_roi(self) -> None:
        self.view.clear_roi()          # emits roiCleared -> _on_roi_cleared

    def _on_roi_changed(self, x: float, y: float, w: float, h: float) -> None:
        roi = (int(round(x)), int(round(y)), int(round(w)), int(round(h)))
        self._apply_pick_roi(roi)
        key = self.active_recipe_key()
        if self.calibration_manager is not None and key:
            self.calibration_manager.save_roi(key, roi)
        self._update_roi_buttons()
        self._on_detect()              # re-classify covers against the new region
        self._set_status(
            f"Pick region set ({roi[2]}×{roi[3]} px) — covers outside it are skipped.",
            theme.SUCCESS,
        )

    def _on_roi_cleared(self) -> None:
        self._apply_pick_roi(None)
        key = self.active_recipe_key()
        if self.calibration_manager is not None and key:
            self.calibration_manager.clear_roi(key)
        self._update_roi_buttons()
        if self._frame is not None:
            self._on_detect()
        self._set_status(
            "Pick region cleared — covers pickable anywhere reachable.", theme.TEXT_DIM
        )

    def _update_roi_buttons(self) -> None:
        can_draw = self._frame is not None or (
            self.camera is not None and self.camera.is_open)
        self.draw_roi_btn.setEnabled(can_draw)
        self.draw_roi_btn.setToolTip(
            "Drag a rectangle around where the covers are; covers outside it are "
            "ignored." if can_draw
            else "Connect a camera or grab a frame first."
        )
        self.clear_roi_btn.setEnabled(self._pick_roi is not None)

    # --- recipe (changeover) ------------------------------------------------
    def active_recipe_key(self) -> Optional[str]:
        key = self.recipe_combo.currentData()
        return str(key) if key is not None else None

    def select_recipe(self, key: str) -> None:
        """Set the changeover combo to ``key`` without re-emitting recipeChanged."""
        idx = self.recipe_combo.findData(key)
        if idx >= 0:
            self.recipe_combo.blockSignals(True)
            self.recipe_combo.setCurrentIndex(idx)
            self.recipe_combo.blockSignals(False)

    def _on_recipe_changed(self) -> None:
        key = self.active_recipe_key()
        if key is not None:
            self.recipeChanged.emit(key)

    def reload_recipes(self) -> None:
        """Repopulate the changeover combo from the store (e.g. after a recipe is
        added elsewhere), keeping the current selection if it still exists."""
        if self.recipes is None:
            return
        current = self.active_recipe_key()
        self.recipe_combo.blockSignals(True)
        self.recipe_combo.clear()
        for r in self.recipes.list():
            self.recipe_combo.addItem(r.name, r.key)
        idx = self.recipe_combo.findData(current)
        if idx >= 0:
            self.recipe_combo.setCurrentIndex(idx)
        self.recipe_combo.blockSignals(False)

    def set_hole_count(self, count: int, diameter_mm: float = 0.0,
                       tolerance: float = 0.25) -> None:
        """Apply the recipe's vent-hole count and drop-hole diameter (a separate
        size from the cover — a shouldered bung is wider than its hole).

        Drop holes are searched across the whole frame (not confined to a bright
        battery blob, which would lock onto the covers) and the pick region is
        excluded; the straight row is kept via collinear-subset selection."""
        lo, hi = self._hole_size_window()
        self.hole_detector = HoleDetector(HoleDetectorConfig(
            expected_count=count, expected_diameter_mm=diameter_mm,
            diameter_tolerance=tolerance, method="shape", auto_battery_roi=False,
            select_collinear_subset=True, exclude_roi=self._pick_roi,
            min_diameter_px=lo, max_diameter_px=hi))

    def set_cover_diameter_mm(self, diameter_mm: float, tolerance: float = 0.25) -> None:
        """Apply the active recipe's nominal cover (bung) size + size tolerance as a
        physical-size gate (needs a calibration to measure; 0 leaves covers ungated
        on size). Preserves the current pick region."""
        self.cover_detector = CoverDetector(
            CoverDetectorConfig(
                method="hough",
                min_diameter_px=250,
                max_diameter_px=400,
                reject_crowded=False,          # chute feeds one cover at a time
                expected_diameter_mm=diameter_mm,
                diameter_tolerance=tolerance,
                pick_roi=self._pick_roi,
            )
        )
        if hasattr(self, "tune_min"):    # keep any live-tuned Hough settings
            self._apply_tuning()

    # --- automatic cycle ----------------------------------------------------
    def _on_start(self) -> None:
        if self._running:
            return
        self.refresh()
        # Vision bypass: run the cycle against fixed, reachable targets so the
        # plan -> PLC handshake loop can be tested with no camera/detection.
        source = None
        if self.bypass_chk.isChecked():
            holes, covers = default_scripted_targets(self.controller)
            source = ScriptedTargetSource(holes, covers)
        config = CycleConfig(max_holes=1) if self.single_step_chk.isChecked() else None
        manager = CycleManager(
            self.controller, self.camera, self.calibration,
            hole_detector=self.hole_detector, cover_detector=self.cover_detector,
            target_source=source, config=config,
        )
        block = manager.preflight()
        if block is not None:
            self._set_status(f"Cannot start: {block}", theme.WARN)
            return

        # Run off the GUI thread — a real PLC handshake takes seconds per hole.
        self._running = True
        self.start_btn.setEnabled(False)
        mode = "scripted (vision bypass)" if source else "automatic"
        if self.single_step_chk.isChecked():
            mode += ", single step"
        self._set_status(f"Running {mode} cycle…", theme.INFO)

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
        self._update_roi_buttons()

    def _set_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color:{color};")
