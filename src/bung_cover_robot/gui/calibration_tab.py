"""Calibration tab — build a real pixel->robot calibration interactively.

Workflow (Claude.md §13):
  1. Pick the recipe (battery type) you're calibrating, place a target with known
     robot-frame coordinates in view, and capture a frame.
  2. Click each known point in the image; type its robot X/Y (mm) in the table.
  3. Fit the homography (>= 4 points) — the RMS residual reports the quality.
  4. Save it — the calibration is stored *per recipe* (calibration/<key>.npy).

Each recipe owns its own calibration; a changeover loads the right one. The
math/persistence lives in vision/calibration; this tab is a thin operator view.
A saved calibration is broadcast (recipe key + transform) so the Vision tab picks
it up live when it's the active recipe.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextBrowser,
    QVBoxLayout,
    QWidget,
)

from ..app.recipes import Recipe, RecipeError, RecipeStore, slugify_key
from ..vision.calibration import (
    CalibrationError,
    CalibrationManager,
    HomographyTransform,
)
from ..vision.camera import Camera, CameraError
from . import theme
from .calibration_help import HELP_HTML
from .imaging import ndarray_to_qpixmap
from .widgets import ZoomPanImageView


_TOTAL_STEPS = 6


class CoachBanner(QFrame):
    """A state-driven guidance strip that walks the operator through calibration:
    a 'Step N of 6' progress row (with dots), a bold instruction, and a dim tip.
    It reflects live state — it never traps the operator in a modal wizard."""

    helpRequested = Signal()
    hideToggled = Signal(bool)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setObjectName("coach")
        self._hidden = False
        v = QVBoxLayout(self)
        v.setContentsMargins(12, 8, 12, 8)
        v.setSpacing(4)

        top = QHBoxLayout()
        self.step_label = QLabel("Step 1 of 6")
        self.step_label.setStyleSheet("font-weight:600;")
        self.dots = QLabel()
        top.addWidget(self.step_label)
        top.addWidget(self.dots)
        top.addStretch(1)
        self.help_btn = QPushButton("? Help")
        self.help_btn.setToolTip("How to calibrate — process, tools, and tips")
        self.help_btn.clicked.connect(self.helpRequested)
        self.hide_btn = QPushButton("Hide guide")
        self.hide_btn.setCheckable(True)
        self.hide_btn.toggled.connect(self._on_hide)
        top.addWidget(self.help_btn)
        top.addWidget(self.hide_btn)
        v.addLayout(top)

        self.main_label = QLabel()
        self.main_label.setWordWrap(True)
        self.main_label.setStyleSheet("font-weight:600;")
        self.tip_label = QLabel()
        self.tip_label.setWordWrap(True)
        self.tip_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        v.addWidget(self.main_label)
        v.addWidget(self.tip_label)
        self._apply_kind(theme.INFO)

    def _on_hide(self, checked: bool) -> None:
        self._hidden = checked
        self.hide_btn.setText("Show guide" if checked else "Hide guide")
        self.main_label.setVisible(not checked)
        self.tip_label.setVisible(not checked)
        self.hideToggled.emit(checked)

    def _apply_kind(self, color: str) -> None:
        self.setStyleSheet(
            f"#coach {{ background:{theme.PANEL}; border:1px solid {color};"
            f" border-left:4px solid {color}; border-radius:8px; }}"
        )

    def set_state(self, step: int, main: str, tip: str, color: str) -> None:
        self.step_label.setText(f"Step {step} of {_TOTAL_STEPS}")
        dots = []
        for i in range(1, _TOTAL_STEPS + 1):
            dots.append("●" if i < step else "◉" if i == step else "○")
        self.dots.setText("  " + " ".join(dots))
        self.main_label.setText(main)
        self.tip_label.setText(tip)
        self.tip_label.setVisible(bool(tip) and not self._hidden)
        self._apply_kind(color)


class CalibrationTab(QWidget):
    """Collect pixel<->robot correspondences and save a per-recipe calibration."""

    # emits (recipe_key, HomographyTransform) when a calibration is saved
    calibrationSaved = Signal(str, object)
    recipesChanged = Signal()  # a recipe was added to the shared store

    def __init__(
        self,
        camera: Camera,
        manager: Optional[CalibrationManager] = None,
        recipes: Optional[RecipeStore] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.camera = camera
        self.manager = manager or CalibrationManager()
        self.recipes = recipes or RecipeStore()
        self._frame = None
        self._base_pix = None            # base QPixmap, built once per capture
        self._points: List[Tuple[float, float]] = []
        self._fitted: Optional[HomographyTransform] = None
        self._saved_key: Optional[str] = None  # recipe last saved this session

        root = QHBoxLayout(self)
        root.addLayout(self._build_view_column(), 1)
        root.addWidget(self._build_side_panel())

        self._reload_recipes()
        self._capture()

    # --- layout -------------------------------------------------------------
    def _build_view_column(self) -> QVBoxLayout:
        col = QVBoxLayout()
        self.coach = CoachBanner()
        self.coach.helpRequested.connect(self._show_help)
        col.addWidget(self.coach)
        self.view = ZoomPanImageView()
        self.view.pixelClicked.connect(self._on_pixel_clicked)
        self.view.zoomChanged.connect(self._on_zoom_changed)
        col.addWidget(self.view, 1)

        # Zoom toolbar
        zrow = QHBoxLayout()
        for label, slot, tip in (
            ("−", self.view.zoom_out, "Zoom out"),
            ("+", self.view.zoom_in, "Zoom in"),
            ("Fit", self.view.fit, "Fit the whole frame"),
            ("1:1", self.view.one_to_one, "One screen pixel per image pixel"),
        ):
            b = QPushButton(label)
            b.setFixedWidth(48)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            zrow.addWidget(b)
        self.loupe_check = QCheckBox("Loupe")
        self.loupe_check.setChecked(True)
        self.loupe_check.setToolTip("Magnifier at the cursor for pixel-precise picks")
        self.loupe_check.toggled.connect(self.view.set_loupe)
        zrow.addWidget(self.loupe_check)
        zrow.addStretch(1)
        self.zoom_label = QLabel("100%")
        self.zoom_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        zrow.addWidget(self.zoom_label)
        col.addLayout(zrow)

        self.status_label = QLabel("Capture a frame to begin.")
        self.status_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        col.addWidget(self.status_label)
        return col

    def _on_zoom_changed(self, pct: float) -> None:
        self.zoom_label.setText(f"{pct:.0f}%")

    def _show_help(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Calibration help")
        dlg.resize(660, 720)
        v = QVBoxLayout(dlg)
        browser = QTextBrowser()
        browser.setOpenExternalLinks(False)
        browser.setHtml(HELP_HTML)
        v.addWidget(browser)
        close = QPushButton("Close")
        close.clicked.connect(dlg.accept)
        v.addWidget(close)
        dlg.exec()

    def _build_side_panel(self) -> QWidget:
        panel = QWidget()
        panel.setFixedWidth(360)
        v = QVBoxLayout(panel)
        v.setContentsMargins(0, 0, 0, 0)

        # Recipe selection — calibration is stored per recipe (battery type).
        tgt = QGroupBox("Recipe (battery type)")
        tg = QVBoxLayout(tgt)
        self.recipe_combo = QComboBox()
        self.recipe_combo.currentIndexChanged.connect(self._on_recipe_changed)
        tg.addWidget(self.recipe_combo)
        self.recipe_status = QLabel()
        self.recipe_status.setStyleSheet(f"color:{theme.TEXT_DIM};")
        tg.addWidget(self.recipe_status)
        nrow = QHBoxLayout()
        self.new_recipe_edit = QLineEdit()
        self.new_recipe_edit.setPlaceholderText("New recipe name…")
        add_btn = QPushButton("Add")
        add_btn.clicked.connect(self._on_add_recipe)
        nrow.addWidget(self.new_recipe_edit)
        nrow.addWidget(add_btn)
        tg.addLayout(nrow)
        capture_btn = QPushButton("Capture frame")
        capture_btn.clicked.connect(self._on_capture)
        tg.addWidget(capture_btn)
        v.addWidget(tgt)

        # Recipe parameters — the process values that vary by battery type. These
        # drive the detectors (hole count; cover/bung size gate). Editable so a new
        # battery type is fully configurable from the app.
        params = QGroupBox("Recipe parameters")
        form = QFormLayout(params)
        self.hole_count_spin = QSpinBox()
        self.hole_count_spin.setRange(1, 99)
        self.hole_count_spin.setToolTip("Expected vent-hole count (drop targets).")
        self.bung_dia_spin = QDoubleSpinBox()
        self.bung_dia_spin.setRange(0.0, 200.0)
        self.bung_dia_spin.setDecimals(1)
        self.bung_dia_spin.setSingleStep(0.5)
        self.bung_dia_spin.setSuffix(" mm")
        self.bung_dia_spin.setToolTip(
            "Nominal cover/bung diameter. 0 = no size gate. Needs a calibration to "
            "measure real size."
        )
        self.tol_spin = QDoubleSpinBox()
        self.tol_spin.setRange(1.0, 100.0)
        self.tol_spin.setDecimals(0)
        self.tol_spin.setSuffix(" %")
        self.tol_spin.setToolTip(
            "Size tolerance: accept a cover whose real diameter is within ± this % "
            "of the nominal. Tighter rejects wrong parts; too tight rejects good ones."
        )
        form.addRow("Vent holes", self.hole_count_spin)
        form.addRow("Bung diameter", self.bung_dia_spin)
        form.addRow("Size tolerance", self.tol_spin)
        self.save_params_btn = QPushButton("Save recipe settings")
        self.save_params_btn.clicked.connect(self._on_save_recipe_params)
        form.addRow(self.save_params_btn)
        v.addWidget(params)

        # Correspondence table
        pts_box = QGroupBox("Correspondences (>= 4 points)")
        pv = QVBoxLayout(pts_box)
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(
            ["Pixel X", "Pixel Y", "Robot X (mm)", "Robot Y (mm)"]
        )
        self.table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.table.verticalHeader().setDefaultSectionSize(26)
        self.table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self.table.itemChanged.connect(self._on_item_changed)
        pv.addWidget(self.table)
        brow = QHBoxLayout()
        remove_btn = QPushButton("Remove selected")
        remove_btn.clicked.connect(self._on_remove)
        clear_btn = QPushButton("Clear all")
        clear_btn.clicked.connect(self._on_clear)
        brow.addWidget(remove_btn)
        brow.addWidget(clear_btn)
        pv.addLayout(brow)
        v.addWidget(pts_box, 1)

        # Fit + save
        fit_box = QGroupBox("Fit / save")
        fv = QVBoxLayout(fit_box)
        self.fit_btn = QPushButton("Fit homography")
        self.fit_btn.clicked.connect(self._on_fit)
        self.residual_label = QLabel("Not fitted.")
        self.residual_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        self.save_btn = QPushButton("Save calibration")
        self.save_btn.setProperty("accent", "primary")
        self.save_btn.clicked.connect(self._on_save)
        fv.addWidget(self.fit_btn)
        fv.addWidget(self.residual_label)
        fv.addWidget(self.save_btn)
        v.addWidget(fit_box)

        self._update_buttons()
        return panel

    # --- camera -------------------------------------------------------------
    def set_camera(self, camera: Camera) -> None:
        self.camera = camera
        self._capture()

    def _on_capture(self) -> None:
        self._capture()

    def _capture(self) -> None:
        # Re-capturing invalidates the picked pixels (they refer to the old frame).
        if self._points:
            ok = QMessageBox.question(
                self,
                "Re-capture frame",
                "Re-capturing clears the points you've already picked "
                "(they refer to the current frame). Continue?",
            )
            if ok != QMessageBox.StandardButton.Yes:
                return
            self._on_clear()
        try:
            self._frame = self.camera.grab()
        except CameraError as exc:
            self._set_status(f"Capture failed: {exc}", theme.DANGER)
            return
        # Build the display pixmap ONCE per capture; clicks only repaint overlays.
        self._base_pix = ndarray_to_qpixmap(self._frame)
        self.view.set_pixmap(self._base_pix)
        self._render()
        self._update_buttons()
        self._set_status(
            "Frame captured. Zoom in and click a known point to add a correspondence.",
            theme.TEXT_DIM,
        )

    # --- point collection ---------------------------------------------------
    def _on_pixel_clicked(self, px: float, py: float) -> None:
        if self._frame is None:
            return
        self._points.append((px, py))
        row = self.table.rowCount()
        self.table.blockSignals(True)
        self.table.insertRow(row)
        for col, val in ((0, px), (1, py)):
            item = QTableWidgetItem(f"{val:.1f}")
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            item.setForeground(Qt.GlobalColor.gray)
            self.table.setItem(row, col, item)
        for col in (2, 3):
            self.table.setItem(row, col, QTableWidgetItem(""))
        self.table.blockSignals(False)
        self._render()
        self._invalidate_fit()
        # Jump straight to entering the robot X for the new point.
        self.table.setCurrentCell(row, 2)
        self.table.editItem(self.table.item(row, 2))

    def _on_item_changed(self, _item) -> None:
        self._invalidate_fit()

    def _on_remove(self) -> None:
        rows = sorted({i.row() for i in self.table.selectedIndexes()}, reverse=True)
        if not rows:
            return
        self.table.blockSignals(True)
        for r in rows:
            self.table.removeRow(r)
            del self._points[r]
        self.table.blockSignals(False)
        self._render()
        self._invalidate_fit()

    def _on_clear(self) -> None:
        self.table.blockSignals(True)
        self.table.setRowCount(0)
        self.table.blockSignals(False)
        self._points.clear()
        self._render()
        self._invalidate_fit()

    # --- fit / save ---------------------------------------------------------
    def _collect(self) -> Tuple[List[List[float]], List[List[float]]]:
        """Return (pixel_pts, robot_pts) for rows with complete, valid robot XY."""
        pix: List[List[float]] = []
        rob: List[List[float]] = []
        for r in range(self.table.rowCount()):
            try:
                px = float(self.table.item(r, 0).text())
                py = float(self.table.item(r, 1).text())
                rx = float(self.table.item(r, 2).text())
                ry = float(self.table.item(r, 3).text())
            except (ValueError, AttributeError):
                continue  # blank / half-filled row — skipped, not an error
            pix.append([px, py])
            rob.append([rx, ry])
        return pix, rob

    def _on_fit(self) -> None:
        pix, rob = self._collect()
        if len(pix) < 4:
            self._invalidate_fit()
            self._set_status(
                f"Need >= 4 complete points to fit ({len(pix)} entered).", theme.WARN
            )
            return
        try:
            transform = HomographyTransform.from_correspondences(
                pix, rob, intrinsics=self.manager.intrinsics, name="calibration"
            )
        except CalibrationError as exc:
            self._invalidate_fit()
            self._set_status(f"Fit failed: {exc}", theme.DANGER)
            return
        self._fitted = transform
        res = transform.residual_mm
        quality = theme.SUCCESS if res <= 1.0 else theme.WARN
        self.residual_label.setText(
            f"Fitted {len(pix)} points · RMS residual {res:.3f} mm"
        )
        self.residual_label.setStyleSheet(f"color:{quality}; font-weight:600;")
        self._set_status(
            "Homography fitted. Review the residual, then save.", theme.SUCCESS
        )
        self._update_buttons()

    def _on_save(self) -> None:
        if self._fitted is None:
            return
        key = self._selected_recipe_key()
        if not key:
            self._set_status("Select a recipe before saving.", theme.WARN)
            return
        if self.manager.has(key):
            ok = QMessageBox.question(
                self,
                "Replace calibration",
                f"This replaces the saved calibration for '{key}'. Continue?",
            )
            if ok != QMessageBox.StandardButton.Yes:
                return
        path = self.manager.save(key, self._fitted)
        self._saved_key = key
        self.calibrationSaved.emit(key, self._fitted)
        self._update_recipe_status()
        self._set_status(f"Saved '{key}' calibration -> {path}", theme.SUCCESS)
        self._update_buttons()

    def _invalidate_fit(self) -> None:
        self._fitted = None
        self.residual_label.setText("Not fitted.")
        self.residual_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        self._update_buttons()

    # --- recipes ------------------------------------------------------------
    def _reload_recipes(self) -> None:
        self.recipe_combo.blockSignals(True)
        self.recipe_combo.clear()
        for r in self.recipes.list():
            self.recipe_combo.addItem(r.name, r.key)
        self.recipe_combo.blockSignals(False)
        self._update_recipe_status()
        self._populate_recipe_params()

    def _selected_recipe_key(self) -> Optional[str]:
        key = self.recipe_combo.currentData()
        return str(key) if key is not None else None

    def _on_recipe_changed(self) -> None:
        self._update_recipe_status()
        self._populate_recipe_params()
        self._update_buttons()

    def _populate_recipe_params(self) -> None:
        """Load the selected recipe's process values into the editor spinboxes."""
        key = self._selected_recipe_key()
        if not key or not self.recipes.has(key):
            return
        r = self.recipes.get(key)
        for spin, val in (
            (self.hole_count_spin, r.hole_count),
            (self.bung_dia_spin, r.cover_diameter_mm),
            (self.tol_spin, round(r.diameter_tolerance * 100.0)),
        ):
            spin.blockSignals(True)
            spin.setValue(val)
            spin.blockSignals(False)

    def _on_save_recipe_params(self) -> None:
        """Persist the editor values back onto the selected recipe (upsert)."""
        key = self._selected_recipe_key()
        if not key or not self.recipes.has(key):
            self._set_status("Select a recipe first.", theme.WARN)
            return
        name = self.recipes.get(key).name
        try:
            self.recipes.add(
                Recipe(
                    key=key,
                    name=name,
                    hole_count=int(self.hole_count_spin.value()),
                    cover_diameter_mm=float(self.bung_dia_spin.value()),
                    diameter_tolerance=max(0.01, self.tol_spin.value() / 100.0),
                )
            )
        except RecipeError as exc:
            self._set_status(f"Cannot save recipe: {exc}", theme.DANGER)
            return
        self.recipesChanged.emit()  # re-applies detectors for the active recipe
        self._set_status(f"Saved settings for '{key}'.", theme.SUCCESS)

    def _update_recipe_status(self) -> None:
        key = self._selected_recipe_key()
        if key and self.manager.has(key):
            self.recipe_status.setText(f"'{key}' — calibration on file (re-fit to replace).")
        elif key:
            self.recipe_status.setText(f"'{key}' — not calibrated yet.")
        else:
            self.recipe_status.setText("")

    def _on_add_recipe(self) -> None:
        name = self.new_recipe_edit.text().strip()
        key = slugify_key(name)
        if not key:
            self._set_status("Enter a recipe name to add.", theme.WARN)
            return
        try:
            self.recipes.add(
                Recipe(
                    key=key,
                    name=name,
                    hole_count=int(self.hole_count_spin.value()),
                    cover_diameter_mm=float(self.bung_dia_spin.value()),
                    diameter_tolerance=max(0.01, self.tol_spin.value() / 100.0),
                )
            )
        except RecipeError as exc:
            self._set_status(f"Cannot add recipe: {exc}", theme.DANGER)
            return
        self.new_recipe_edit.clear()
        self._reload_recipes()
        self.recipe_combo.setCurrentIndex(self.recipe_combo.findData(key))
        self.recipesChanged.emit()
        self._set_status(f"Added recipe '{key}'. Now calibrate it.", theme.SUCCESS)

    # --- helpers ------------------------------------------------------------
    def _update_buttons(self) -> None:
        pix, _ = self._collect() if self.table.rowCount() else ([], [])
        self.fit_btn.setEnabled(len(pix) >= 4)
        self.save_btn.setEnabled(
            self._fitted is not None and bool(self._selected_recipe_key())
        )
        self._render_coach(len(pix))

    # --- guided coach -------------------------------------------------------
    def _coach_step(self, complete: int) -> int:
        """Derive the current step purely from live state (never a stored counter)."""
        key = self._selected_recipe_key()
        if not key:
            return 1                        # select recipe
        if self._frame is None:
            return 2                        # capture
        if complete < 4:
            return 3                        # mark points
        if self._fitted is None:
            return 4                        # fit
        if self._saved_key != key:
            return 5                        # review residual
        return 6                            # saved / done

    def _render_coach(self, complete: int) -> None:
        step = self._coach_step(complete)
        key = self._selected_recipe_key() or "(none)"
        tip = ""
        color = theme.INFO
        if step == 1:
            main = ("Choose the battery type you're calibrating from the Recipe list "
                    "on the right. Each recipe stores its own calibration. Don't see "
                    "it? Type a name and press Add.")
        elif step == 2:
            main = ("Place the calibration target flat in the robot's work plane so "
                    "its known points are all visible, then press Capture frame.")
            tip = ("The target must sit at the same height as the picked parts — a few "
                   "mm off shifts every point (parallax).")
        elif step == 3:
            clicked = len(self._points)
            trailing_blank = (
                clicked > 0
                and self.table.rowCount() >= clicked
                and not (self._row_has_robot_xy(clicked - 1))
            )
            if trailing_blank:
                main = (f"Now type that point's real robot X and Y (mm) in the "
                        f"highlighted row, then press Enter.  Points: {complete}/4.")
            else:
                main = (f"Click a known point on the target in the image — zoom in "
                        f"first for accuracy.  Points: {complete}/4.")
            tip = ("Best points: sharp, high-contrast marks spread to the corners and "
                   "centre of the work area — not clustered.")
            color = theme.WARN if complete < 4 else theme.INFO
        elif step == 4:
            main = (f"Press Fit homography to solve the pixel→robot map from your "
                    f"{complete} points. Add more first (6–9, well spread) for a "
                    f"tighter result.")
        elif step == 5:
            res = self._fitted.residual_mm if self._fitted else float("nan")
            if res <= 1.0:
                main = (f"Good fit: RMS residual {res:.3f} mm (≤ 1.0 mm target). "
                        f"You're clear to Save.")
                color = theme.SUCCESS
            elif res <= 3.0:
                main = (f"Fit is {res:.3f} mm — usable but loose. Re-check a suspect "
                        f"row or add more spread-out points and Fit again, or Save if "
                        f"this is acceptable.")
                color = theme.WARN
            else:
                main = (f"High residual: {res:.3f} mm. A point is probably mislabeled "
                        f"or misclicked — fix its robot X/Y or remove it, then Fit "
                        f"again before saving.")
                color = theme.DANGER
        else:  # 6
            res = self._fitted.residual_mm if self._fitted else float("nan")
            main = (f"Saved for '{key}' (RMS {res:.3f} mm). It's live in the Vision "
                    f"tab. Edit any point to re-fit, or pick another recipe to "
                    f"calibrate next.")
            color = theme.SUCCESS
        self.coach.set_state(step, main, tip, color)

    def _row_has_robot_xy(self, row: int) -> bool:
        try:
            float(self.table.item(row, 2).text())
            float(self.table.item(row, 3).text())
            return True
        except (ValueError, AttributeError):
            return False

    def _render(self) -> None:
        # Reticles are overlays (painted by the view), never baked into the frame —
        # so they stay crisp at any zoom and are visible on a mono image. The base
        # pixmap is already set in _capture; here we only push the point list.
        if self._base_pix is None:
            return
        self.view.set_points(list(self._points))
        self.view.set_active(self.table.currentRow())

    def _set_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color:{color};")
