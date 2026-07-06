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
    QComboBox,
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

from ..app.recipes import Recipe, RecipeError, RecipeStore, slugify_key
from ..vision.calibration import (
    CalibrationError,
    CalibrationManager,
    HomographyTransform,
)
from ..vision.camera import Camera, CameraError
from . import theme
from .imaging import annotate_points, ndarray_to_qpixmap
from .widgets import ClickableImageView


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
        self._points: List[Tuple[float, float]] = []
        self._fitted: Optional[HomographyTransform] = None

        root = QHBoxLayout(self)
        root.addLayout(self._build_view_column(), 1)
        root.addWidget(self._build_side_panel())

        self._reload_recipes()
        self._capture()

    # --- layout -------------------------------------------------------------
    def _build_view_column(self) -> QVBoxLayout:
        col = QVBoxLayout()
        header = QLabel("Calibration target — click a known point, then enter its robot X/Y")
        header.setStyleSheet("font-size:15px; font-weight:600;")
        self.view = ClickableImageView()
        self.view.pixelClicked.connect(self._on_pixel_clicked)
        col.addWidget(header)
        col.addWidget(self.view, 1)
        self.status_label = QLabel("Capture a frame to begin.")
        self.status_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        col.addWidget(self.status_label)
        return col

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
        try:
            self._frame = self.camera.grab()
        except CameraError as exc:
            self._set_status(f"Capture failed: {exc}", theme.DANGER)
            return
        self._render()
        self._set_status(
            "Frame captured. Click a known point to add a correspondence.",
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
        path = self.manager.save(key, self._fitted)
        self.calibrationSaved.emit(key, self._fitted)
        self._update_recipe_status()
        self._set_status(f"Saved '{key}' calibration -> {path}", theme.SUCCESS)

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

    def _selected_recipe_key(self) -> Optional[str]:
        key = self.recipe_combo.currentData()
        return str(key) if key is not None else None

    def _on_recipe_changed(self) -> None:
        self._update_recipe_status()
        self._update_buttons()

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
            self.recipes.add(Recipe(key=key, name=name))
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

    def _render(self) -> None:
        if self._frame is None:
            return
        overlay = annotate_points(self._frame, self._points)
        self.view.set_pixmap(ndarray_to_qpixmap(overlay))

    def _set_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color:{color};")
