"""Calibration tab — build a real pixel->robot calibration interactively.

Workflow (Claude.md §13):
  1. Place a target with known robot-frame coordinates in view and capture a frame.
  2. Click each known point in the image; type its robot X/Y (mm) in the table.
  3. Fit the homography (>= 4 points) — the RMS residual reports the quality.
  4. Save it as the cover-plane transform or a per-recipe battery-top transform.

The math/persistence lives in vision/calibration; this tab is a thin operator view
over it. Saved cover calibrations are broadcast so the Vision tab picks them up live.
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

from ..vision.calibration import (
    CalibrationError,
    CalibrationManager,
    HomographyTransform,
)
from ..vision.camera import Camera, CameraError
from . import theme
from .imaging import annotate_points, ndarray_to_qpixmap
from .widgets import ClickableImageView

_COVER = "Cover plane (pick surface)"
_BATTERY = "Battery top (per recipe)"


class CalibrationTab(QWidget):
    """Collect pixel<->robot correspondences and save a calibration."""

    coverCalibrationSaved = Signal(object)  # emits the saved HomographyTransform

    def __init__(
        self,
        camera: Camera,
        manager: Optional[CalibrationManager] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.camera = camera
        self.manager = manager or CalibrationManager()
        self._frame = None
        self._points: List[Tuple[float, float]] = []
        self._fitted: Optional[HomographyTransform] = None

        root = QHBoxLayout(self)
        root.addLayout(self._build_view_column(), 1)
        root.addWidget(self._build_side_panel())

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

        # Target selection
        tgt = QGroupBox("Calibration target")
        tg = QVBoxLayout(tgt)
        self.plane_combo = QComboBox()
        self.plane_combo.addItems([_COVER, _BATTERY])
        self.plane_combo.currentIndexChanged.connect(self._on_plane_changed)
        tg.addWidget(self.plane_combo)
        rrow = QHBoxLayout()
        rrow.addWidget(QLabel("Recipe key:"))
        self.recipe_edit = QLineEdit()
        self.recipe_edit.setPlaceholderText("e.g. g31")
        self.recipe_edit.setEnabled(False)
        self.recipe_edit.textChanged.connect(self._update_buttons)
        rrow.addWidget(self.recipe_edit)
        tg.addLayout(rrow)
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
        if self._is_battery():
            key = self.recipe_edit.text().strip()
            if not key:
                self._set_status("Enter a recipe key before saving.", theme.WARN)
                return
            path = self.manager.save_battery_transform(key, self._fitted)
            self._set_status(f"Saved battery '{key}' calibration -> {path}", theme.SUCCESS)
        else:
            path = self.manager.save_cover_transform(self._fitted)
            self.coverCalibrationSaved.emit(self._fitted)
            self._set_status(f"Saved cover calibration -> {path}", theme.SUCCESS)

    def _invalidate_fit(self) -> None:
        self._fitted = None
        self.residual_label.setText("Not fitted.")
        self.residual_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        self._update_buttons()

    # --- helpers ------------------------------------------------------------
    def _is_battery(self) -> bool:
        return self.plane_combo.currentText() == _BATTERY

    def _on_plane_changed(self) -> None:
        self.recipe_edit.setEnabled(self._is_battery())
        self._update_buttons()

    def _update_buttons(self) -> None:
        pix, _ = self._collect() if self.table.rowCount() else ([], [])
        self.fit_btn.setEnabled(len(pix) >= 4)
        can_save = self._fitted is not None and (
            not self._is_battery() or bool(self.recipe_edit.text().strip())
        )
        self.save_btn.setEnabled(can_save)

    def _render(self) -> None:
        if self._frame is None:
            return
        overlay = annotate_points(self._frame, self._points)
        self.view.set_pixmap(ndarray_to_qpixmap(overlay))

    def _set_status(self, text: str, color: str) -> None:
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color:{color};")
