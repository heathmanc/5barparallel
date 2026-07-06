"""Camera (Basler) tab — connection + imaging controls with a live preview.

Controls write through to the shared camera (real GenICam nodes on a Basler); the
preview also applies brightness/contrast/gamma locally so the sliders have a
visible effect with the mock camera in dry-run.
"""

from __future__ import annotations

from typing import Dict, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ..vision.camera import (
    BaslerCamera,
    Camera,
    CameraConfig,
    CameraControlError,
    CameraError,
    MockCamera,
)
from . import theme
from .imaging import adjust_preview, demo_frame, ndarray_to_qpixmap
from .widgets import ImageView

# (label, control-name, slider min, max, scale, default) — value = slider/scale
_CONTROLS = [
    ("Exposure (µs)", "exposure_time_us", 100, 30000, 1, 8000),
    ("Gain (dB)", "gain", 0, 240, 10, 20),
    ("Brightness", "brightness", -100, 100, 100, 0),
    ("Contrast", "contrast", 0, 200, 100, 100),
    ("Gamma", "gamma", 30, 300, 100, 100),
]


class CameraTab(QWidget):
    cameraChanged = Signal()

    def __init__(self, camera: Camera, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.camera = camera
        self._frame = None
        self._sliders: Dict[str, QSlider] = {}
        self._value_labels: Dict[str, QLabel] = {}
        self._scales: Dict[str, float] = {}

        root = QHBoxLayout(self)
        root.addWidget(self._build_controls_column())
        right = QVBoxLayout()
        right.addWidget(self._build_preview_group(), 1)
        root.addLayout(right, 1)

        self._grab()

    # --- layout -------------------------------------------------------------
    def _build_controls_column(self) -> QWidget:
        col = QWidget()
        col.setFixedWidth(340)
        v = QVBoxLayout(col)
        v.setContentsMargins(0, 0, 0, 0)

        conn = QGroupBox("Connection")
        cg = QVBoxLayout(conn)
        self.conn_status = QLabel()
        cg.addWidget(self.conn_status)
        row = QHBoxLayout()
        row.addWidget(QLabel("Serial:"))
        self.serial_edit = QLineEdit()
        self.serial_edit.setPlaceholderText("(first found)")
        row.addWidget(self.serial_edit)
        cg.addLayout(row)
        brow = QHBoxLayout()
        connect_btn = QPushButton("Connect Basler")
        connect_btn.clicked.connect(self._on_connect_basler)
        mock_btn = QPushButton("Use mock")
        mock_btn.clicked.connect(self._on_use_mock)
        brow.addWidget(connect_btn)
        brow.addWidget(mock_btn)
        cg.addLayout(brow)
        v.addWidget(conn)

        ctrl = QGroupBox("Imaging controls")
        grid = QGridLayout(ctrl)
        for r, (label, name, lo, hi, scale, default) in enumerate(_CONTROLS):
            self._scales[name] = scale
            name_lbl = QLabel(label)
            name_lbl.setStyleSheet(f"color:{theme.TEXT_DIM};")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(lo, hi)
            slider.setValue(default)
            slider.valueChanged.connect(lambda _v, n=name: self._on_control(n))
            val = QLabel()
            val.setMinimumWidth(56)
            val.setAlignment(Qt.AlignmentFlag.AlignRight)
            self._sliders[name] = slider
            self._value_labels[name] = val
            grid.addWidget(name_lbl, r, 0)
            grid.addWidget(slider, r, 1)
            grid.addWidget(val, r, 2)
        v.addWidget(ctrl)
        v.addStretch(1)
        self._refresh_conn_status()
        return col

    def _build_preview_group(self) -> QGroupBox:
        box = QGroupBox("Preview")
        v = QVBoxLayout(box)
        self.view = ImageView()
        v.addWidget(self.view, 1)
        row = QHBoxLayout()
        grab_btn = QPushButton("Grab frame")
        grab_btn.clicked.connect(self._grab)
        self.info_label = QLabel()
        self.info_label.setStyleSheet(f"color:{theme.TEXT_DIM};")
        row.addWidget(grab_btn)
        row.addStretch(1)
        row.addWidget(self.info_label)
        v.addLayout(row)
        return box

    # --- controls -----------------------------------------------------------
    def _value(self, name: str) -> float:
        return self._sliders[name].value() / self._scales[name]

    def _on_control(self, name: str) -> None:
        value = self._value(name)
        self._value_labels[name].setText(
            f"{value:.0f}" if name == "exposure_time_us" else f"{value:.2f}"
        )
        try:
            self.camera.set_control(name, value)
        except CameraControlError:
            pass  # mock stores it; a real camera may not expose this node
        self._render_preview()

    def _grab(self) -> None:
        try:
            self._frame = self.camera.grab()
        except CameraError as exc:
            self.info_label.setText(f"grab failed: {exc}")
            return
        h, w = self._frame.shape[:2]
        self.info_label.setText(f"{w}×{h}")
        for name in self._sliders:
            self._on_control(name)  # push all control values + render

    def _render_preview(self) -> None:
        if self._frame is None:
            return
        img = adjust_preview(
            self._frame,
            brightness=self._value("brightness"),
            contrast=self._value("contrast"),
            gamma=self._value("gamma"),
        )
        self.view.set_pixmap(ndarray_to_qpixmap(img))

    # --- connection ---------------------------------------------------------
    def _on_connect_basler(self) -> None:
        serial = self.serial_edit.text().strip() or None
        cam = BaslerCamera(CameraConfig(serial_number=serial))
        try:
            cam.open()
        except CameraError as exc:
            self.conn_status.setText(f"Connect failed: {exc}")
            self.conn_status.setStyleSheet(f"color:{theme.DANGER}; font-weight:600;")
            return
        self._swap(cam)

    def _on_use_mock(self) -> None:
        cam = MockCamera(
            CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
        )
        cam.open()
        self._swap(cam)

    def _swap(self, cam: Camera) -> None:
        old = self.camera
        self.camera = cam
        if old is not cam:
            try:
                old.close()
            except Exception:
                pass
        self._refresh_conn_status()
        self._grab()
        self.cameraChanged.emit()

    def _refresh_conn_status(self) -> None:
        if isinstance(self.camera, MockCamera):
            self.conn_status.setText("Mock camera (dry-run)")
            self.conn_status.setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")
        else:
            self.conn_status.setText("Basler connected")
            self.conn_status.setStyleSheet(f"color:{theme.SUCCESS}; font-weight:600;")
