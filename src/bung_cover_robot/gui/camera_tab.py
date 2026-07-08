"""Camera (Basler) tab — connection + imaging controls with a LIVE preview.

A background grabber thread streams frames continuously (so the preview is a live
feed, not a one-shot snapshot) and the controls write through to the shared camera
(real GenICam nodes on a Basler). Exposure/gain each have an Auto toggle:
Auto = the camera's own ExposureAuto/GainAuto Continuous loop (good for getting a
usable image fast); turning Auto off locks in the last converged value so
detection sees a stable frame. Brightness/contrast/gamma are also applied locally
to the preview so the sliders have a visible effect against the mock camera.
"""

from __future__ import annotations

import time
from typing import Dict, Optional

from PySide6.QtCore import Qt, Signal, QThread
from PySide6.QtWidgets import (
    QCheckBox,
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
from .imaging import (
    adjust_preview,
    demo_frame,
    downscale_for_preview,
    ndarray_to_qpixmap,
)
from .widgets import ImageView

# (label, control-name, slider min, max, scale, default) — value = slider/scale
_CONTROLS = [
    ("Exposure (µs)", "exposure_time_us", 100, 30000, 1, 8000),
    ("Gain (dB)", "gain", 0, 240, 10, 0),
    ("Brightness", "brightness", -100, 100, 100, 0),
    ("Contrast", "contrast", 0, 200, 100, 100),
    ("Gamma", "gamma", 30, 300, 100, 100),
]

# Controls that map to a real camera node with an Auto sibling, and whether Auto
# starts on. Auto-exposure on gives a usable image immediately; auto-GAIN starts
# OFF (gain=0) because auto-gain amplifies sensor noise -> a grainy image. Raise
# gain by hand only if exposure alone can't brighten the scene.
_AUTO = {"exposure_time_us": "exposure_auto", "gain": "gain_auto"}
_AUTO_DEFAULT = {"exposure_time_us": True, "gain": False}


class _LiveGrabber(QThread):
    """Continuously grabs frames off the GUI thread and emits them. A short
    per-grab timeout keeps it responsive; grab errors are surfaced (not fatal)."""

    # (preview_frame downscaled off the GUI thread, (h, w) of the full sensor frame)
    frameReady = Signal(object, object)
    grabError = Signal(str)

    def __init__(self, camera: Camera, fps: float = 60.0, parent=None) -> None:
        super().__init__(parent)
        self._camera = camera
        self._interval = 1.0 / max(1.0, fps)
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def run(self) -> None:
        while not self._stop:
            t0 = time.monotonic()
            try:
                frame = self._camera.grab(timeout_ms=1000)
            except Exception as exc:  # noqa: BLE001 - never let the thread die silently
                if self._stop:
                    break
                self.grabError.emit(str(exc))
                time.sleep(0.3)
                continue
            if self._stop:
                break
            h, w = frame.shape[:2]
            # Downscale HERE, off the GUI thread: a multi-MP frame is far bigger
            # than the view, and shrinking it makes the QPixmap/scale path cheap.
            self.frameReady.emit(downscale_for_preview(frame), (h, w))
            dt = time.monotonic() - t0
            if (rest := self._interval - dt) > 0:
                time.sleep(rest)


class CameraTab(QWidget):
    cameraChanged = Signal()

    def __init__(self, camera: Camera, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.camera = camera
        self._frame = None
        self._sliders: Dict[str, QSlider] = {}
        self._value_labels: Dict[str, QLabel] = {}
        self._scales: Dict[str, float] = {}
        self._auto_checks: Dict[str, QCheckBox] = {}
        self._grabber: Optional[_LiveGrabber] = None
        self._visible = False
        self._frame_times: list = []   # rolling timestamps for a live FPS readout

        root = QHBoxLayout(self)
        root.addWidget(self._build_controls_column())
        right = QVBoxLayout()
        right.addWidget(self._build_preview_group(), 1)
        root.addLayout(right, 1)

        self._apply_initial_controls()
        self._grab()

    # --- layout -------------------------------------------------------------
    def _build_controls_column(self) -> QWidget:
        col = QWidget()
        col.setFixedWidth(360)
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
        self.live_check = QCheckBox("Live view")
        self.live_check.setChecked(True)
        self.live_check.toggled.connect(self._on_live_toggled)
        cg.addWidget(self.live_check)
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
            # Auto toggle for exposure/gain, on the far right.
            if name in _AUTO:
                auto = QCheckBox("Auto")
                auto.setChecked(_AUTO_DEFAULT[name])
                auto.toggled.connect(lambda c, n=name: self._on_auto_toggled(n, c))
                self._auto_checks[name] = auto
                grid.addWidget(auto, r, 3)
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
        # If this control is under Auto, don't fight the camera's auto loop.
        if name in _AUTO and self._auto_checks[name].isChecked():
            self._render_preview()
            return
        try:
            self.camera.set_control(name, value)
        except CameraControlError:
            pass  # mock stores it; a real camera may not expose this node
        self._render_preview()

    def _on_auto_toggled(self, name: str, checked: bool) -> None:
        auto_ctrl = _AUTO[name]
        slider = self._sliders[name]
        if checked:
            slider.setEnabled(False)
            try:
                self.camera.set_control(auto_ctrl, "Continuous")
            except CameraControlError:
                pass
        else:
            # Lock in whatever the auto loop converged to, then go manual.
            try:
                converged = float(self.camera.get_control(name))
                slider.blockSignals(True)
                slider.setValue(int(round(converged * self._scales[name])))
                slider.blockSignals(False)
            except (CameraControlError, CameraError, ValueError, TypeError):
                pass
            try:
                self.camera.set_control(auto_ctrl, "Off")
            except CameraControlError:
                pass
            slider.setEnabled(True)
            self._on_control(name)  # push the (now manual) value

    def _apply_initial_controls(self) -> None:
        """Push the current UI state to a freshly connected camera: auto modes
        first, then manual values / preview-only controls. Skips nodes the model
        doesn't expose."""
        for name in _AUTO:
            self._on_auto_toggled(name, self._auto_checks[name].isChecked())
        for _, name, *_ in _CONTROLS:
            if name in _AUTO and self._auto_checks[name].isChecked():
                continue
            self._on_control(name)

    def _grab(self) -> None:
        """Single manual grab (also the initial frame). Live view keeps the
        preview updating on its own."""
        try:
            raw = self.camera.grab()
        except CameraError as exc:
            self.info_label.setText(f"grab failed: {exc}")
            return
        h, w = raw.shape[:2]
        self._frame = downscale_for_preview(raw)
        self._frame_times.clear()
        self.info_label.setText(f"{w}×{h} · still")
        self._render_preview()

    def _on_frame(self, frame, orig) -> None:
        self._frame = frame
        h, w = orig
        self.info_label.setText(f"{w}×{h} · {self._live_fps():.0f} fps · live")
        self._render_preview()

    def _live_fps(self) -> float:
        now = time.monotonic()
        self._frame_times.append(now)
        if len(self._frame_times) > 30:
            self._frame_times.pop(0)
        span = self._frame_times[-1] - self._frame_times[0]
        n = len(self._frame_times)
        return (n - 1) / span if n > 1 and span > 0 else 0.0

    def _on_grab_error(self, msg: str) -> None:
        self.info_label.setText(f"grab failed: {msg}")

    def _render_preview(self) -> None:
        if self._frame is None:
            return
        # A real Basler applies brightness/contrast/gamma on-sensor (the sliders
        # write those nodes), so don't re-do it in software on every live frame;
        # only the mock needs the CPU-side adjust to make its sliders visible.
        if isinstance(self.camera, MockCamera):
            img = adjust_preview(
                self._frame,
                brightness=self._value("brightness"),
                contrast=self._value("contrast"),
                gamma=self._value("gamma"),
            )
        else:
            img = self._frame
        self.view.set_pixmap(ndarray_to_qpixmap(img))

    # --- live streaming -----------------------------------------------------
    def _start_live(self) -> None:
        if self._grabber is not None and self._grabber.isRunning():
            return
        if not self.live_check.isChecked():
            return
        self._grabber = _LiveGrabber(self.camera, parent=self)
        self._grabber.frameReady.connect(self._on_frame)
        self._grabber.grabError.connect(self._on_grab_error)
        self._grabber.start()

    def _stop_live(self) -> None:
        grabber = self._grabber
        self._grabber = None
        if grabber is not None:
            grabber.stop()
            grabber.wait(2000)
            grabber.deleteLater()

    def _on_live_toggled(self, checked: bool) -> None:
        if checked and self._visible:
            self._start_live()
        elif not checked:
            self._stop_live()

    def showEvent(self, event) -> None:  # noqa: N802 - Qt override
        super().showEvent(event)
        self._visible = True
        self._start_live()

    def hideEvent(self, event) -> None:  # noqa: N802 - Qt override
        super().hideEvent(event)
        self._visible = False
        self._stop_live()

    def closeEvent(self, event) -> None:  # noqa: N802 - Qt override
        self._stop_live()
        super().closeEvent(event)

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
        self._stop_live()
        old = self.camera
        self.camera = cam
        if old is not cam:
            try:
                old.close()
            except Exception:
                pass
        self._adopt_camera_controls()
        self._refresh_conn_status()
        self._apply_initial_controls()
        self._grab()
        if self._visible:
            self._start_live()
        self.cameraChanged.emit()

    def _adopt_camera_controls(self) -> None:
        """Match each slider's RANGE and starting VALUE to the connected camera's
        actual node. The hardcoded defaults suit the software preview model (e.g.
        contrast is a multiplier centered at 1.0), but a real Basler's BslContrast
        is centered at 0 over roughly [-1, 1] — so a default of 1.0 is max contrast,
        and 0 has no room below it. Adopting the camera's own range + current value
        gives every slider the right zero and headroom. Best-effort per control;
        the mock exposes no ranges, so it keeps the designed software sliders."""
        control_range = getattr(self.camera, "control_range", None)
        get_control = getattr(self.camera, "get_control", None)
        if control_range is None:
            return
        for _, name, *_rest in _CONTROLS:
            try:
                lo, hi = control_range(name)
            except (CameraControlError, CameraError):
                continue  # node absent on this model -> keep the designed slider
            cur = None
            if get_control is not None:
                try:
                    cur = float(get_control(name))
                except (CameraControlError, CameraError, ValueError, TypeError):
                    cur = None
            if cur is None:
                cur = 0.0 if lo <= 0.0 <= hi else (lo + hi) / 2.0  # neutral fallback
            scale = self._scales[name]
            slider = self._sliders[name]
            slider.blockSignals(True)
            slider.setRange(int(round(lo * scale)), int(round(hi * scale)))
            slider.setValue(int(round(cur * scale)))
            slider.blockSignals(False)
            self._value_labels[name].setText(
                f"{self._value(name):.0f}" if name == "exposure_time_us"
                else f"{self._value(name):.2f}"
            )

    def _refresh_conn_status(self) -> None:
        if isinstance(self.camera, MockCamera):
            self.conn_status.setText("Mock camera (dry-run)")
            self.conn_status.setStyleSheet(f"color:{theme.TEXT_DIM}; font-weight:600;")
        else:
            self.conn_status.setText("Basler connected")
            self.conn_status.setStyleSheet(f"color:{theme.SUCCESS}; font-weight:600;")
