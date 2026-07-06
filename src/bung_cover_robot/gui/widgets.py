"""Reusable GUI widgets."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QLabel, QSizePolicy

from . import theme


class ImageView(QLabel):
    """A QLabel that shows a pixmap scaled to fit while keeping aspect ratio."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._pix: Optional[QPixmap] = None
        self.setMinimumSize(360, 260)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet(
            "background:#12151b; border:1px solid #363d4a; border-radius:8px; color:#5b6472;"
        )
        self.setText("no image")

    def set_pixmap(self, pix: QPixmap) -> None:
        self._pix = pix
        self._render()

    def resizeEvent(self, event) -> None:  # noqa: N802 (Qt override)
        self._render()
        super().resizeEvent(event)

    def _render(self) -> None:
        if self._pix is None or self._pix.isNull():
            return
        self.setPixmap(
            self._pix.scaled(
                self.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )

    def widget_to_source(self, ex: float, ey: float) -> Optional[tuple]:
        """Map a widget-space click to a pixel in the ORIGINAL (unscaled) image.

        Returns None if there is no image or the click lands outside it. The
        pixmap is drawn scaled with KeepAspectRatio and centered, so we undo
        that letterbox offset and scale to recover source coordinates.
        """
        if self._pix is None or self._pix.isNull():
            return None
        pw, ph = self._pix.width(), self._pix.height()
        aw, ah = self.width(), self.height()
        if pw <= 0 or ph <= 0:
            return None
        scale = min(aw / pw, ah / ph)
        if scale <= 0:
            return None
        ox = (aw - pw * scale) / 2.0
        oy = (ah - ph * scale) / 2.0
        sx = (ex - ox) / scale
        sy = (ey - oy) / scale
        if 0.0 <= sx < pw and 0.0 <= sy < ph:
            return (sx, sy)
        return None


class ClickableImageView(ImageView):
    """An ImageView that emits ``pixelClicked`` in original-image coordinates."""

    pixelClicked = Signal(float, float)

    def mousePressEvent(self, event) -> None:  # noqa: N802 (Qt override)
        pos = event.position()
        pt = self.widget_to_source(pos.x(), pos.y())
        if pt is not None:
            self.pixelClicked.emit(pt[0], pt[1])
        super().mousePressEvent(event)


class StatusPill(QLabel):
    """A small rounded status chip; call set_state to recolor."""

    def __init__(self, text: str = "", parent=None) -> None:
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_state(text, "idle")

    def set_state(self, text: str, kind: str) -> None:
        self.setText(text)
        self.setStyleSheet(theme.pill_style(kind))
