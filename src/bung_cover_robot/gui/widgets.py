"""Reusable GUI widgets."""

from __future__ import annotations

from typing import Optional, Tuple

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QPixmap
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


class RoiImageView(ImageView):
    """An ImageView you can draw a rectangular pick region on by dragging.

    Emits ``roiChanged(x, y, w, h)`` in ORIGINAL-image pixels when a region is
    drawn, and ``roiCleared`` when it's removed. The committed region is painted
    over the frame, so it survives frame/overlay updates.
    """

    roiChanged = Signal(float, float, float, float)
    roiCleared = Signal()

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._roi: Optional[Tuple[float, float, float, float]] = None  # source px
        self._draw_enabled = False
        self._drag_start: Optional[Tuple[float, float]] = None          # source px
        self._drag_cur: Optional[Tuple[float, float]] = None            # source px

    # --- public API ---------------------------------------------------------
    def set_draw_enabled(self, on: bool) -> None:
        self._draw_enabled = on
        self._drag_start = self._drag_cur = None
        self.setCursor(Qt.CursorShape.CrossCursor if on else Qt.CursorShape.ArrowCursor)
        self.update()

    def is_drawing(self) -> bool:
        return self._draw_enabled

    def set_roi(self, roi: Optional[Tuple[float, float, float, float]]) -> None:
        """Show a committed region (source px), or None to hide it. Silent."""
        self._roi = tuple(float(v) for v in roi) if roi else None  # type: ignore[assignment]
        self.update()

    def roi(self) -> Optional[Tuple[float, float, float, float]]:
        return self._roi

    def clear_roi(self) -> None:
        self._roi = None
        self.update()
        self.roiCleared.emit()

    # --- coordinate mapping (inverse of widget_to_source) -------------------
    def _layout(self):
        if self._pix is None or self._pix.isNull():
            return None
        pw, ph = self._pix.width(), self._pix.height()
        aw, ah = self.width(), self.height()
        if pw <= 0 or ph <= 0:
            return None
        scale = min(aw / pw, ah / ph)
        if scale <= 0:
            return None
        return scale, (aw - pw * scale) / 2.0, (ah - ph * scale) / 2.0, pw, ph

    def source_to_widget(self, sx: float, sy: float) -> Optional[Tuple[float, float]]:
        lay = self._layout()
        if lay is None:
            return None
        scale, ox, oy, _pw, _ph = lay
        return (ox + sx * scale, oy + sy * scale)

    def _clamp_source(self, ex: float, ey: float) -> Optional[Tuple[float, float]]:
        """Widget point -> source px, clamped to the image so a drag can reach
        the border even when the cursor leaves the image."""
        lay = self._layout()
        if lay is None:
            return None
        scale, ox, oy, pw, ph = lay
        sx = min(max((ex - ox) / scale, 0.0), pw - 1.0)
        sy = min(max((ey - oy) / scale, 0.0), ph - 1.0)
        return (sx, sy)

    # --- mouse --------------------------------------------------------------
    def mousePressEvent(self, event) -> None:  # noqa: N802 (Qt override)
        if self._draw_enabled and event.button() == Qt.MouseButton.LeftButton:
            pos = event.position()
            pt = self.widget_to_source(pos.x(), pos.y())
            if pt is not None:
                self._drag_start = pt
                self._drag_cur = pt
                self.update()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802 (Qt override)
        if self._draw_enabled and self._drag_start is not None:
            pos = event.position()
            self._drag_cur = self._clamp_source(pos.x(), pos.y()) or self._drag_cur
            self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802 (Qt override)
        if self._draw_enabled and self._drag_start is not None:
            x0, y0 = self._drag_start
            x1, y1 = self._drag_cur or self._drag_start
            self._drag_start = self._drag_cur = None
            x, y, w, h = min(x0, x1), min(y0, y1), abs(x1 - x0), abs(y1 - y0)
            if w >= 5 and h >= 5:            # ignore an accidental click/tiny drag
                self._roi = (x, y, w, h)
                self.set_draw_enabled(False)
                self.roiChanged.emit(x, y, w, h)
            self.update()
        super().mouseReleaseEvent(event)

    # --- painting -----------------------------------------------------------
    def paintEvent(self, event) -> None:  # noqa: N802 (Qt override)
        super().paintEvent(event)
        if self._drag_start is not None and self._drag_cur is not None:
            corners, live = (self._drag_start, self._drag_cur), True
        elif self._roi is not None:
            x, y, w, h = self._roi
            corners, live = ((x, y), (x + w, y + h)), False
        else:
            return
        p0 = self.source_to_widget(*corners[0])
        p1 = self.source_to_widget(*corners[1])
        if p0 is None or p1 is None:
            return
        rect = QRectF(QPointF(*p0), QPointF(*p1)).normalized()
        painter = QPainter(self)
        accent = QColor(90, 190, 255)
        painter.fillRect(rect, QColor(90, 190, 255, 38))
        painter.setPen(QPen(accent, 2, Qt.PenStyle.DashLine if live else Qt.PenStyle.SolidLine))
        painter.drawRect(rect)
        painter.drawText(rect.topLeft() + QPointF(5, 15), "PICK ROI")
        painter.end()


class StatusPill(QLabel):
    """A small rounded status chip; call set_state to recolor."""

    def __init__(self, text: str = "", parent=None) -> None:
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_state(text, "idle")

    def set_state(self, text: str, kind: str) -> None:
        self.setText(text)
        self.setStyleSheet(theme.pill_style(kind))
