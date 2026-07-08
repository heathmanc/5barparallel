"""Reusable GUI widgets."""

from __future__ import annotations

from typing import Optional, Tuple

import math

from PySide6.QtCore import QPointF, QRectF, Qt, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPixmap
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


class ZoomPanImageView(ImageView):
    """An image view with wheel zoom, drag pan, a magnifier loupe, and
    high-visibility reticles at picked points — for precise calibration picking.

    The captured frame is stored as a QPixmap and the picked points as a list of
    source-pixel (x, y); the frame and all overlays are drawn in ``paintEvent``
    through one shared source<->widget transform, so reticles stay crisp and
    screen-constant-size at every zoom level. Emits ``pixelClicked(sx, sy)`` in
    ORIGINAL-image pixels on a click (not a pan)."""

    pixelClicked = Signal(float, float)
    zoomChanged = Signal(float)          # effective scale as a percent (100 = fit)

    Z_MIN = 1.0
    Z_MAX = 40.0
    WHEEL_STEP = 1.25
    MOVE_THRESH = 4          # widget px; a drag beyond this is a pan, not a click
    LOUPE_R = 74             # loupe radius, widget px
    LOUPE_M = 8.0            # loupe magnification

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setText("")
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self._z = 1.0
        self._ox = 0.0
        self._oy = 0.0
        self._pts: list = []            # source px (sx, sy)
        self._active = -1
        self._cursor: Optional[QPointF] = None
        self._loupe_on = True
        self._press: Optional[QPointF] = None
        self._press_off: Tuple[float, float] = (0.0, 0.0)
        self._moved = False

    # --- public API ---------------------------------------------------------
    def set_pixmap(self, pix: QPixmap) -> None:  # override (no QLabel setPixmap)
        new_size = pix is not None and (
            self._pix is None
            or pix.width() != self._pix.width()
            or pix.height() != self._pix.height()
        )
        self._pix = pix
        if new_size:
            self._z = 1.0          # a new frame size -> back to fit
        self._clamp_pan()
        self.update()
        self._emit_zoom()

    def set_points(self, pts) -> None:
        self._pts = [(float(x), float(y)) for x, y in pts]
        self.update()

    def set_active(self, index: int) -> None:
        self._active = int(index)
        self.update()

    def set_loupe(self, on: bool) -> None:
        self._loupe_on = bool(on)
        self.update()

    def place_source_point(self, sx: float, sy: float) -> None:
        """Test seam: emit a pick in source coords, bypassing screen geometry."""
        self.pixelClicked.emit(float(sx), float(sy))

    def zoom_in(self) -> None:
        self._zoom_to(self.WHEEL_STEP, self.width() / 2.0, self.height() / 2.0)

    def zoom_out(self) -> None:
        self._zoom_to(1.0 / self.WHEEL_STEP, self.width() / 2.0, self.height() / 2.0)

    def fit(self) -> None:
        self._z = 1.0
        self._clamp_pan()
        self.update()
        self._emit_zoom()

    def one_to_one(self) -> None:
        f = self._fit_scale()
        if f is None or f <= 0:
            return
        self._zoom_to((1.0 / f) / self._z, self.width() / 2.0, self.height() / 2.0)

    # --- transform (single source of truth) ---------------------------------
    def _fit_scale(self) -> Optional[float]:
        if self._pix is None or self._pix.isNull():
            return None
        w, h = self._pix.width(), self._pix.height()
        aw, ah = self.width(), self.height()
        if w <= 0 or h <= 0 or aw <= 0 or ah <= 0:
            return None
        return min(aw / w, ah / h)

    def _scale(self) -> Optional[float]:
        f = self._fit_scale()
        return None if f is None else f * self._z

    def source_to_widget(self, sx: float, sy: float) -> Optional[Tuple[float, float]]:
        s = self._scale()
        if s is None:
            return None
        return (self._ox + sx * s, self._oy + sy * s)

    def widget_to_source(self, ex: float, ey: float, clamp: bool = False):
        s = self._scale()
        if s is None:
            return None
        w, h = self._pix.width(), self._pix.height()
        sx, sy = (ex - self._ox) / s, (ey - self._oy) / s
        if clamp:
            return (min(max(sx, 0.0), w - 1.0), min(max(sy, 0.0), h - 1.0))
        return (sx, sy) if (0.0 <= sx < w and 0.0 <= sy < h) else None

    def _clamp_pan(self) -> None:
        s = self._scale()
        if s is None:
            return
        aw, ah = self.width(), self.height()
        dw, dh = self._pix.width() * s, self._pix.height() * s
        self._ox = (aw - dw) / 2.0 if dw <= aw else min(0.0, max(aw - dw, self._ox))
        self._oy = (ah - dh) / 2.0 if dh <= ah else min(0.0, max(ah - dh, self._oy))

    def _zoom_to(self, factor: float, ax: float, ay: float) -> None:
        s_old = self._scale()
        if s_old is None:
            return
        px, py = (ax - self._ox) / s_old, (ay - self._oy) / s_old  # anchor, OLD scale
        self._z = min(max(self._z * factor, self.Z_MIN), self.Z_MAX)
        s_new = self._scale()
        self._ox, self._oy = ax - px * s_new, ay - py * s_new       # keep anchor put
        self._clamp_pan()
        self.update()
        self._emit_zoom()

    def _emit_zoom(self) -> None:
        s = self._scale()
        if s is not None:
            self.zoomChanged.emit(round(s * 100.0))

    # --- events -------------------------------------------------------------
    def wheelEvent(self, event) -> None:  # noqa: N802
        d = event.angleDelta().y()
        if d != 0:
            factor = self.WHEEL_STEP ** (d / 120.0)
            pos = event.position()
            self._zoom_to(factor, pos.x(), pos.y())
        event.accept()  # don't let the wheel scroll the recipe table behind us

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton:
            self._press = event.position()
            self._press_off = (self._ox, self._oy)
            self._moved = False
        elif event.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._press = event.position()
            self._press_off = (self._ox, self._oy)
            self._moved = True  # middle/right = always pan, never a pick
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        pos = event.position()
        self._cursor = pos
        if self._press is not None:
            dx, dy = pos.x() - self._press.x(), pos.y() - self._press.y()
            if self._moved or math.hypot(dx, dy) > self.MOVE_THRESH:
                self._moved = True
                self._ox = self._press_off[0] + dx
                self._oy = self._press_off[1] + dy
                self._clamp_pan()
        self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if event.button() == Qt.MouseButton.LeftButton and self._press is not None:
            was_pan = self._moved
            self._press = None
            if not was_pan:
                pt = self.widget_to_source(event.position().x(), event.position().y())
                if pt is not None:
                    self.pixelClicked.emit(pt[0], pt[1])
        elif event.button() in (Qt.MouseButton.MiddleButton, Qt.MouseButton.RightButton):
            self._press = None
        super().mouseReleaseEvent(event)

    def leaveEvent(self, event) -> None:  # noqa: N802
        self._cursor = None
        self.update()
        super().leaveEvent(event)

    def resizeEvent(self, event) -> None:  # noqa: N802
        self._clamp_pan()
        self.update()  # do NOT call ImageView._render (which uses QLabel.setPixmap)

    def keyPressEvent(self, event) -> None:  # noqa: N802
        k = event.key()
        if k in (Qt.Key.Key_Plus, Qt.Key.Key_Equal):
            self.zoom_in()
        elif k == Qt.Key.Key_Minus:
            self.zoom_out()
        elif k in (Qt.Key.Key_0, Qt.Key.Key_F):
            self.fit()
        elif k == Qt.Key.Key_1:
            self.one_to_one()
        elif k in (Qt.Key.Key_Left, Qt.Key.Key_Right, Qt.Key.Key_Up, Qt.Key.Key_Down):
            step = 40.0
            self._ox += -step if k == Qt.Key.Key_Left else step if k == Qt.Key.Key_Right else 0.0
            self._oy += -step if k == Qt.Key.Key_Up else step if k == Qt.Key.Key_Down else 0.0
            self._clamp_pan()
            self.update()
        else:
            super().keyPressEvent(event)

    # --- painting -----------------------------------------------------------
    def paintEvent(self, event) -> None:  # noqa: N802 (full override, no super)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        rect = QRectF(0.5, 0.5, self.width() - 1.0, self.height() - 1.0)
        painter.fillRect(self.rect(), QColor("#12151b"))
        painter.setPen(QPen(QColor("#363d4a"), 1))
        painter.drawRoundedRect(rect, 8, 8)
        if self._pix is None or self._pix.isNull():
            painter.setPen(QColor("#5b6472"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "no image")
            painter.end()
            return

        s = self._scale()
        w, h = self._pix.width(), self._pix.height()
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, True)
        painter.drawPixmap(
            QRectF(self._ox, self._oy, w * s, h * s), self._pix, QRectF(0, 0, w, h)
        )

        for i, (sx, sy) in enumerate(self._pts):
            wp = self.source_to_widget(sx, sy)
            if wp is not None:
                self._draw_reticle(painter, wp[0], wp[1], i + 1, i == self._active)

        panning = self._press is not None and self._moved
        if self._cursor is not None and not panning:
            self._draw_cursor(painter)
            if self._loupe_on:
                self._draw_loupe(painter)
        painter.end()

    def _draw_reticle(self, painter, wx, wy, index, active) -> None:
        R, ARM, GAP = 13.0, 22.0, 5.0
        core = QColor(63, 185, 80) if active else QColor(90, 190, 255)
        halo_pen = QPen(QColor(0, 0, 0, 205), 5)
        halo_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        core_pen = QPen(core, 2)
        core_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        for pen, is_core in ((halo_pen, False), (core_pen, True)):
            painter.setPen(pen)
            painter.drawEllipse(QPointF(wx, wy), R, R)
            painter.drawLine(QPointF(wx - ARM, wy), QPointF(wx - GAP, wy))
            painter.drawLine(QPointF(wx + GAP, wy), QPointF(wx + ARM, wy))
            painter.drawLine(QPointF(wx, wy - ARM), QPointF(wx, wy - GAP))
            painter.drawLine(QPointF(wx, wy + GAP), QPointF(wx, wy + ARM))
            if is_core:
                painter.drawEllipse(QPointF(wx, wy), 1.5, 1.5)
        # index chip
        chip = QRectF(wx + 12, wy - 27, 12 + 9 * len(str(index)), 17)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(0, 0, 0, 180))
        painter.drawRoundedRect(chip, 4, 4)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QColor(235, 240, 245))
        f = QFont(painter.font())
        f.setBold(True)
        painter.setFont(f)
        painter.drawText(chip, Qt.AlignmentFlag.AlignCenter, str(index))

    def _draw_cursor(self, painter) -> None:
        cx, cy = self._cursor.x(), self._cursor.y()
        aw, ah = self.width(), self.height()
        for pen in (QPen(QColor(0, 0, 0, 140), 3), QPen(QColor(255, 255, 255, 230), 1)):
            painter.setPen(pen)
            painter.drawLine(QPointF(0, cy), QPointF(aw, cy))
            painter.drawLine(QPointF(cx, 0), QPointF(cx, ah))
        src = self.widget_to_source(cx, cy, clamp=True)
        if src is not None:
            painter.setPen(QColor(255, 255, 255, 230))
            painter.drawText(QPointF(cx + 10, cy - 8), f"{src[0]:.0f}, {src[1]:.0f}")

    def _draw_loupe(self, painter) -> None:
        cx, cy = self._cursor.x(), self._cursor.y()
        p = self.widget_to_source(cx, cy)
        if p is None:
            return
        Lr, M = float(self.LOUPE_R), self.LOUPE_M
        # Put the loupe in the opposite corner from the cursor so it never hides it.
        lx = Lr + 14 if cx > self.width() / 2 else self.width() - Lr - 14
        ly = Lr + 14 if cy > self.height() / 2 else self.height() - Lr - 14
        lc = QPointF(lx, ly)
        half = Lr / M
        src = QRectF(p[0] - half, p[1] - half, 2 * half, 2 * half)
        dst = QRectF(lx - Lr, ly - Lr, 2 * Lr, 2 * Lr)
        painter.save()
        clip = QPainterPath()
        clip.addEllipse(lc, Lr, Lr)
        painter.setClipPath(clip)
        painter.fillRect(dst, QColor("#12151b"))
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform, False)  # pixels
        painter.drawPixmap(dst, self._pix, src)
        # crosshair at the magnified target pixel
        for pen in (QPen(QColor(0, 0, 0, 160), 3), QPen(QColor(255, 255, 255, 235), 1)):
            painter.setPen(pen)
            painter.drawLine(QPointF(lx - Lr, ly), QPointF(lx + Lr, ly))
            painter.drawLine(QPointF(lx, ly - Lr), QPointF(lx, ly + Lr))
        painter.restore()
        for pen in (QPen(QColor(0, 0, 0, 180), 4), QPen(QColor(90, 190, 255), 2)):
            painter.setPen(pen)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(lc, Lr, Lr)


class StatusPill(QLabel):
    """A small rounded status chip; call set_state to recolor."""

    def __init__(self, text: str = "", parent=None) -> None:
        super().__init__(text, parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.set_state(text, "idle")

    def set_state(self, text: str, kind: str) -> None:
        self.setText(text)
        self.setStyleSheet(theme.pill_style(kind))
