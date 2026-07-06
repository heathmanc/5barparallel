"""Image helpers for the GUI: numpy <-> Qt, a synthetic demo scene, and a
preview adjustment so the mock camera controls have a visible effect."""

from __future__ import annotations

import numpy as np


def ndarray_to_qpixmap(frame: np.ndarray):
    """Convert an OpenCV BGR (H,W,3) or gray (H,W) uint8 array to a QPixmap."""
    from PySide6.QtGui import QImage, QPixmap

    if frame.ndim == 2:
        h, w = frame.shape
        img = QImage(bytes(frame.data), w, h, w, QImage.Format.Format_Grayscale8)
    else:
        h, w, _ = frame.shape
        rgb = np.ascontiguousarray(frame[:, :, ::-1])  # BGR -> RGB
        img = QImage(bytes(rgb.data), w, h, 3 * w, QImage.Format.Format_RGB888)
    return QPixmap.fromImage(img)


def demo_frame(w: int = 760, h: int = 520) -> np.ndarray:
    """A synthetic overhead scene (battery + 6 holes + loose covers) so the
    vision views look meaningful in dry-run. BGR uint8."""
    import cv2

    img = np.full((h, w, 3), (30, 27, 25), np.uint8)
    cv2.rectangle(img, (0, int(h * 0.28)), (w, int(h * 0.72)), (48, 43, 39), -1)
    bx0, by0 = int(w * 0.18), int(h * 0.37)
    bx1, by1 = int(w * 0.82), int(h * 0.63)
    cv2.rectangle(img, (bx0, by0), (bx1, by1), (70, 84, 92), -1)
    cv2.rectangle(img, (bx0, by0), (bx1, by1), (120, 140, 150), 2)
    cy = (by0 + by1) // 2
    for i in range(6):
        cx = int(bx0 + (i + 0.5) * (bx1 - bx0) / 6)
        cv2.circle(img, (cx, cy), 15, (18, 18, 18), -1)
        cv2.circle(img, (cx, cy), 15, (150, 160, 170), 2)
    for cx, cyp in [
        (int(w * 0.10), int(h * 0.16)), (int(w * 0.90), int(h * 0.84)),
        (int(w * 0.13), int(h * 0.85)), (int(w * 0.88), int(h * 0.15)),
        (int(w * 0.50), int(h * 0.86)),
    ]:
        cv2.circle(img, (cx, cyp), 17, (60, 120, 190), -1)
        cv2.circle(img, (cx, cyp), 17, (120, 180, 240), 2)
    return img


def adjust_preview(
    frame: np.ndarray, brightness: float = 0.0, contrast: float = 1.0, gamma: float = 1.0
) -> np.ndarray:
    """Apply brightness/contrast/gamma for the mock-camera preview so the control
    sliders visibly change the image (a real Basler applies these itself)."""
    f = frame.astype(np.float32) * float(contrast) + float(brightness) * 255.0
    f = np.clip(f, 0, 255)
    if abs(gamma - 1.0) > 1e-3:
        f = ((f / 255.0) ** (1.0 / max(gamma, 0.01))) * 255.0
    return np.clip(f, 0, 255).astype(np.uint8)
