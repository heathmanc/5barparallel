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


def demo_transform():
    """A plausible pixel->robot HomographyTransform for the demo_frame scene, so
    the Vision tab can show cover reachability without a real calibration. Maps
    the battery hole row into the work zone; corner covers fall outside it."""
    from ..vision.calibration import HomographyTransform

    sx, x0 = 300.0 / 406.0, 379.0     # px -> robot X (mm), holes centered
    sy, y_at, y_ref = -0.25, 259.0, 250.0  # px -> robot Y (mm), holes at Y=250
    H = np.array(
        [[sx, 0.0, -sx * x0], [0.0, sy, y_ref - sy * y_at], [0.0, 0.0, 1.0]]
    )
    return HomographyTransform.from_matrix(H, name="demo")


def annotate_points(frame: np.ndarray, points, labels=None) -> np.ndarray:
    """Draw numbered crosshair markers at pixel ``points`` (list of (x,y)).

    Used by the calibration tab to show which points the operator has picked.
    Returns a copy so the captured frame is never mutated. BGR uint8.
    """
    import cv2

    out = frame.copy()
    for i, (px, py) in enumerate(points):
        p = (int(round(px)), int(round(py)))
        cv2.circle(out, p, 9, (60, 220, 90), 2)
        cv2.line(out, (p[0] - 13, p[1]), (p[0] + 13, p[1]), (60, 220, 90), 1)
        cv2.line(out, (p[0], p[1] - 13), (p[0], p[1] + 13), (60, 220, 90), 1)
        text = labels[i] if labels is not None else str(i + 1)
        cv2.putText(
            out, text, (p[0] + 11, p[1] - 11),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 245, 150), 1, cv2.LINE_AA,
        )
    return out


def adjust_preview(
    frame: np.ndarray, brightness: float = 0.0, contrast: float = 1.0, gamma: float = 1.0
) -> np.ndarray:
    """Apply brightness/contrast/gamma for the mock-camera preview so the control
    sliders visibly change the image (a real Basler applies these itself)."""
    # Fast path: identity settings -> no per-frame float conversion (this runs on
    # every live frame, so the no-op case must be free).
    if abs(brightness) < 1e-3 and abs(contrast - 1.0) < 1e-3 and abs(gamma - 1.0) < 1e-3:
        return frame
    f = frame.astype(np.float32) * float(contrast) + float(brightness) * 255.0
    f = np.clip(f, 0, 255)
    if abs(gamma - 1.0) > 1e-3:
        f = ((f / 255.0) ** (1.0 / max(gamma, 0.01))) * 255.0
    return np.clip(f, 0, 255).astype(np.uint8)


def downscale_for_preview(frame: np.ndarray, max_dim: int = 960) -> np.ndarray:
    """Shrink a frame so its longest side is <= max_dim, for a cheap live preview.
    A multi-MP sensor frame is far larger than any on-screen view; scaling it down
    once (off the GUI thread, in the grabber) makes the QImage/QPixmap/scaled()
    path cheap. Returns the frame unchanged if already small enough."""
    import cv2

    h, w = frame.shape[:2]
    longest = max(h, w)
    if longest <= max_dim:
        return frame
    s = max_dim / float(longest)
    return cv2.resize(
        frame, (max(1, int(w * s)), max(1, int(h * s))), interpolation=cv2.INTER_AREA
    )
