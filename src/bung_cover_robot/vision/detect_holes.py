"""Hole detection (Claude.md §12).

ROI crop -> gray -> blur -> threshold (dark) -> contours -> circularity + diameter
filter -> line-fit the centres (the 6 vent holes are collinear along the battery)
as a sanity check.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .detection import (
    ROI,
    Circle,
    crop_roi,
    find_battery_roi,
    find_blobs,
    offset_circles,
)


@dataclass
class HoleDetectorConfig:
    expected_count: int = 6
    min_diameter_px: float = 10.0
    max_diameter_px: float = 60.0
    min_circularity: float = 0.7
    blur: int = 5
    threshold: Optional[int] = None       # None => Otsu
    collinear_tol_px: float = 6.0         # max perpendicular residual to the fit line
    roi: Optional[ROI] = None
    auto_battery_roi: bool = True         # confine to the battery when no roi is set


@dataclass
class HoleDetectionResult:
    holes: List[Circle]                   # sorted along the battery axis
    line: Optional[Tuple[float, float, float, float]]  # (vx, vy, x0, y0)
    max_residual_px: float
    ok: bool
    reason: str

    @property
    def count(self) -> int:
        return len(self.holes)


class HoleDetector:
    def __init__(self, config: Optional[HoleDetectorConfig] = None) -> None:
        self.config = config or HoleDetectorConfig()

    def detect(self, frame: np.ndarray) -> HoleDetectionResult:
        cfg = self.config
        roi = cfg.roi
        if roi is None and cfg.auto_battery_roi:
            roi = find_battery_roi(frame)
        sub, ox, oy = crop_roi(frame, roi)
        circles = find_blobs(
            sub, True, cfg.min_diameter_px, cfg.max_diameter_px,
            cfg.min_circularity, cfg.blur,
        )
        circles = offset_circles(circles, ox, oy)

        if len(circles) < 2:
            return HoleDetectionResult(circles, None, float("inf"), False,
                                       f"found {len(circles)} holes (need >= 2 to fit)")

        line, residual, order = _fit_line(circles)
        holes = [circles[i] for i in order]
        ok = len(holes) == cfg.expected_count and residual <= cfg.collinear_tol_px
        if len(holes) != cfg.expected_count:
            reason = f"found {len(holes)} holes, expected {cfg.expected_count}"
        elif residual > cfg.collinear_tol_px:
            reason = f"holes not collinear (residual {residual:.1f} px > {cfg.collinear_tol_px:.1f})"
        else:
            reason = "ok"
        return HoleDetectionResult(holes, line, residual, ok, reason)


def _fit_line(circles: List[Circle]):
    import cv2

    pts = np.array([[c.cx, c.cy] for c in circles], dtype=np.float32)
    vx, vy, x0, y0 = (float(v) for v in cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).flatten())
    # perpendicular distance of each centre to the line
    nx, ny = -vy, vx
    residuals = np.abs((pts[:, 0] - x0) * nx + (pts[:, 1] - y0) * ny)
    # order the holes by projection along the line direction
    proj = (pts[:, 0] - x0) * vx + (pts[:, 1] - y0) * vy
    order = list(np.argsort(proj))
    return (vx, vy, x0, y0), float(residuals.max()), order
