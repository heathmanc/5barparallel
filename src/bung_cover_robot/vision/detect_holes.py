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
    find_round_objects,
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
    # Pick region (x, y, w, h px): holes centred inside it are dropped — that area
    # is the cover chute, not a drop target. Everything *outside* it is a candidate
    # drop hole. None => consider the whole search area.
    exclude_roi: Optional[ROI] = None
    # Keep the largest subset of centres that lie on a common line (RANSAC-style)
    # instead of requiring every detected blob to be collinear — so a stray
    # false-positive off the row can't spoil the whole detection. The drop holes
    # are a straight row; scattered clutter is discarded.
    select_collinear_subset: bool = False
    # Finder: "shape" = region outline (color-agnostic — a hole may be dark OR
    # bright-centered); "blob" = dark fill; "auto" = shape when confined to a
    # battery/ROI, else blob.
    method: str = "auto"
    shape_min_circularity: float = 0.6
    shape_min_solidity: float = 0.9
    # Physical-size gate (per recipe): reject blobs whose real diameter is off the
    # expected drop-hole size. Needs a calibration (to_robot); 0 disables.
    expected_diameter_mm: float = 0.0
    diameter_tolerance: float = 0.25


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

    def detect(self, frame: np.ndarray, to_robot=None) -> HoleDetectionResult:
        cfg = self.config
        roi = cfg.roi
        if roi is None and cfg.auto_battery_roi:
            roi = find_battery_roi(frame)
        # Region outlines are color-agnostic; use them once confined to a region.
        use_shape = cfg.method == "shape" or (
            cfg.method == "auto" and roi is not None)
        sub, ox, oy = crop_roi(frame, roi)
        if use_shape:
            found = find_round_objects(
                sub, cfg.min_diameter_px, cfg.max_diameter_px, cfg.blur,
                cfg.shape_min_circularity, cfg.shape_min_solidity)
        else:
            found = find_blobs(
                sub, True, cfg.min_diameter_px, cfg.max_diameter_px,
                cfg.min_circularity, cfg.blur)
        circles = offset_circles(found, ox, oy)

        # The pick region (cover chute) is not a drop target — drop only the holes
        # whose centre falls inside it, keeping everything outside.
        if cfg.exclude_roi is not None:
            ex, ey, ew, eh = cfg.exclude_roi
            circles = [c for c in circles
                       if not (ex <= c.cx <= ex + ew and ey <= c.cy <= ey + eh)]

        # Physical-size gate: keep only blobs the right real diameter (drop-hole).
        if cfg.expected_diameter_mm > 0 and to_robot is not None:
            lo = cfg.expected_diameter_mm * (1.0 - cfg.diameter_tolerance)
            hi = cfg.expected_diameter_mm * (1.0 + cfg.diameter_tolerance)
            circles = [c for c in circles
                       if lo <= _diameter_mm(c, to_robot) <= hi]

        if len(circles) < 2:
            return HoleDetectionResult(circles, None, float("inf"), False,
                                       f"found {len(circles)} holes (need >= 2 to fit)")

        if cfg.select_collinear_subset:
            holes, line, residual = _select_collinear(circles, cfg.collinear_tol_px)
        else:
            line, residual, order = _fit_line(circles)
            holes = [circles[i] for i in order]
        count_ok = cfg.expected_count <= 0 or len(holes) == cfg.expected_count
        ok = count_ok and residual <= cfg.collinear_tol_px
        if not count_ok:
            reason = f"found {len(holes)} holes, expected {cfg.expected_count}"
        elif residual > cfg.collinear_tol_px:
            reason = f"holes not collinear (residual {residual:.1f} px > {cfg.collinear_tol_px:.1f})"
        else:
            reason = "ok"
        return HoleDetectionResult(holes, line, residual, ok, reason)


def _diameter_mm(c: Circle, to_robot) -> float:
    """Real diameter (mm) via the calibration — mean of the x- and y-spans."""
    ax, bx = to_robot(c.cx - c.radius, c.cy), to_robot(c.cx + c.radius, c.cy)
    ay, by = to_robot(c.cx, c.cy - c.radius), to_robot(c.cx, c.cy + c.radius)
    dx = float(np.hypot(bx[0] - ax[0], bx[1] - ax[1]))
    dy = float(np.hypot(by[0] - ay[0], by[1] - ay[1]))
    return 0.5 * (dx + dy)


def _select_collinear(circles: List[Circle], tol: float):
    """Largest subset of centres on a common line (within ``tol`` px), ordered
    along it. RANSAC-lite: every centre pair defines a candidate line; keep the
    line with the most inliers. Returns ``(subset, line, residual)`` — the fit is
    over the inliers, so a stray blob off the row can't inflate the residual."""
    pts = np.array([[c.cx, c.cy] for c in circles], dtype=np.float64)
    n = len(circles)
    best: Optional[np.ndarray] = None
    for i in range(n):
        for j in range(i + 1, n):
            d = pts[j] - pts[i]
            length = float(np.hypot(d[0], d[1]))
            if length < 1e-6:
                continue
            nx, ny = -d[1] / length, d[0] / length      # unit normal
            dist = np.abs((pts[:, 0] - pts[i, 0]) * nx + (pts[:, 1] - pts[i, 1]) * ny)
            inliers = np.nonzero(dist <= tol)[0]
            if best is None or len(inliers) > len(best):
                best = inliers
    idx = best if best is not None else np.arange(n)
    subset = [circles[k] for k in idx]
    if len(subset) < 2:
        return subset, None, float("inf")
    line, residual, order = _fit_line(subset)
    return [subset[k] for k in order], line, residual


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
