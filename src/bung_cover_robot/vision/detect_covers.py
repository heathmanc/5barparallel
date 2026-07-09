"""Cover detection (Claude.md §12).

ROI crop -> gray -> threshold (bright) -> contours -> circularity + area filter ->
reject covers that are near a frame/tray edge, crowded (touching a neighbour), or
outside the reachable workspace. Re-run every pick cycle (loose covers shift).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

import numpy as np

from ..robot.workspace import WorkspaceValidator
from .detection import (
    ROI,
    Circle,
    crop_roi,
    find_blobs,
    find_hough_circles,
    find_round_objects,
    offset_circles,
)

# pixel (cx, cy) -> robot-frame (x, y) mm; supplied by calibration when available.
PixelToRobot = Callable[[float, float], Tuple[float, float]]


@dataclass
class CoverDetectorConfig:
    min_diameter_px: float = 15.0
    max_diameter_px: float = 200.0        # covers can be large in a close overhead view
    min_circularity: float = 0.75
    blur: int = 5
    threshold: Optional[int] = None       # None => Otsu
    edge_margin_px: float = 8.0           # reject covers this close to the frame border
    min_gap_px: float = 6.0               # min clear gap to a neighbour (else "crowded")
    reject_crowded: bool = True           # reject a cover touching a neighbour (off for a chute)
    roi: Optional[ROI] = None
    # Operator-drawn pick region (x, y, w, h in pixels): covers centred outside it
    # are rejected ("outside pick region"). None => pick anywhere reachable.
    pick_roi: Optional[ROI] = None
    # Finder: "shape" = region outline (color-agnostic, D-shape tolerant); "blob" =
    # dark/bright fill; "auto" = shape when a pick region is drawn (search confined
    # to it), else blob. Covers vary in color AND are D-shaped, so shape-in-region.
    # "shape" (region+edge outline), "hough" (Hough circles — best for a clean
    # round cover), "blob" (dark/bright fill), or "auto" (shape within a pick region).
    method: str = "auto"
    shape_min_circularity: float = 0.6    # relaxed so a flat-sided (D) cover passes
    shape_min_solidity: float = 0.9       # convexity: rejects grain, keeps a D
    shape_canny_lo_frac: float = 0.33     # edge sensitivity (of median): lower = more
    shape_canny_hi_frac: float = 0.66
    hough_param1: float = 100.0           # Hough edge threshold: lower = softer edges
    hough_param2: float = 30.0            # Hough accumulator: lower = more circles
    # Physical-size gate (per recipe): reject blobs whose real diameter is off the
    # expected cover size. Needs a calibration (to_robot); 0 disables the check.
    expected_diameter_mm: float = 0.0
    diameter_tolerance: float = 0.25      # accept expected * (1 ± tolerance)


@dataclass
class CoverDetection:
    circle: Circle
    robot_xy: Optional[Tuple[float, float]]
    accepted: bool
    reason: str


@dataclass
class CoverDetectionResult:
    covers: List[CoverDetection]

    @property
    def accepted(self) -> List[CoverDetection]:
        return [c for c in self.covers if c.accepted]

    @property
    def count(self) -> int:
        return len(self.covers)


class CoverDetector:
    def __init__(self, config: Optional[CoverDetectorConfig] = None) -> None:
        self.config = config or CoverDetectorConfig()

    def detect(
        self,
        frame: np.ndarray,
        to_robot: Optional[PixelToRobot] = None,
        validator: Optional[WorkspaceValidator] = None,
    ) -> CoverDetectionResult:
        """Detect covers and classify each as pickable or not.

        If ``to_robot`` (pixel->robot XY, from calibration) and ``validator`` are
        given, covers outside the clean workspace are rejected. Without them, only
        geometric quality (size, circularity, edge, crowding) is checked.
        """
        cfg = self.config
        h, w = frame.shape[:2]
        # Region (outline) finding is color- and D-shape-agnostic; confine it to the
        # pick region so clutter can't accumulate. "auto" uses it once a pick region
        # is drawn.
        use_shape = cfg.method == "shape" or (
            cfg.method == "auto" and cfg.pick_roi is not None)
        confined = cfg.method in ("shape", "hough") or use_shape
        region = cfg.pick_roi if (confined and cfg.pick_roi is not None) else cfg.roi
        sub, ox, oy = crop_roi(frame, region)
        if cfg.method == "hough":
            found = find_hough_circles(
                sub, cfg.min_diameter_px, cfg.max_diameter_px, cfg.blur,
                param1=cfg.hough_param1, param2=cfg.hough_param2)
        elif use_shape:
            found = find_round_objects(
                sub, cfg.min_diameter_px, cfg.max_diameter_px, cfg.blur,
                cfg.shape_min_circularity, cfg.shape_min_solidity,
                cfg.shape_canny_lo_frac, cfg.shape_canny_hi_frac)
        else:
            found = find_blobs(sub, False, cfg.min_diameter_px, cfg.max_diameter_px,
                               cfg.min_circularity, cfg.blur)
        circles = offset_circles(found, ox, oy)

        covers: List[CoverDetection] = []
        for c in circles:
            robot_xy = to_robot(c.cx, c.cy) if to_robot else None
            diameter_mm = self._physical_diameter_mm(c, to_robot)
            reason = self._classify(c, circles, w, h, robot_xy, diameter_mm, validator)
            covers.append(CoverDetection(c, robot_xy, reason == "ok", reason))
        return CoverDetectionResult(covers)

    def _physical_diameter_mm(
        self, c: Circle, to_robot: Optional[PixelToRobot]
    ) -> Optional[float]:
        """Real cover diameter (mm) via the calibration — the mean of the x- and
        y-spans across the blob, so it's robust to a slightly anisotropic scale.
        None when there's no calibration to measure with."""
        if to_robot is None:
            return None
        ax, bx = to_robot(c.cx - c.radius, c.cy), to_robot(c.cx + c.radius, c.cy)
        ay, by = to_robot(c.cx, c.cy - c.radius), to_robot(c.cx, c.cy + c.radius)
        dx = float(np.hypot(bx[0] - ax[0], bx[1] - ax[1]))
        dy = float(np.hypot(by[0] - ay[0], by[1] - ay[1]))
        return 0.5 * (dx + dy)

    def _classify(
        self,
        c: Circle,
        others: List[Circle],
        w: int,
        h: int,
        robot_xy: Optional[Tuple[float, float]],
        diameter_mm: Optional[float],
        validator: Optional[WorkspaceValidator],
    ) -> str:
        cfg = self.config
        if cfg.pick_roi is not None:
            rx, ry, rw, rh = cfg.pick_roi
            if not (rx <= c.cx <= rx + rw and ry <= c.cy <= ry + rh):
                return "outside pick region"
        m = cfg.edge_margin_px
        if c.cx - c.radius < m or c.cy - c.radius < m or c.cx + c.radius > w - m or c.cy + c.radius > h - m:
            return "near edge"
        if cfg.reject_crowded:
            for o in others:
                if o is c:
                    continue
                dist = float(np.hypot(c.cx - o.cx, c.cy - o.cy))
                if dist < c.radius + o.radius + cfg.min_gap_px:
                    return "crowded (touching neighbour)"
        if cfg.expected_diameter_mm > 0 and diameter_mm is not None:
            lo = cfg.expected_diameter_mm * (1.0 - cfg.diameter_tolerance)
            hi = cfg.expected_diameter_mm * (1.0 + cfg.diameter_tolerance)
            if not (lo <= diameter_mm <= hi):
                return (
                    f"wrong size ({diameter_mm:.0f} mm, expected "
                    f"~{cfg.expected_diameter_mm:.0f} mm)"
                )
        if robot_xy is not None and validator is not None:
            res = validator.validate(*robot_xy)
            if not res.ok:
                return f"unreachable: {res.reason}"
        return "ok"
