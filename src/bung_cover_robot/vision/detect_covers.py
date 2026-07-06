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
from .detection import ROI, Circle, crop_roi, find_blobs, offset_circles

# pixel (cx, cy) -> robot-frame (x, y) mm; supplied by calibration when available.
PixelToRobot = Callable[[float, float], Tuple[float, float]]


@dataclass
class CoverDetectorConfig:
    min_diameter_px: float = 15.0
    max_diameter_px: float = 80.0
    min_circularity: float = 0.75
    blur: int = 5
    threshold: Optional[int] = None       # None => Otsu
    edge_margin_px: float = 8.0           # reject covers this close to the frame border
    min_gap_px: float = 6.0               # min clear gap to a neighbour (else "crowded")
    roi: Optional[ROI] = None
    # Physical-size gate (per recipe): reject blobs whose real diameter is off the
    # expected cover size. Needs a calibration (to_robot); 0 disables the check.
    expected_diameter_mm: float = 0.0
    diameter_tolerance: float = 0.4       # accept expected * (1 ± tolerance)


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
        sub, ox, oy = crop_roi(frame, cfg.roi)
        circles = offset_circles(
            find_blobs(sub, False, cfg.min_diameter_px, cfg.max_diameter_px,
                       cfg.min_circularity, cfg.blur),
            ox, oy,
        )

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
        m = cfg.edge_margin_px
        if c.cx - c.radius < m or c.cy - c.radius < m or c.cx + c.radius > w - m or c.cy + c.radius > h - m:
            return "near edge"
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
