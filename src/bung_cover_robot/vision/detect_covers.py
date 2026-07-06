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
            reason = self._classify(c, circles, w, h, robot_xy, validator)
            covers.append(CoverDetection(c, robot_xy, reason == "ok", reason))
        return CoverDetectionResult(covers)

    def _classify(
        self,
        c: Circle,
        others: List[Circle],
        w: int,
        h: int,
        robot_xy: Optional[Tuple[float, float]],
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
        if robot_xy is not None and validator is not None:
            res = validator.validate(*robot_xy)
            if not res.ok:
                return f"unreachable: {res.reason}"
        return "ok"
