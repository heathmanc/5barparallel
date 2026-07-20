"""Workspace / singularity validation — the go/no-go guard before the PLC.

A 5-bar has two failure modes (Claude.md §10):

  * Parallel (direct-kinematic) singularity: the two *distal* links become
    collinear. The mechanism loses control of force along that line and can
    flip assembly modes. Forms a band arcing through the workspace.
  * Serial (inverse-kinematic) singularity: an arm reaches full extension or
    full fold (proximal & distal collinear) — the outer workspace boundary,
    where the arm is compliant and placement accuracy degrades.

WorkspaceValidator measures the margin to both and rejects targets that get
close, plus a stiffness (reach-fraction) cap. NOTHING should ever be written to
the PLC without passing validate().
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

from .fivebar_kinematics import (
    FiveBarConfig,
    FiveBarKinematics,
    JointTarget,
    KinematicsError,
    Point,
)

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]


# Max |forward(inverse(target)) - target| for a target to count as realizable
# at the intended assembly. A valid pose round-trips to ~1e-13 mm; the mirror
# pose is hundreds of mm off, so anything but a hair means wrong-assembly.
_ASSEMBLY_TOL_MM = 1e-3


@dataclass(frozen=True)
class SingularityLimits:
    """Guard thresholds (Claude.md §3.6, §9). Defaults are the verified values."""

    parallel_min_deg: float = 20.0
    serial_min_deg: float = 15.0
    reach_fraction_max: float = 0.85

    @classmethod
    def from_yaml(cls, path: str | Path) -> "SingularityLimits":
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required for SingularityLimits.from_yaml")
        data = yaml.safe_load(Path(path).read_text()) or {}
        sec = data.get("singularity", {})
        base = cls()
        return cls(
            parallel_min_deg=float(sec.get("parallel_min_deg", base.parallel_min_deg)),
            serial_min_deg=float(sec.get("serial_min_deg", base.serial_min_deg)),
            reach_fraction_max=float(
                sec.get("reach_fraction_max", base.reach_fraction_max)
            ),
        )


@dataclass(frozen=True)
class WorkArea:
    """An axis-aligned rectangle in the robot frame (mm) — the usable work area."""

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center(self) -> "tuple[float, float]":
        return ((self.x_min + self.x_max) / 2.0, (self.y_min + self.y_max) / 2.0)

    def contains(self, x: float, y: float) -> bool:
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max


def largest_safe_rectangle(
    validator: "WorkspaceValidator",
    step: float = 4.0,
    inset: float = 0.0,
    max_cells: int = 200,
) -> "WorkArea | None":
    """The largest axis-aligned rectangle whose every point passes ``validate``.

    Grid-samples the geometry's safe region (reachable + within joint limits +
    clear of both singularities + under the stiffness cap), then finds the biggest
    all-safe rectangle (maximal-rectangle-in-a-binary-matrix). This IS the usable
    work area for the current arms — it shrinks/grows automatically with L1/L2/base
    spacing, so a smaller robot yields a smaller area instead of being refused.

    ``inset`` shrinks the result by a safety margin on every side. Returns None if
    the geometry has no safe area at all. ``max_cells`` caps the grid per axis (the
    sample step is coarsened to fit), so the scan stays fast on any geometry.
    """
    cfg = validator.config
    reach = cfg.max_reach_mm
    x0, x1 = -reach, reach
    y0, y1 = 0.0, reach
    step = max(step, (x1 - x0) / max_cells, (y1 - y0) / max_cells)
    nx = int((x1 - x0) / step) + 1
    ny = int((y1 - y0) / step) + 1
    mask = [
        [1 if validator.is_safe(x0 + ix * step, y0 + iy * step) else 0
         for ix in range(nx)]
        for iy in range(ny)
    ]
    rect = _max_true_rectangle(mask)
    if rect is None:
        return None
    top, left, bottom, right = rect
    area = WorkArea(
        x_min=x0 + left * step, x_max=x0 + right * step,
        y_min=y0 + top * step, y_max=y0 + bottom * step,
    )
    if inset > 0.0:
        area = WorkArea(area.x_min + inset, area.x_max - inset,
                        area.y_min + inset, area.y_max - inset)
        if area.width <= 0.0 or area.height <= 0.0:
            return None
    return area


def _max_true_rectangle(mask: List[List[int]]):
    """(top, left, bottom, right) inclusive indices of the largest all-1 rectangle
    in a binary matrix, or None if it's all zeros. Histogram method, O(rows*cols)."""
    if not mask or not mask[0]:
        return None
    cols = len(mask[0])
    heights = [0] * cols
    best_area = 0
    best = None
    for i, row in enumerate(mask):
        for j in range(cols):
            heights[j] = heights[j] + 1 if row[j] else 0
        # largest rectangle in this histogram, tracking its position
        stack: List[int] = []   # indices of increasing bar heights
        j = 0
        while j <= cols:
            h = heights[j] if j < cols else 0
            if not stack or h >= heights[stack[-1]]:
                stack.append(j)
                j += 1
            else:
                top_idx = stack.pop()
                height = heights[top_idx]
                left = stack[-1] + 1 if stack else 0
                right = j - 1
                area = height * (right - left + 1)
                if height > 0 and area > best_area:
                    best_area = area
                    best = (i - height + 1, left, i, right)
    return best


@dataclass
class ValidationResult:
    """Outcome of a single validate() call. ``metrics`` always populated when
    the point was at least reachable."""

    ok: bool
    reason: str
    metrics: Dict[str, float] = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.ok


class WorkspaceValidator:
    """The guard. validate() never raises; it returns a ValidationResult."""

    def __init__(
        self,
        kinematics: FiveBarKinematics | None = None,
        limits: SingularityLimits | None = None,
    ) -> None:
        self.kin = kinematics or FiveBarKinematics()
        self.limits = limits or SingularityLimits()

    @property
    def config(self) -> FiveBarConfig:
        return self.kin.config

    # --- individual metrics -------------------------------------------------
    def parallel_margin_deg(self, x: float, y: float) -> float:
        """Degrees from the two distal links being collinear."""
        jt = self.kin.inverse(x, y)
        return self._parallel_margin(jt)

    def serial_margin_deg(self, x: float, y: float) -> float:
        """Degrees from either arm being straight (extended) or folded."""
        jt = self.kin.inverse(x, y)
        return self._serial_margin(jt)

    def reach_fraction(self, x: float, y: float) -> float:
        """TCP distance / full reach for the more-extended arm (stiffness proxy)."""
        cfg = self.config
        return max(
            math.hypot(x - cfg.left_base[0], y - cfg.left_base[1]),
            math.hypot(x - cfg.right_base[0], y - cfg.right_base[1]),
        ) / cfg.max_reach_mm

    # --- the guard ----------------------------------------------------------
    def validate(self, x: float, y: float) -> ValidationResult:
        """Full go/no-go, in order: reachable -> joint limits -> parallel
        singularity -> serial singularity -> stiffness cap."""
        try:
            jt = self.kin.inverse(x, y)
        except KinematicsError as exc:
            return ValidationResult(False, f"unreachable: {exc}", {})

        parallel = self._parallel_margin(jt)
        serial = self._serial_margin(jt)
        reach = self.reach_fraction(x, y)
        metrics = {
            "left_deg": jt.left_deg,
            "right_deg": jt.right_deg,
            "parallel_margin_deg": parallel,
            "serial_margin_deg": serial,
            "reach_fraction": reach,
        }

        # Wrong-assembly / mirror guard. inverse() solves each arm independently
        # with fixed up/down branches; those branches only realize the intended
        # +Y assembly in the reaching half-plane. For a target the mechanism
        # cannot actually assemble at (e.g. below the shoulder line), inverse()
        # still returns valid-looking angles, but driving them lands the TCP at
        # the mirror pose — hundreds of mm away. Reject anything whose forward
        # solution doesn't return to the requested point. (Round-trips to ~1e-13
        # for a realizable pose, so the tolerance is generous.)
        fx, fy = self.kin.forward(jt.left_deg, jt.right_deg)
        if math.hypot(fx - x, fy - y) > _ASSEMBLY_TOL_MM:
            return ValidationResult(
                False,
                f"wrong-assembly / mirror pose: angles realize ({fx:.1f}, {fy:.1f}), "
                f"not ({x:.1f}, {y:.1f}) — target not reachable in the +Y work zone",
                metrics,
            )

        if not self.kin.within_joint_limits(jt.left_deg):
            return ValidationResult(
                False, f"left shoulder {jt.left_deg:.1f} deg out of joint limits", metrics
            )
        if not self.kin.within_joint_limits(jt.right_deg):
            return ValidationResult(
                False,
                f"right shoulder {jt.right_deg:.1f} deg out of joint limits",
                metrics,
            )
        if parallel < self.limits.parallel_min_deg:
            return ValidationResult(
                False,
                f"parallel-singularity margin {parallel:.1f} deg "
                f"< {self.limits.parallel_min_deg:.1f} deg",
                metrics,
            )
        if serial < self.limits.serial_min_deg:
            return ValidationResult(
                False,
                f"serial-singularity margin {serial:.1f} deg "
                f"< {self.limits.serial_min_deg:.1f} deg",
                metrics,
            )
        if reach > self.limits.reach_fraction_max:
            return ValidationResult(
                False,
                f"reach fraction {reach:.3f} > cap {self.limits.reach_fraction_max:.2f}",
                metrics,
            )
        return ValidationResult(True, "ok", metrics)

    def is_safe(self, x: float, y: float) -> bool:
        return self.validate(x, y).ok

    def scan(
        self,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        step: float = 5.0,
    ) -> List[Dict[str, float]]:
        """Offline map generation only. Returns one record per grid point with
        its metrics and ok flag. Not for the runtime path."""
        if step <= 0:
            raise ValueError("step must be positive")
        records: List[Dict[str, float]] = []
        eps = step * 1e-9  # tolerate float drift without overshooting the bound
        n_x = int(math.floor((x_max - x_min + eps) / step)) + 1
        n_y = int(math.floor((y_max - y_min + eps) / step)) + 1
        for iy in range(n_y):
            y = y_min + iy * step
            for ix in range(n_x):
                x = x_min + ix * step
                res = self.validate(x, y)
                rec: Dict[str, float] = {"x": x, "y": y, "ok": float(res.ok)}
                rec.update(res.metrics)
                records.append(rec)
        return records

    # --- geometry helpers ---------------------------------------------------
    def _parallel_margin(self, jt: JointTarget) -> float:
        # Vectors along each distal link, from the TCP back to the elbow.
        a = (jt.left_elbow[0] - jt.tcp[0], jt.left_elbow[1] - jt.tcp[1])
        b = (jt.right_elbow[0] - jt.tcp[0], jt.right_elbow[1] - jt.tcp[1])
        phi = _angle_between(a, b)
        # Collinear at phi == 0 (parallel) and phi == 180 (anti-parallel).
        return min(phi, 180.0 - phi)

    def _serial_margin(self, jt: JointTarget) -> float:
        cfg = self.config
        margins = []
        for base, elbow in (
            (cfg.left_base, jt.left_elbow),
            (cfg.right_base, jt.right_elbow),
        ):
            to_base = (base[0] - elbow[0], base[1] - elbow[1])
            to_tcp = (jt.tcp[0] - elbow[0], jt.tcp[1] - elbow[1])
            theta = _angle_between(to_base, to_tcp)  # interior elbow angle
            # Collinear (singular) at theta == 0 (folded) and 180 (straight).
            margins.append(min(theta, 180.0 - theta))
        return min(margins)


def _angle_between(u: Point, v: Point) -> float:
    """Angle between two vectors in degrees, in [0, 180]."""
    nu = math.hypot(*u)
    nv = math.hypot(*v)
    if nu == 0.0 or nv == 0.0:
        return 0.0
    cos = (u[0] * v[0] + u[1] * v[1]) / (nu * nv)
    cos = max(-1.0, min(1.0, cos))  # clamp float error
    return math.degrees(math.acos(cos))
