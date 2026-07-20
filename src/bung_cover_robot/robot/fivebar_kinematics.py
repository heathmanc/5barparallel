"""5-bar parallel-SCARA inverse/forward kinematics.

Geometry and conventions are the *verified* design point from Claude.md §3-§5.
Read those sections before changing anything here.

Robot frame (what this module uses):
  - The two shoulder bases lie on the X axis (Y = 0), symmetric about the
    origin: left at (-spacing/2, 0), right at (+spacing/2, 0).
  - The mechanism reaches into +Y.
  - Angles are CCW from +X, in degrees.

Each arm is two links: proximal L1 (shoulder -> elbow) then distal L2
(elbow -> TCP). Both distal links meet at the shared tool center point (TCP).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

try:  # PyYAML is only needed for from_yaml(); keep the import soft.
    import yaml
except ImportError:  # pragma: no cover - exercised only without PyYAML
    yaml = None  # type: ignore[assignment]

Point = Tuple[float, float]

_VALID_BRANCHES = {"up", "down"}


class KinematicsError(ValueError):
    """Raised when a TCP lies outside an arm's reachable envelope."""


@dataclass(frozen=True)
class FiveBarConfig:
    """Immutable geometry / drivetrain / joint description.

    The dataclass defaults ARE the verified design point, so ``FiveBarConfig()``
    with no arguments is a valid, buildable robot. ``from_yaml`` overrides
    fields from ``config/robot_config.yaml``.
    """

    l1_mm: float = 200.0
    l2_mm: float = 230.0
    base_spacing_mm: float = 80.0
    left_elbow: str = "up"
    right_elbow: str = "down"
    joint_min_deg: float = -20.0
    joint_max_deg: float = 200.0
    # A6 servo drivetrain: 17-bit absolute encoder (131072 counts/rev) through a
    # 3:1 belt. In CSP the drive's position unit is one encoder count, so
    # ``pulses_per_degree`` (131072*3/360 = 1092.267) is the deg<->counts factor
    # the trajectory planner and EtherCAT driver both use.
    pulses_per_rev: int = 131072
    belt_reduction: float = 3.0

    def __post_init__(self) -> None:
        if self.left_elbow not in _VALID_BRANCHES:
            raise ValueError(f"left_elbow must be one of {_VALID_BRANCHES}")
        if self.right_elbow not in _VALID_BRANCHES:
            raise ValueError(f"right_elbow must be one of {_VALID_BRANCHES}")
        if self.l1_mm <= 0 or self.l2_mm <= 0:
            raise ValueError("link lengths must be positive")
        if self.joint_max_deg <= self.joint_min_deg:
            raise ValueError("joint_max_deg must exceed joint_min_deg")

    # --- derived geometry ---------------------------------------------------
    @property
    def max_reach_mm(self) -> float:
        """Fully-extended reach of a single arm (L1 + L2)."""
        return self.l1_mm + self.l2_mm

    @property
    def min_reach_mm(self) -> float:
        """Fully-folded reach of a single arm (|L1 - L2|)."""
        return abs(self.l1_mm - self.l2_mm)

    @property
    def pulses_per_degree(self) -> float:
        """Drive pulses per degree of shoulder rotation."""
        return self.pulses_per_rev * self.belt_reduction / 360.0

    @property
    def left_base(self) -> Point:
        return (-self.base_spacing_mm / 2.0, 0.0)

    @property
    def right_base(self) -> Point:
        return (self.base_spacing_mm / 2.0, 0.0)

    # --- loading ------------------------------------------------------------
    @classmethod
    def from_yaml(cls, path: str | Path) -> "FiveBarConfig":
        """Load geometry from a robot_config.yaml file.

        Missing sections/keys fall back to the verified defaults above, so a
        partial YAML is fine.
        """
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required for FiveBarConfig.from_yaml")
        data = yaml.safe_load(Path(path).read_text()) or {}
        geo = data.get("geometry", {})
        asm = data.get("assembly", {})
        lim = data.get("joint_limits", {})
        drv = data.get("drivetrain", {})
        base = cls()  # defaults
        return cls(
            l1_mm=float(geo.get("l1_mm", base.l1_mm)),
            l2_mm=float(geo.get("l2_mm", base.l2_mm)),
            base_spacing_mm=float(geo.get("base_spacing_mm", base.base_spacing_mm)),
            left_elbow=str(asm.get("left_elbow", base.left_elbow)),
            right_elbow=str(asm.get("right_elbow", base.right_elbow)),
            joint_min_deg=float(lim.get("min_deg", base.joint_min_deg)),
            joint_max_deg=float(lim.get("max_deg", base.joint_max_deg)),
            pulses_per_rev=int(drv.get("pulses_per_rev", base.pulses_per_rev)),
            belt_reduction=float(drv.get("belt_reduction", base.belt_reduction)),
        )


@dataclass(frozen=True)
class JointTarget:
    """Result of an inverse solve for a single TCP position."""

    left_deg: float
    right_deg: float
    left_pulses: int
    right_pulses: int
    tcp: Point
    left_elbow: Point
    right_elbow: Point


class FiveBarKinematics:
    """Inverse/forward kinematics for the 5-bar linkage."""

    def __init__(self, config: FiveBarConfig | None = None) -> None:
        self.config = config or FiveBarConfig()
        # up branch -> +normal, down branch -> -normal (see _solve_elbow).
        self._left_sign = 1.0 if self.config.left_elbow == "up" else -1.0
        self._right_sign = 1.0 if self.config.right_elbow == "up" else -1.0

    # --- public API ---------------------------------------------------------
    def inverse(self, x: float, y: float) -> JointTarget:
        """Solve shoulder angles for a TCP at (x, y) in the robot frame.

        Raises KinematicsError if either arm cannot reach the point.
        """
        cfg = self.config
        left_elbow = self._solve_elbow(cfg.left_base, (x, y), self._left_sign, "left")
        right_elbow = self._solve_elbow(cfg.right_base, (x, y), self._right_sign, "right")
        left_deg = self._shoulder_angle(cfg.left_base, left_elbow)
        right_deg = self._shoulder_angle(cfg.right_base, right_elbow)
        return JointTarget(
            left_deg=left_deg,
            right_deg=right_deg,
            left_pulses=self.degrees_to_pulses(left_deg),
            right_pulses=self.degrees_to_pulses(right_deg),
            tcp=(x, y),
            left_elbow=left_elbow,
            right_elbow=right_elbow,
        )

    def forward(self, left_deg: float, right_deg: float) -> Point:
        """Recover the TCP from the two shoulder angles.

        Picks the +Y (reaching-side) circle-circle intersection, which is the
        correct assembly for this robot's work zone. For round-trip checks and
        diagnostics.

        DOMAIN: this is the true inverse of ``inverse()`` only for poses whose
        TCP is the upper (+Y) distal intersection — i.e. the actual +Y work
        zone. For angles that place the TCP below the shoulder line it returns
        the mirrored (upper) point, not the physical TCP. WorkspaceValidator
        uses exactly this to reject wrong-assembly targets, so don't "fix" it to
        return the lower point without updating that guard.
        """
        cfg = self.config
        left_elbow = self._elbow_from_angle(cfg.left_base, left_deg)
        right_elbow = self._elbow_from_angle(cfg.right_base, right_deg)
        return self._tcp_from_elbows(left_elbow, right_elbow)

    def is_reachable(self, x: float, y: float) -> bool:
        """Geometric envelope check only.

        Does NOT consider joint limits or singularities. Use
        WorkspaceValidator.validate() for the real go/no-go.
        """
        cfg = self.config
        for base in (cfg.left_base, cfg.right_base):
            d = math.hypot(x - base[0], y - base[1])
            if d > cfg.max_reach_mm or d < cfg.min_reach_mm:
                return False
        return True

    def degrees_to_pulses(self, deg: float) -> int:
        return round(deg * self.config.pulses_per_degree)

    def within_joint_limits(self, deg: float) -> bool:
        deg = self._normalize_into_window(deg)
        return self.config.joint_min_deg <= deg <= self.config.joint_max_deg

    # --- internals ----------------------------------------------------------
    def _solve_elbow(self, base: Point, tcp: Point, sign: float, which: str) -> Point:
        """Circle-circle intersection: elbow is L1 from base and L2 from TCP.

        ``sign`` selects the branch: +1 places the elbow on the CCW side of the
        base->TCP line ("up"), -1 on the CW side ("down").
        """
        cfg = self.config
        bx, by = base
        px, py = tcp
        dx, dy = px - bx, py - by
        d = math.hypot(dx, dy)
        if d > cfg.max_reach_mm or d < cfg.min_reach_mm or d == 0.0:
            raise KinematicsError(
                f"{which} arm cannot reach ({px:.1f}, {py:.1f}): "
                f"distance {d:.1f} mm outside [{cfg.min_reach_mm:.1f}, "
                f"{cfg.max_reach_mm:.1f}]"
            )
        # distance from base to the foot of the elbow's perpendicular
        a = (cfg.l1_mm**2 - cfg.l2_mm**2 + d**2) / (2.0 * d)
        h_sq = cfg.l1_mm**2 - a**2
        h = math.sqrt(max(h_sq, 0.0))  # clamp tiny negatives from float error
        ux, uy = dx / d, dy / d  # unit base->TCP
        nx, ny = -uy, ux  # unit normal, 90deg CCW from base->TCP
        mx, my = bx + a * ux, by + a * uy  # foot point
        return (mx + sign * h * nx, my + sign * h * ny)

    def _elbow_from_angle(self, base: Point, deg: float) -> Point:
        rad = math.radians(deg)
        return (
            base[0] + self.config.l1_mm * math.cos(rad),
            base[1] + self.config.l1_mm * math.sin(rad),
        )

    def _tcp_from_elbows(self, left_elbow: Point, right_elbow: Point) -> Point:
        """Intersect the two distal circles (radius L2), take the +Y solution."""
        l2 = self.config.l2_mm
        ex1, ey1 = left_elbow
        ex2, ey2 = right_elbow
        dx, dy = ex2 - ex1, ey2 - ey1
        d = math.hypot(dx, dy)
        if d > 2 * l2 or d == 0.0:
            raise KinematicsError(
                f"elbows {left_elbow} / {right_elbow} admit no TCP (separation "
                f"{d:.1f} mm)"
            )
        a = d / 2.0  # symmetric: both radii equal L2
        h = math.sqrt(max(l2**2 - a**2, 0.0))
        ux, uy = dx / d, dy / d
        nx, ny = -uy, ux
        mx, my = ex1 + a * ux, ey1 + a * uy
        cand1 = (mx + h * nx, my + h * ny)
        cand2 = (mx - h * nx, my - h * ny)
        return cand1 if cand1[1] >= cand2[1] else cand2

    def _shoulder_angle(self, base: Point, elbow: Point) -> float:
        deg = math.degrees(math.atan2(elbow[1] - base[1], elbow[0] - base[0]))
        return self._normalize_into_window(deg)

    def _normalize_into_window(self, deg: float) -> float:
        """Fold an angle into [joint_min, joint_min + 360).

        atan2 returns (-180, 180]; a real 185deg pose reads as -175. Normalizing
        into the joint's 360deg window prevents that from being mis-rejected.
        """
        lo = self.config.joint_min_deg
        span = 360.0
        while deg < lo:
            deg += span
        while deg >= lo + span:
            deg -= span
        return deg
