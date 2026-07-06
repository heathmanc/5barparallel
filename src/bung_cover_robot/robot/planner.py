"""Pick/place job planning (Claude.md §11, §14).

Turns validated robot-frame pick & drop points into a `PickPlaceJob` carrying the
solved shoulder angles for both poses, and orders the battery's holes along the
conveyor so the cycle fills them in a sensible sequence.

A job is only ever built from targets that pass `WorkspaceValidator` — an
unreachable/near-singular pick or drop raises `PlanningError` and no job is
created, so a bad target can never reach the PLC (Claude.md §15).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Sequence, Tuple

from .fivebar_kinematics import FiveBarKinematics, JointTarget
from .workspace import WorkspaceValidator

Point = Tuple[float, float]


class PlanningError(Exception):
    """A pick or drop target could not be turned into a valid job."""


@dataclass(frozen=True)
class PickPlaceJob:
    """A validated pick->place job: both poses solved to shoulder angles."""

    hole_index: int
    cover_id: int
    pick: JointTarget
    drop: JointTarget

    @property
    def pick_xy(self) -> Point:
        return self.pick.tcp

    @property
    def drop_xy(self) -> Point:
        return self.drop.tcp


def _solve(
    kin: FiveBarKinematics, validator: WorkspaceValidator, xy: Point, what: str
) -> JointTarget:
    res = validator.validate(xy[0], xy[1])
    if not res.ok:
        raise PlanningError(f"{what} ({xy[0]:.1f}, {xy[1]:.1f}) rejected: {res.reason}")
    return kin.inverse(xy[0], xy[1])


def make_job(
    kin: FiveBarKinematics,
    validator: WorkspaceValidator,
    *,
    hole_index: int,
    cover_id: int,
    pick_xy: Point,
    drop_xy: Point,
) -> PickPlaceJob:
    """Build a validated job. Raises PlanningError if pick or drop is not safe."""
    pick = _solve(kin, validator, pick_xy, "pick")
    drop = _solve(kin, validator, drop_xy, "drop")
    return PickPlaceJob(hole_index=hole_index, cover_id=cover_id, pick=pick, drop=drop)


def sort_holes_along_conveyor(holes_xy: Sequence[Point]) -> List[int]:
    """Indices of ``holes_xy`` ordered along the conveyor (robot X, then Y)."""
    return sorted(range(len(holes_xy)), key=lambda i: (holes_xy[i][0], holes_xy[i][1]))
