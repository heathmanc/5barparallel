"""Home-offset calibration: solve ``HOME_OFFSET_L/R`` from a known TCP pose.

At a physically known robot-frame point the true shoulder angles are fixed by
the kinematics. Comparing them to the ClearLink's post-home ``CommandedPosn``
(steps, zeroed at the prox trip point) gives the offset that makes the PLC's
``Status.ActualDeg`` report the true angle:

    ActualDeg = (CommandedPosn + HOME_OFFSET) / STEPS_PER_DEG  ==  theta
      =>  HOME_OFFSET = round(theta * STEPS_PER_DEG - CommandedPosn)

See ``docs/home_offset_calibration.md`` for the field procedure.
"""

from __future__ import annotations

from dataclasses import dataclass

from .fivebar_kinematics import FiveBarKinematics


@dataclass(frozen=True)
class HomeOffsetSolution:
    """The computed offsets plus the intermediates, for display/audit."""

    x: float
    y: float
    theta_left_deg: float
    theta_right_deg: float
    posn_left: int
    posn_right: int
    offset_left: int
    offset_right: int


def solve_home_offsets(
    kin: FiveBarKinematics,
    x: float,
    y: float,
    posn_left: int,
    posn_right: int,
) -> HomeOffsetSolution:
    """Solve both home offsets from a known TCP ``(x, y)`` and the live
    ClearLink commanded positions read at that pose.

    Raises ``KinematicsError`` if ``(x, y)`` is unreachable (a bad jig
    coordinate), so the caller can surface it instead of writing garbage.
    """
    jt = kin.inverse(x, y)
    spd = kin.config.pulses_per_degree
    return HomeOffsetSolution(
        x=x,
        y=y,
        theta_left_deg=jt.left_deg,
        theta_right_deg=jt.right_deg,
        posn_left=int(posn_left),
        posn_right=int(posn_right),
        offset_left=round(jt.left_deg * spd - posn_left),
        offset_right=round(jt.right_deg * spd - posn_right),
    )
