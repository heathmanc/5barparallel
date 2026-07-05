"""Robot driver abstraction — the boundary between control logic and the motion
hardware.

The jog/home logic (app.robot_test_controller) computes and *validates* shoulder
angles, then hands them to a RobotDriver to actuate. Today only DryRunRobotDriver
exists (simulates instantly, no hardware), so the GUI and tests run with nothing
connected. A PLC-backed driver comes later: jogging needs a small manual-move
surface on the PLC (a manual/jog mode plus MoveTo-angles + InPosition tags),
separate from the pick/place job handshake in Claude.md §11.

Contract: a driver never decides *where* to go — it only enables/disables the
axes and moves to angles it is given. All reachability/singularity gating happens
upstream in WorkspaceValidator (Claude.md §15).
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

Angles = Tuple[float, float]  # (left_deg, right_deg)


class RobotDriverError(Exception):
    """A driver command was rejected (e.g. move while disabled)."""


class RobotDriver(ABC):
    """Actuates the two shoulder axes. Angles are in degrees, robot frame."""

    @property
    @abstractmethod
    def is_enabled(self) -> bool:
        ...

    @abstractmethod
    def enable(self) -> None:
        """Energize/enable the drives. Motion is refused until enabled."""

    @abstractmethod
    def disable(self) -> None:
        """De-energize the drives."""

    @abstractmethod
    def move_to_angles(self, left_deg: float, right_deg: float) -> None:
        """Command an absolute move of both shoulders. Raises RobotDriverError
        if the drives are not enabled."""

    @abstractmethod
    def read_angles(self) -> Optional[Angles]:
        """Current shoulder angles, or None if position is not yet known
        (e.g. before homing)."""

    @abstractmethod
    def home(self) -> None:
        """Run the hardware homing routine (find the reference). On the real
        robot this is owned by the PLC (Claude.md §7)."""

    @abstractmethod
    def stop(self) -> None:
        """Abort any in-progress motion. Does not disable the drives."""


class DryRunRobotDriver(RobotDriver):
    """Simulated driver: moves are instantaneous and just recorded. Lets the
    whole robot-test flow run with no PLC/hardware (Claude.md §15 --dry-run)."""

    def __init__(self, home_angles: Optional[Angles] = None) -> None:
        self._enabled = False
        self._angles: Optional[Angles] = None  # unknown until homed/moved
        self._home_angles = home_angles
        self.command_log: List[Angles] = []

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        self._enabled = True
        logger.info("[dry-run] drives ENABLED")

    def disable(self) -> None:
        self._enabled = False
        logger.info("[dry-run] drives DISABLED")

    def move_to_angles(self, left_deg: float, right_deg: float) -> None:
        if not self._enabled:
            raise RobotDriverError("cannot move: drives are disabled")
        self._angles = (left_deg, right_deg)
        self.command_log.append(self._angles)
        logger.info("[dry-run] move -> L=%.3f deg R=%.3f deg", left_deg, right_deg)

    def read_angles(self) -> Optional[Angles]:
        return self._angles

    def home(self) -> None:
        if not self._enabled:
            raise RobotDriverError("cannot home: drives are disabled")
        self._angles = self._home_angles or (0.0, 0.0)
        logger.info("[dry-run] homed -> %s", self._angles)

    def stop(self) -> None:
        logger.info("[dry-run] STOP")
