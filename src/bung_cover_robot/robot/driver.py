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
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

Angles = Tuple[float, float]  # (left_deg, right_deg)


class RobotDriverError(Exception):
    """A driver command was rejected (e.g. move while disabled)."""


@dataclass(frozen=True)
class HomingConfig:
    """Home/limit switch reference (config/robot_config.yaml `homing` block).

    Defaults are the verified reference (see docs/homing.md): mid-travel centre
    home, flag on the proximal link L1 at r=40 mm, hard limits at -20/+200 deg.
    The PLC homing routine and the GUI's "Home (find ref)" read these instead of
    hard-coding them.
    """

    home_left_deg: float = 140.5406
    home_right_deg: float = 39.4594
    home_tcp_mm: Tuple[float, float] = (0.0, 250.0)
    flag_radius_mm: float = 40.0
    limit_min_deg: float = -20.0
    limit_max_deg: float = 200.0

    @property
    def home_angles(self) -> Angles:
        return (self.home_left_deg, self.home_right_deg)

    @classmethod
    def from_yaml(cls, path: str | Path) -> "HomingConfig":
        import yaml

        data = yaml.safe_load(Path(path).read_text()) or {}
        sec = data.get("homing", {})
        base = cls()
        tcp = sec.get("home_tcp_mm", list(base.home_tcp_mm))
        return cls(
            home_left_deg=float(sec.get("home_left_deg", base.home_left_deg)),
            home_right_deg=float(sec.get("home_right_deg", base.home_right_deg)),
            home_tcp_mm=(float(tcp[0]), float(tcp[1])),
            flag_radius_mm=float(sec.get("flag_radius_mm", base.flag_radius_mm)),
            limit_min_deg=float(sec.get("limit_min_deg", base.limit_min_deg)),
            limit_max_deg=float(sec.get("limit_max_deg", base.limit_max_deg)),
        )


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

    @abstractmethod
    def reset(self) -> None:
        """Clear a latched fault so the drives can be enabled/homed again."""

    def set_auto_mode(self, on: bool) -> None:
        """Select Auto (automatic pick/place owns motion) vs Manual (jog/home)
        mode. No-op for drivers with no mode concept."""

    @property
    def is_auto_mode(self) -> bool:
        """True when the machine is in Auto mode. Default False (manual)."""
        return False

    @property
    def is_referenced(self) -> bool:
        """True if a home reference is currently established. Default: we know a
        position. The PLC driver overrides to read Status.Homed live, so a
        disable/fault (which clears the PLC's home) is reflected immediately -- an
        open-loop stepper loses its datum on disable and must be re-homed."""
        return self.read_angles() is not None

    @property
    def is_faulted(self) -> bool:
        """True if a fault is latched. Default False for drivers with no fault
        concept (overridden by the PLC driver)."""
        return False

    def fault_code(self) -> Optional[int]:
        """The active fault code, or None when not faulted / not applicable."""
        return None


class DryRunRobotDriver(RobotDriver):
    """Simulated driver: moves are instantaneous and just recorded. Lets the
    whole robot-test flow run with no PLC/hardware (Claude.md §15 --dry-run)."""

    def __init__(self, home_angles: Optional[Angles] = None) -> None:
        self._enabled = False
        self._angles: Optional[Angles] = None  # unknown until homed/moved
        self._home_angles = home_angles
        self._auto = False
        self.command_log: List[Angles] = []

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def set_auto_mode(self, on: bool) -> None:
        self._auto = bool(on)
        logger.info("[dry-run] mode = %s", "AUTO" if on else "MANUAL")

    @property
    def is_auto_mode(self) -> bool:
        return self._auto

    def enable(self) -> None:
        self._enabled = True
        logger.info("[dry-run] drives ENABLED")

    def disable(self) -> None:
        self._enabled = False
        # Disable loses the reference (open-loop: no feedback once de-energized).
        self._angles = None
        logger.info("[dry-run] drives DISABLED (reference cleared)")

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

    def reset(self) -> None:
        logger.info("[dry-run] reset (no fault to clear)")
