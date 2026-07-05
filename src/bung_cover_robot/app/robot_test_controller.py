"""Headless logic for the Robot Test tab: establish home + jog the robot.

This carries no GUI dependency so it is fully unit-testable. The Qt tab
(gui.robot_test_tab) is a thin view over it.

Every commanded pose is gated by WorkspaceValidator before it reaches the driver
(Claude.md §15): a jog that would leave the clean workspace, cross a singularity,
or over-extend the arm is rejected and the robot does not move.

Home model (both teach + go-home):
  * set_home()  — capture the current pose as the software home reference.
  * go_home()   — drive the robot back to that reference.
Jog is measured from wherever the robot currently is.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from ..robot.driver import RobotDriver
from ..robot.fivebar_kinematics import FiveBarKinematics
from ..robot.workspace import WorkspaceValidator

Point = Tuple[float, float]

# Robot-frame default home: centered in the work zone at Y = 250 mm, a
# well-conditioned pose (Claude.md §4). Teach a new one at runtime with set_home.
DEFAULT_HOME_XY: Point = (0.0, 250.0)

JOINTS = ("left", "right")
AXES = ("x", "y")


@dataclass(frozen=True)
class RobotState:
    """A fully-resolved, validated pose."""

    left_deg: float
    right_deg: float
    tcp: Point
    left_pulses: int
    right_pulses: int
    metrics: Dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class MoveResult:
    """Outcome of a home/jog request. ``state`` is the resulting pose on success,
    or the unchanged current pose on rejection."""

    ok: bool
    reason: str
    state: RobotState

    def __bool__(self) -> bool:
        return self.ok


class RobotTestController:
    def __init__(
        self,
        driver: RobotDriver,
        kinematics: Optional[FiveBarKinematics] = None,
        validator: Optional[WorkspaceValidator] = None,
        home_xy: Point = DEFAULT_HOME_XY,
    ) -> None:
        self.kin = kinematics or FiveBarKinematics()
        self.validator = validator or WorkspaceValidator(self.kin)
        self.driver = driver
        self._home_xy: Point = home_xy
        state, res = self._state_from_xy(*home_xy)
        if state is None:
            raise ValueError(f"home {home_xy} is not a valid pose: {res}")
        self._state: RobotState = state
        self._homed = False

    # --- status -------------------------------------------------------------
    @property
    def state(self) -> RobotState:
        return self._state

    @property
    def home_xy(self) -> Point:
        return self._home_xy

    @property
    def is_enabled(self) -> bool:
        return self.driver.is_enabled

    @property
    def is_homed(self) -> bool:
        return self._homed

    # --- enable -------------------------------------------------------------
    def enable(self) -> None:
        self.driver.enable()

    def disable(self) -> None:
        self.driver.disable()

    def stop(self) -> None:
        self.driver.stop()

    # --- home ---------------------------------------------------------------
    def set_home(self) -> Point:
        """Teach: capture the current pose as the home reference."""
        self._home_xy = self._state.tcp
        return self._home_xy

    def go_home(self) -> MoveResult:
        """Drive the robot to the taught home reference."""
        if not self.is_enabled:
            return self._reject("drives are disabled — enable first")
        result = self._move_to_xy(*self._home_xy)
        if result.ok:
            self._homed = True
        return result

    # --- jog ----------------------------------------------------------------
    def jog_joint(self, joint: str, delta_deg: float) -> MoveResult:
        """Jog one shoulder by delta_deg (robot frame)."""
        if joint not in JOINTS:
            raise ValueError(f"joint must be one of {JOINTS}, got {joint!r}")
        guard = self._motion_guard()
        if guard is not None:
            return guard

        left = self._state.left_deg + (delta_deg if joint == "left" else 0.0)
        right = self._state.right_deg + (delta_deg if joint == "right" else 0.0)
        if not self.kin.within_joint_limits(left):
            return self._reject(f"left shoulder {left:.1f} deg exceeds joint limits")
        if not self.kin.within_joint_limits(right):
            return self._reject(f"right shoulder {right:.1f} deg exceeds joint limits")

        state, res = self._state_from_angles(left, right)
        if state is None:
            return self._reject(res)
        return self._command(state)

    def jog_cartesian(self, axis: str, delta_mm: float) -> MoveResult:
        """Jog the TCP along a robot-frame axis (x = along conveyor,
        y = across / reach) by delta_mm."""
        if axis not in AXES:
            raise ValueError(f"axis must be one of {AXES}, got {axis!r}")
        guard = self._motion_guard()
        if guard is not None:
            return guard

        x, y = self._state.tcp
        if axis == "x":
            x += delta_mm
        else:
            y += delta_mm
        return self._move_to_xy(x, y)

    def move_to_xy(self, x: float, y: float) -> MoveResult:
        """Absolute Cartesian move of the TCP (validated)."""
        guard = self._motion_guard()
        if guard is not None:
            return guard
        return self._move_to_xy(x, y)

    # --- internals ----------------------------------------------------------
    def _motion_guard(self) -> Optional[MoveResult]:
        if not self.is_enabled:
            return self._reject("drives are disabled — enable first")
        if not self._homed:
            return self._reject("robot is not homed — Go Home first")
        return None

    def _move_to_xy(self, x: float, y: float) -> MoveResult:
        state, res = self._state_from_xy(x, y)
        if state is None:
            return self._reject(res)
        return self._command(state)

    def _command(self, state: RobotState) -> MoveResult:
        self.driver.move_to_angles(state.left_deg, state.right_deg)
        self._state = state
        return MoveResult(True, "ok", state)

    def _reject(self, reason: str) -> MoveResult:
        return MoveResult(False, reason, self._state)

    def _state_from_xy(self, x: float, y: float) -> Tuple[Optional[RobotState], str]:
        res = self.validator.validate(x, y)
        if not res.ok:
            return None, res.reason
        jt = self.kin.inverse(x, y)
        return (
            RobotState(
                left_deg=jt.left_deg,
                right_deg=jt.right_deg,
                tcp=(x, y),
                left_pulses=jt.left_pulses,
                right_pulses=jt.right_pulses,
                metrics=res.metrics,
            ),
            "ok",
        )

    def _state_from_angles(
        self, left_deg: float, right_deg: float
    ) -> Tuple[Optional[RobotState], str]:
        x, y = self.kin.forward(left_deg, right_deg)
        res = self.validator.validate(x, y)
        if not res.ok:
            return None, res.reason
        return (
            RobotState(
                left_deg=left_deg,
                right_deg=right_deg,
                tcp=(x, y),
                left_pulses=self.kin.degrees_to_pulses(left_deg),
                right_pulses=self.kin.degrees_to_pulses(right_deg),
                metrics=res.metrics,
            ),
            "ok",
        )
