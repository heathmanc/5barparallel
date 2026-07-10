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

from ..robot.driver import (
    DryRunRobotDriver,
    HomingConfig,
    RobotDriver,
    RobotDriverError,
)
from ..robot.fivebar_kinematics import FiveBarConfig, FiveBarKinematics
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
        self._referenced = False  # hardware home found (switches), from driver.home()

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
    def is_referenced(self) -> bool:
        """Referenced only if we ran a home AND the driver still reports it homed.
        The driver's is_referenced is live (PLC Status.Homed), so a disable/fault
        -- which clears the machine's home on an open-loop stepper -- drops this
        immediately and forces a re-home, instead of trusting a stale cache."""
        return self._referenced and self.driver.is_referenced

    @property
    def is_faulted(self) -> bool:
        """True if the PLC/drive has a latched fault (needs reset)."""
        return self.driver.is_faulted

    def fault_code(self) -> Optional[int]:
        """The active PLC fault code, or None when not faulted."""
        return self.driver.fault_code()

    # --- mode (Manual / Auto) ----------------------------------------------
    @property
    def is_auto_mode(self) -> bool:
        """True when the machine is in Auto mode (PLC scans the pick/place
        routine). The automatic cycle requires this."""
        return self.driver.is_auto_mode

    def set_auto_mode(self, on: bool) -> None:
        """Select Auto vs Manual mode on the machine."""
        self.driver.set_auto_mode(on)

    # --- driver ------------------------------------------------------------
    def set_driver(self, driver: RobotDriver) -> None:
        """Hot-swap the motion driver (e.g. connect/disconnect a real PLC).

        Disables and closes the old driver, then *reconciles* with the new one's
        actual state instead of blindly assuming disabled/un-homed. Real
        handshaking: if the PLC reports a valid reference (Status.Homed with live
        angles), adopt it; otherwise require homing. The PLC's own heartbeat
        watchdog guarantees the drives can't be enabled without the app talking,
        so a reference only survives here if the machine genuinely still holds it."""
        old = self.driver
        if old is not driver:
            try:
                if old.is_enabled:
                    old.disable()
            except Exception:  # best effort; the old link may already be gone
                pass
            closer = getattr(old, "close", None)
            if callable(closer):
                try:
                    closer()
                except Exception:
                    pass
        self.driver = driver
        self._referenced = False
        # Reconcile the reference from the PLC's reported state (None => not homed).
        try:
            angles = driver.read_angles()
        except Exception:  # a read hiccup just leaves us un-referenced (safe)
            angles = None
        if angles is not None:
            state, _ = self._state_from_angles(*angles)
            if state is not None:
                self._state = state
                self._referenced = True

    # --- enable / reset -----------------------------------------------------
    def enable(self) -> MoveResult:
        try:
            self.driver.enable()
        except RobotDriverError as exc:
            return self._reject(str(exc))
        return MoveResult(True, "ok", self._state)

    def disable(self) -> MoveResult:
        try:
            self.driver.disable()
        except RobotDriverError as exc:
            return self._reject(str(exc))
        return MoveResult(True, "ok", self._state)

    def stop(self) -> MoveResult:
        try:
            self.driver.stop()
        except RobotDriverError as exc:
            return self._reject(str(exc))
        return MoveResult(True, "ok", self._state)

    def reset(self) -> MoveResult:
        """Clear a latched PLC fault so the drives can be enabled/homed again."""
        try:
            self.driver.reset()
        except RobotDriverError as exc:
            return self._reject(str(exc))
        return MoveResult(True, "ok", self._state)

    # --- home ---------------------------------------------------------------
    def home_reference(self) -> MoveResult:
        """Run the hardware homing routine (find the home switches) and adopt the
        reference pose the driver reports. Required before jogging."""
        if not self.is_enabled:
            return self._reject("drives are disabled — enable first")
        try:
            self.driver.home()
            angles = self.driver.read_angles()
        except RobotDriverError as exc:
            self._referenced = False
            return self._reject(str(exc))
        if angles is not None:
            state, reason = self._state_from_angles(*angles)
            if state is None:
                return self._reject(f"reported home pose is invalid: {reason}")
            self._state = state
        self._referenced = True
        return MoveResult(True, "ok", self._state)

    def set_home(self) -> Point:
        """Teach: capture the current pose as the software home reference."""
        self._home_xy = self._state.tcp
        self.driver.set_home_angles((self._state.left_deg, self._state.right_deg))
        return self._home_xy

    def set_home_xy(self, x: float, y: float) -> MoveResult:
        """Set the software home to a specific TCP (x, y) mm and recompute its
        shoulder angles under the current geometry. Rejects an unreachable/unsafe
        home. Adopts the new home angles as the driver's reference too."""
        state, reason = self._state_from_xy(x, y)
        if state is None:
            return self._reject(f"home ({x:.1f}, {y:.1f}) invalid: {reason}")
        self._home_xy = (float(x), float(y))
        self.driver.set_home_angles((state.left_deg, state.right_deg))
        if not self._referenced:
            self._state = state
        return MoveResult(True, "ok", state)

    def go_home(self) -> MoveResult:
        """Drive the robot to the taught software home pose."""
        guard = self._motion_guard()
        if guard is not None:
            return guard
        return self._move_to_xy(*self._home_xy)

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
        if not self.is_referenced:
            return self._reject("robot is not referenced — Home (find ref) first")
        return None

    def _move_to_xy(self, x: float, y: float) -> MoveResult:
        state, res = self._state_from_xy(x, y)
        if state is None:
            return self._reject(res)
        return self._command(state)

    def _command(self, state: RobotState) -> MoveResult:
        try:
            self.driver.move_to_angles(state.left_deg, state.right_deg)
        except RobotDriverError as exc:
            return self._reject(str(exc))
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

    def update_geometry(
        self,
        config: FiveBarConfig,
        validator: Optional[WorkspaceValidator] = None,
        home_xy: Optional[Point] = None,
    ) -> None:
        """Swap in new geometry (used by the Settings tab after re-validation).

        Rebuilds kinematics + validator. If ``home_xy`` is given, adopt it as the
        new (validated) home and current pose — needed when a geometry change also
        moves the home (e.g. a smaller robot). Otherwise re-resolve the current
        pose, snapping back to the existing home if it's no longer valid.
        """
        self.kin = FiveBarKinematics(config)
        self.validator = validator or WorkspaceValidator(self.kin)
        if home_xy is not None:
            state, reason = self._state_from_xy(*home_xy)
            if state is None:
                raise ValueError(f"home {home_xy} invalid under new geometry: {reason}")
            self._home_xy = (float(home_xy[0]), float(home_xy[1]))
            self.driver.set_home_angles((state.left_deg, state.right_deg))
            self._state = state
            return
        state, _ = self._state_from_xy(*self._state.tcp)
        if state is None:
            state, _ = self._state_from_xy(*self._home_xy)
        if state is None:
            raise ValueError("home pose invalid under new geometry")
        self._state = state


def build_dry_run_controller(
    config: Optional[FiveBarConfig] = None,
    homing: Optional[HomingConfig] = None,
    home_xy: Optional[Point] = None,
) -> RobotTestController:
    """A controller backed by the simulated driver, using the configured home
    reference (config/robot_config.yaml `homing` block)."""
    kin = FiveBarKinematics(config) if config is not None else FiveBarKinematics()
    homing = homing or HomingConfig()
    driver = DryRunRobotDriver(home_angles=homing.home_angles)
    hx = home_xy if home_xy is not None else homing.home_tcp_mm
    return RobotTestController(driver, kin, home_xy=hx)
