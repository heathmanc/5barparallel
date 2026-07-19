"""EtherCAT RobotDriver — the PC drives the two A6 servos in CSP.

Implements the same ``RobotDriver`` seam the rest of the app already uses, so
``CycleManager``, the controllers, and the GUI are unchanged. The difference from
the old PLC driver is that motion is planned and streamed *here*:

  * ``enable()``  — walk both drives up the CiA 402 state machine to Operation
    Enabled (one transition per PDO cycle).
  * ``home()``    — CiA 402 homing mode: the drive finds its switch; its absolute
    encoder then holds the datum even across a disable (unlike the old open-loop
    steppers), so a disable no longer forces a re-home.
  * ``move_to_angles()`` — plan a validated straight-line Cartesian move to the
    pose those angles imply, then stream the per-cycle CSP setpoints. Nothing
    unvalidated ever reaches the drives (every path point is workspace-checked at
    plan time).

The streaming loop (``_stream``) is what becomes the SCHED_FIFO real-time thread
on real hardware; against the simulator it just runs synchronously.
"""

from __future__ import annotations

import dataclasses
import logging
from typing import List, Optional

from ..robot.driver import Angles, RobotDriver, RobotDriverError
from ..robot.fivebar_kinematics import FiveBarKinematics
from ..robot.workspace import WorkspaceValidator
from . import cia402
from .master import EtherCatMaster, MasterError
from .trajectory import (
    TrajectoryError,
    TrajectoryLimits,
    plan_linear_move,
    ramp_counts,
    ramp_counts_multi,
)

logger = logging.getLogger(__name__)


class EtherCatRobotDriver(RobotDriver):
    def __init__(
        self,
        master: EtherCatMaster,
        kinematics: Optional[FiveBarKinematics] = None,
        validator: Optional[WorkspaceValidator] = None,
        home_angles: Angles = (140.5406, 39.4594),
        limits: Optional[TrajectoryLimits] = None,
        max_transition_cycles: int = 200,
        position_tol_counts: int = 5,
    ) -> None:
        self.master = master
        self.kin = kinematics or FiveBarKinematics()
        self.validator = validator or WorkspaceValidator(self.kin)
        self._home_angles: Angles = (float(home_angles[0]), float(home_angles[1]))
        self.limits = limits or TrajectoryLimits(cycle_dt_s=master.cycle_dt_s)
        self.max_transition_cycles = max_transition_cycles
        self.position_tol_counts = position_tol_counts
        self._referenced = False
        # Absolute drive counts at each home shoulder angle (drive zero == home).
        self._home_counts = self._counts(self._home_angles)

    # --- lifecycle ----------------------------------------------------------
    def connect(self) -> "EtherCatRobotDriver":
        if not self.master.is_open:
            self.master.open()
        return self

    def close(self) -> None:
        try:
            self.disable()
        finally:
            self.master.close()

    # --- conversions --------------------------------------------------------
    @property
    def _ppd(self) -> float:
        return self.kin.config.pulses_per_degree

    def _counts(self, angles: Angles) -> List[int]:
        return [round(angles[0] * self._ppd), round(angles[1] * self._ppd)]

    def _drive_target(self, abs_angle_deg: float, drive: int) -> int:
        """Absolute shoulder angle -> drive counts (relative to the home datum)."""
        return round(abs_angle_deg * self._ppd) - self._home_counts[drive]

    def _angle_from_actual(self, drive: int) -> float:
        counts = self.master.drives[drive].actual_position + self._home_counts[drive]
        return counts / self._ppd

    # --- status -------------------------------------------------------------
    @property
    def is_enabled(self) -> bool:
        return all(
            cia402.is_operation_enabled(d.statusword) for d in self.master.drives
        )

    @property
    def is_referenced(self) -> bool:
        # A6 absolute encoders hold the datum across a disable, so once homed we
        # stay referenced (no open-loop re-home) unless a fault clears it.
        return self._referenced and not self.is_faulted

    @property
    def is_faulted(self) -> bool:
        return any(cia402.is_fault(d.statusword) for d in self.master.drives)

    def fault_code(self) -> Optional[int]:
        for i, d in enumerate(self.master.drives):
            if cia402.is_fault(d.statusword):
                return i + 1        # which drive faulted (1-based); real code is SDO 0x603F
        return None

    # --- enable / reset -----------------------------------------------------
    def enable(self) -> None:
        if self.is_faulted:
            raise RobotDriverError("drives are faulted; reset before enabling")
        self.master.exchange()                        # fresh actual positions
        for d in self.master.drives:
            d.mode_of_operation = cia402.MODE_CSP
            d.target_position = d.actual_position      # hold on enable — no jump
        if not self._run_state_machine(target_enabled=True):
            raise RobotDriverError("drives did not reach Operation Enabled")

    def jog_counts(self, drive: int, delta_counts: int,
                   speed_counts_s: float = 20000.0,
                   accel_counts_s2: float = 100000.0) -> None:
        """Single-axis bench jog: ramp ``drive`` by ``delta_counts`` (raw drive
        counts), other drives holding. Requires Operation Enabled; the move is
        CSP-streamed through a trapezoidal ramp so it's smooth (a step would fault
        on following error). Motion — only with the E-stop/contactor live."""
        if self.is_faulted:
            raise RobotDriverError(f"cannot jog: faulted (code {self.fault_code()})")
        if not self.is_enabled:
            raise RobotDriverError("cannot jog: enable the drive first")
        if not 0 <= drive < len(self.master.drives):
            raise RobotDriverError(f"no such drive {drive}")
        self.master.exchange()                        # fresh actuals
        holds = [d.actual_position for d in self.master.drives]
        ramp = ramp_counts(holds[drive], int(delta_counts),
                           speed_counts_s, accel_counts_s2, self.limits.cycle_dt_s)
        targets = [tuple(c if i == drive else holds[i]
                         for i in range(len(holds))) for c in ramp]
        try:
            self.master.run_csp(targets)
        except MasterError as exc:
            raise RobotDriverError(f"jog aborted: {exc}") from exc

    def jog_counts_multi(self, deltas: List[int],
                         speed_counts_s: float = 20000.0,
                         accel_counts_s2: float = 100000.0) -> None:
        """Coordinated bench move: ramp every axis simultaneously through one
        shared time profile so they start and finish together — the first real
        test of two drives streaming in lockstep off one CSP stream. Joint-space
        (no kinematics), so it is safe before the arm is in the linkage. Motion —
        only with the E-stop/contactor live. Requires Operation Enabled."""
        if self.is_faulted:
            raise RobotDriverError(f"cannot move: faulted (code {self.fault_code()})")
        if not self.is_enabled:
            raise RobotDriverError("cannot move: enable the drives first")
        n = len(self.master.drives)
        if len(deltas) != n:
            raise RobotDriverError(f"expected {n} axis delta(s), got {len(deltas)}")
        self.master.exchange()                        # fresh actuals
        starts = [d.actual_position for d in self.master.drives]
        ramp = ramp_counts_multi(starts, [int(x) for x in deltas],
                                 speed_counts_s, accel_counts_s2, self.limits.cycle_dt_s)
        try:
            self.master.run_csp(ramp)
        except MasterError as exc:
            raise RobotDriverError(f"coordinated move aborted: {exc}") from exc

    def disable(self) -> None:
        for d in self.master.drives:
            d.controlword = cia402.CW_DISABLE_VOLTAGE
        try:
            self.master.exchange()
        except Exception:  # pragma: no cover - best effort on shutdown
            pass

    def reset(self) -> None:
        # Walk the fault-reset edge until the fault clears (or time out).
        for _ in range(self.max_transition_cycles):
            if not self.is_faulted:
                return
            for d in self.master.drives:
                d.controlword = cia402.next_controlword(d.statusword, d.controlword)
            self.master.exchange()
        raise RobotDriverError("fault did not clear on reset")

    def _run_state_machine(self, target_enabled: bool) -> bool:
        for _ in range(self.max_transition_cycles):
            for d in self.master.drives:
                if cia402.is_fault(d.statusword):
                    return False
                d.controlword = cia402.next_controlword(d.statusword, d.controlword)
            self.master.exchange()
            if target_enabled and self.is_enabled:
                return True
        return self.is_enabled if target_enabled else True

    # --- home ---------------------------------------------------------------
    def home(self) -> None:
        if not self.is_enabled:
            raise RobotDriverError("cannot home: drives are disabled")
        for d in self.master.drives:
            d.mode_of_operation = cia402.MODE_HOMING
            d.controlword = cia402.CW_HOMING_START     # enable operation + start homing
        homed = False
        for _ in range(self.max_transition_cycles):
            self.master.exchange()
            if all(d.mode_display == cia402.MODE_HOMING for d in self.master.drives):
                homed = True
                break
        # Back to CSP for streamed motion.
        for d in self.master.drives:
            d.mode_of_operation = cia402.MODE_CSP
            d.target_position = d.actual_position       # hold where we are
        self.master.exchange()
        if not homed:
            raise RobotDriverError("homing did not complete")
        self._referenced = True
        logger.info("homed -> reference %s", self._home_angles)

    def set_home_angles(self, angles: Angles) -> None:
        self._home_angles = (float(angles[0]), float(angles[1]))
        self._home_counts = self._counts(self._home_angles)

    def set_home(self) -> None:
        """Set home at the CURRENT pose. This machine homes to hard mechanical
        stops (no home switches): drive each axis to its hard stop, then Set Home
        to declare that pose the datum (= the configured ``home_angles``). All
        subsequent Cartesian moves are referenced to it.

        The configured ``home_angles`` must match the physical hard-stop pose,
        or Cartesian moves will be offset. With a single-turn absolute encoder
        the count is only known modulo a rev, so re-home after each power cycle."""
        self.master.exchange()
        drives = self.master.drives
        self._home_counts = [
            round(self._home_angles[i] * self._ppd) - drives[i].actual_position
            for i in range(min(len(drives), len(self._home_angles)))
        ]
        self._referenced = True
        logger.info("home set at current pose -> %s", self._home_angles)

    # --- motion -------------------------------------------------------------
    def move_to_angles(self, left_deg: float, right_deg: float) -> None:
        if self.is_faulted:
            raise RobotDriverError(
                f"cannot move: drives are faulted (code {self.fault_code()})")
        if not self.is_enabled:
            raise RobotDriverError("cannot move: drives are disabled")
        cur = self.read_angles()
        if cur is None:
            raise RobotDriverError("cannot move: robot is not referenced")
        start_xy = self.kin.forward(*cur)
        goal_xy = self.kin.forward(left_deg, right_deg)
        try:
            traj = plan_linear_move(self.kin, self.validator, start_xy, goal_xy, self.limits)
        except TrajectoryError as exc:
            raise RobotDriverError(f"move planning failed: {exc}") from exc
        self._stream(traj)
        logger.info("moved -> L=%.3f R=%.3f (%d cycles)", left_deg, right_deg, len(traj))

    def jog_cartesian(self, dx_mm: float, dy_mm: float,
                      speed_mm_s: Optional[float] = None) -> None:
        """HMI/bench jog: move the TCP by (dx, dy) mm in a validated straight
        line. Requires Operation Enabled and a referenced robot. Motion — only
        with the E-stop/contactor live."""
        if self.is_faulted:
            raise RobotDriverError(f"cannot jog: faulted (code {self.fault_code()})")
        if not self.is_enabled:
            raise RobotDriverError("cannot jog: enable the drives first")
        cur = self.read_angles()
        if cur is None:
            raise RobotDriverError("cannot jog: robot is not referenced — home it "
                                   "(or use 'Reference here' on the bench) first")
        sx, sy = self.kin.forward(*cur)
        goal = (sx + float(dx_mm), sy + float(dy_mm))
        limits = self.limits
        if speed_mm_s and speed_mm_s > 0:
            limits = dataclasses.replace(limits, speed_mm_s=float(speed_mm_s))
        try:
            traj = plan_linear_move(self.kin, self.validator, (sx, sy), goal, limits)
        except TrajectoryError as exc:
            raise RobotDriverError(f"cartesian jog rejected: {exc}") from exc
        self._stream(traj)

    def _stream(self, traj) -> None:
        """Hand the precomputed per-cycle CSP setpoints to the master to stream.

        The master owns the timing: the simulator plays them out synchronously,
        the real master's SCHED_FIFO thread plays one per DC cycle. We converted
        the whole plan to drive counts up front, so the real-time loop does zero
        kinematics and zero allocation."""
        h0, h1 = self._home_counts
        targets = [(sp.left_counts - h0, sp.right_counts - h1) for sp in traj.setpoints]
        try:
            self.master.run_csp(targets)
        except MasterError as exc:
            raise RobotDriverError(f"move aborted: {exc}") from exc
        want = targets[-1]
        got = (self.master.drives[0].actual_position,
               self.master.drives[1].actual_position)
        if (abs(got[0] - want[0]) > self.position_tol_counts
                or abs(got[1] - want[1]) > self.position_tol_counts):
            raise RobotDriverError(
                f"move did not reach target (following error: want {want}, got {got})")

    def read_angles(self) -> Optional[Angles]:
        if not self._referenced:
            return None
        return (self._angle_from_actual(0), self._angle_from_actual(1))

    def stop(self) -> None:
        # Hold position: command the current actual as the target.
        for d in self.master.drives:
            d.target_position = d.actual_position
        try:
            self.master.exchange()
        except Exception:  # pragma: no cover
            pass
