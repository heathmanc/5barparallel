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
import time
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
    setpoint_velocities,
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
        position_tol_counts: int = 500,
        settle_timeout_s: float = 2.0,
        velocity_ff_scale: float = 1.0,
        vacuum_do_bit: int = 0,
        plunger_do_bit: int = 1,
        tooling_drive: int = 0,
    ) -> None:
        self.master = master
        self.kin = kinematics or FiveBarKinematics()
        self.validator = validator or WorkspaceValidator(self.kin)
        self._home_angles: Angles = (float(home_angles[0]), float(home_angles[1]))
        self.limits = limits or TrajectoryLimits(cycle_dt_s=master.cycle_dt_s)
        self.max_transition_cycles = max_transition_cycles
        # End-of-move window: 500 counts ~ 0.46 deg at the joint (17-bit
        # encoder x 3:1). A real servo needs a real tolerance AND time for its
        # integrator to pull in — both are Drives-tab motion parameters.
        self.position_tol_counts = position_tol_counts
        self.settle_timeout_s = settle_timeout_s
        # Per-cycle velocity feedforward streamed as 0x60B1 (counts/s); the drive
        # uses it only with speed-FF source = Communication (C01.13=5). 0 = don't
        # stream it. A bench-trim scale for the drive's velocity-offset units.
        self.velocity_ff_scale = velocity_ff_scale
        # Pick-head I/O: which drive carries the tooling digital outputs and which
        # bit of its 0x60FE:1 word drives the vacuum solenoid / the air cylinder.
        # Repointable to a dedicated EtherCAT I/O slice later; on the bus today the
        # A6 drives' spare DOs carry it.
        self._tooling_drive = tooling_drive
        self._vacuum_bit = vacuum_do_bit
        self._plunger_bit = plunger_do_bit
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

    def _confirmed_fault(self) -> Optional[str]:
        """Return a fault description only if the fault bit SURVIVES three
        consecutive fresh PDO cycles.

        A real drive fault LATCHES until it is explicitly reset, so re-reading
        never hides one. A single bad statusword sample — a torn shared-memory
        read racing the RT daemon, or a one-cycle bus hiccup — does not survive
        a re-read, and must not kill a move (it aborted a demo run with
        'drives are faulted' while both drives were verifiably fine)."""
        for attempt in range(3):
            if not self.is_faulted:
                if attempt:
                    logger.warning(
                        "ignored transient fault bit (cleared on re-read %d)",
                        attempt)
                return None
            try:
                self.master.exchange()             # fresh process image
            except Exception:  # noqa: BLE001 - fall through with what we have
                break
        if not self.is_faulted:
            logger.warning("ignored transient fault bit (cleared on re-read)")
            return None
        return ", ".join(
            f"drive {i}: sw=0x{d.statusword:04X} err=0x{d.error_code:04X}"
            for i, d in enumerate(self.master.drives)
            if cia402.is_fault(d.statusword))

    def _disabled_detail(self) -> str:
        """Human-readable per-drive state for a 'not enabled' refusal: decoded
        CiA 402 state + raw statusword + drive error code, with an STO hint
        when the voltage bit is down. A drive that silently falls back to
        SWITCH ON DISABLED (no fault) usually means the power-stage enable
        chain blipped — STO/E-stop chatter, 24 V dip, or a drive-side sync
        reaction — and the raw words are what identify which."""
        parts = []
        for i, d in enumerate(self.master.drives):
            if cia402.is_operation_enabled(d.statusword):
                continue
            st = cia402.decode_state(d.statusword).value.replace("_", " ").upper()
            hint = ("" if d.statusword & cia402.SW_VOLTAGE_ENABLED
                    else " - voltage bit CLEAR: STO/E-stop chain open?")
            parts.append(f"drive {i}: {st} sw=0x{d.statusword:04X} "
                         f"err=0x{getattr(d, 'error_code', 0):04X}{hint}")
        return "; ".join(parts) or "all drives report Operation Enabled"

    # --- enable / reset -----------------------------------------------------
    def enable(self) -> None:
        fault = self._confirmed_fault()
        if fault:
            raise RobotDriverError(f"drives are faulted; reset before enabling ({fault})")
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
        fault = self._confirmed_fault()
        if fault:
            raise RobotDriverError(f"cannot jog: drive fault ({fault})")
        if not self.is_enabled:
            raise RobotDriverError(
                f"cannot jog: enable the drive first ({self._disabled_detail()})")
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
        fault = self._confirmed_fault()
        if fault:
            raise RobotDriverError(f"cannot move: drive fault ({fault})")
        if not self.is_enabled:
            raise RobotDriverError(
                f"cannot move: enable the drives first ({self._disabled_detail()})")
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
    def move_to_angles(self, left_deg: float, right_deg: float,
                       speed_mm_s: Optional[float] = None) -> None:
        fault = self._confirmed_fault()
        if fault:
            raise RobotDriverError(f"cannot move: drive fault ({fault})")
        if not self.is_enabled:
            raise RobotDriverError(
                f"cannot move: drives are disabled ({self._disabled_detail()})")
        cur = self.read_angles()
        if cur is None:
            raise RobotDriverError("cannot move: robot is not referenced")
        start_xy = self.kin.forward(*cur)
        goal_xy = self.kin.forward(left_deg, right_deg)
        limits = self.limits
        if speed_mm_s and speed_mm_s > 0:
            limits = dataclasses.replace(limits, speed_mm_s=float(speed_mm_s))
        try:
            traj = plan_linear_move(self.kin, self.validator, start_xy, goal_xy, limits)
        except TrajectoryError as exc:
            raise RobotDriverError(f"move planning failed: {exc}") from exc
        self._stream(traj)
        logger.info("moved -> L=%.3f R=%.3f (%d cycles)", left_deg, right_deg, len(traj))

    def jog_cartesian(self, dx_mm: float, dy_mm: float,
                      speed_mm_s: Optional[float] = None) -> None:
        """HMI/bench jog: move the TCP by (dx, dy) mm in a validated straight
        line. Requires Operation Enabled and a referenced robot. Motion — only
        with the E-stop/contactor live."""
        fault = self._confirmed_fault()
        if fault:
            raise RobotDriverError(f"cannot jog: drive fault ({fault})")
        if not self.is_enabled:
            raise RobotDriverError(
                f"cannot jog: enable the drives first ({self._disabled_detail()})")
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

    def characterize(self, dx_mm: float, dy_mm: float,
                     speed_mm_s: float) -> List[int]:
        """Run a validated out-and-back TCP move and return the peak |following
        error| per drive (counts) seen during it — the tuning assistant's grade
        of how well the servo tracks at ``speed_mm_s``. Ends back at the start
        pose. Motion — needs enable + reference (jog_cartesian enforces both)."""
        peaks = [0] * len(self.master.drives)

        def _leg(ddx: float, ddy: float) -> None:
            self.jog_cartesian(ddx, ddy, speed_mm_s=speed_mm_s)
            got = getattr(self.master, "csp_fe_peak", lambda: [])()
            for i, v in enumerate(got):
                if i < len(peaks):
                    peaks[i] = max(peaks[i], abs(int(v)))

        _leg(dx_mm, dy_mm)
        _leg(-dx_mm, -dy_mm)
        return peaks

    def _stream(self, traj) -> None:
        """Hand the precomputed per-cycle CSP setpoints to the master to stream.

        The master owns the timing: the simulator plays them out synchronously,
        the real master's SCHED_FIFO thread plays one per DC cycle. We converted
        the whole plan to drive counts up front, so the real-time loop does zero
        kinematics and zero allocation."""
        h0, h1 = self._home_counts
        targets = [(sp.left_counts - h0, sp.right_counts - h1) for sp in traj.setpoints]
        # Velocity feedforward from the (smooth) trajectory velocity, streamed as
        # 0x60B1 so the drive doesn't have to differentiate the position steps.
        velocities = None
        if self.velocity_ff_scale:
            velocities = setpoint_velocities(traj.setpoints, traj.cycle_dt_s,
                                             self.velocity_ff_scale)
        try:
            self.master.run_csp(targets, velocities)
        except MasterError as exc:
            raise RobotDriverError(f"move aborted: {exc}") from exc
        want = targets[-1]
        got = self._settle(want)
        if got is None:
            final = tuple(d.actual_position for d in self.master.drives)
            errs = {i: want[i] - final[i]
                    for i in range(min(len(want), len(final)))
                    if abs(want[i] - final[i]) > self.position_tol_counts}
            short = ", ".join(
                f"drive {i}: {abs(e)} counts ({abs(e) / self._ppd:.3f} deg) short"
                for i, e in errs.items())
            turn_hint = self._whole_turn_hint(errs)
            raise RobotDriverError(
                f"move did not settle within {self.settle_timeout_s:g}s "
                f"(tol {self.position_tol_counts} counts): {short}.{turn_hint} "
                f"Raise settle_timeout_s / position_tol_counts in Parameters, or "
                f"tune the drives (stiffness / position loop gain).")

    def _whole_turn_hint(self, errs) -> str:
        """If a shortfall is (within tolerance) an exact multiple of one encoder
        revolution, the feedback datum is offset by whole turns — the classic
        symptom of switching C00.07 to multi-turn absolute without clearing the
        drive's multi-turn data. The 5-bar shoulders sweep a bounded arc and
        never spin whole revs, so a whole-turn miss is never real motion; it is
        a stale absolute datum. Name it, because 'tune the drives' is wrong here."""
        rev = self.kin.config.pulses_per_rev
        tol = self.position_tol_counts
        offenders = {i: e for i, e in errs.items()
                     if abs(abs(e) % rev) <= tol or abs(abs(e) % rev - rev) <= tol}
        if not offenders:
            return ""
        who = ", ".join(f"drive {i} ({round(abs(e) / rev)} turn"
                        f"{'s' if round(abs(e) / rev) != 1 else ''})"
                        for i, e in offenders.items())
        return (f" This shortfall is a whole number of encoder revolutions "
                f"({who}, {rev} counts/rev): the multi-turn absolute datum is "
                f"offset, not a servo-tuning issue. Clear the drive's multi-turn "
                f"encoder data (needed once after setting C00.07=4 + fitting the "
                f"battery), power-cycle the drive, then Set Home again.")

    def _settle(self, want, timeout_s: Optional[float] = None):
        """Wait for the servos to catch up to the final CSP target after the
        stream ends. A real drive lags its target (following error) and its
        integrator needs real time to pull in the last fraction of a degree;
        the window defaults to ``settle_timeout_s`` (a Drives-tab parameter).
        The simulator is instantaneous, so it passes on the first read.
        Returns the settled position tuple, or None on timeout."""
        if timeout_s is None:
            timeout_s = self.settle_timeout_s
        deadline = time.perf_counter() + timeout_s
        while True:
            got = tuple(d.actual_position for d in self.master.drives)
            if all(abs(got[i] - want[i]) <= self.position_tol_counts
                   for i in range(min(len(got), len(want)))):
                return got
            if time.perf_counter() > deadline:
                return None
            self.master.exchange()       # refresh the process image while it settles
            time.sleep(0.005)

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

    # --- end-effector I/O ---------------------------------------------------
    def set_vacuum(self, on: bool) -> None:
        """Energize/vent the vacuum solenoid via the tooling drive's DO."""
        self._set_do(self._vacuum_bit, on)

    def set_plunger(self, extended: bool) -> None:
        """Extend/retract the pick air cylinder via the tooling drive's DO."""
        self._set_do(self._plunger_bit, extended)

    def _set_do(self, bit: int, on: bool) -> None:
        """Set one bit of the tooling drive's digital-output word (0x60FE:1) and
        push it to the bus. The daemon writes the whole word every DC cycle, so
        the bit stays asserted until we clear it."""
        d = self.master.drives[self._tooling_drive]
        mask = 1 << bit
        if on:
            d.digital_outputs = (d.digital_outputs | mask) & 0xFFFFFFFF
        else:
            d.digital_outputs = d.digital_outputs & ~mask & 0xFFFFFFFF
        try:
            self.master.exchange()
        except MasterError as exc:
            raise RobotDriverError(f"digital-output write failed: {exc}") from exc
