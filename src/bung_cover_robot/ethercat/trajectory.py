"""Cartesian straight-line trajectory planning for CSP streaming.

The PC is the motion controller: to move the TCP from A to B in a coordinated
*straight line*, we can't just hand each drive an endpoint (independent joint
moves bow the path). Instead we:

    1. profile the TCP speed along the straight A->B segment (trapezoidal: ramp
       up to a cruise speed, cruise, ramp down),
    2. sample that path at the EtherCAT cycle rate (dt),
    3. inverse-kinematics each sampled TCP point to shoulder angles, and
    4. validate every sample through WorkspaceValidator.

The result is a flat array of per-cycle joint setpoints. The real-time thread
then just indexes this array and writes the two Cyclic-Synchronous-Position
targets each DC cycle — it does NO kinematics and NO allocation, which is what
keeps Python safe inside the real-time loop.

Because every point is validated at plan time, an unreachable or near-singular
path is rejected *before* any motion starts (nothing unvalidated ever reaches the
drives) — the same guarantee the old PLC path gave, now on the PC.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from ..robot.fivebar_kinematics import FiveBarKinematics, KinematicsError
from ..robot.workspace import WorkspaceValidator

Point = Tuple[float, float]


def ramp_counts(start: int, delta: int, speed: float, accel: float,
                dt: float) -> List[int]:
    """Trapezoidal 1-D position ramp in raw drive counts, for a single-axis jog.

    Returns per-cycle absolute count targets from ``start`` to ``start+delta``
    following a trapezoidal velocity profile (accel to ``speed`` counts/s at
    ``accel`` counts/s^2, cruise, decel), sampled at ``dt``. The last sample is
    exactly ``start+delta``. A step (no ramp) would fault a CSP drive on
    following error, so even a bench jog must ramp.
    """
    if speed <= 0 or accel <= 0 or dt <= 0:
        raise ValueError("speed, accel and dt must be positive")
    D = abs(int(delta))
    if D == 0:
        return [int(start)]
    sign = 1 if delta > 0 else -1
    v, a = float(speed), float(accel)
    t_acc = v / a
    d_acc = 0.5 * a * t_acc * t_acc
    if 2.0 * d_acc >= D:                     # triangular (never reaches cruise)
        t_acc = math.sqrt(D / a)
        v = a * t_acc
        d_acc = 0.5 * a * t_acc * t_acc
        t_flat = 0.0
    else:
        t_flat = (D - 2.0 * d_acc) / v
    total = 2.0 * t_acc + t_flat
    out: List[int] = []
    t = 0.0
    while t < total:
        if t < t_acc:
            d = 0.5 * a * t * t
        elif t < t_acc + t_flat:
            d = d_acc + v * (t - t_acc)
        else:
            td = t - t_acc - t_flat
            d = d_acc + v * t_flat + v * td - 0.5 * a * td * td
        out.append(int(start) + sign * int(round(d)))
        t += dt
    out.append(int(start) + int(delta))      # exact endpoint
    return out


def ramp_counts_multi(starts: List[int], deltas: List[int], speed: float,
                      accel: float, dt: float) -> List[Tuple[int, ...]]:
    """Synchronized N-axis trapezoidal ramp for a coordinated bench move.

    Every axis follows ONE shared time profile — at each cycle all axes are at
    the same fraction of their own delta — so they start and finish together
    (the point of the test: two drives streaming in lockstep off one CSP
    stream). The profile is trapezoidal in the dominant axis (largest |delta|),
    so its velocity stays <= ``speed`` counts/s at ``accel`` counts/s^2; the
    others move proportionally slower. Returns one per-cycle tuple of absolute
    count targets per axis; the last tuple lands exactly on ``start+delta``.

    Joint-space only (no kinematics/workspace check), so it is safe before the
    arm is assembled into the linkage — you command explicit small count deltas.
    """
    if speed <= 0 or accel <= 0 or dt <= 0:
        raise ValueError("speed, accel and dt must be positive")
    if len(starts) != len(deltas):
        raise ValueError("starts and deltas must have the same length")
    ds = [int(x) for x in deltas]
    D = max((abs(x) for x in ds), default=0)
    if D == 0:
        return [tuple(int(s) for s in starts)]
    v, a = float(speed), float(accel)
    t_acc = v / a
    d_acc = 0.5 * a * t_acc * t_acc
    if 2.0 * d_acc >= D:                      # triangular (never reaches cruise)
        t_acc = math.sqrt(D / a)
        v = a * t_acc
        d_acc = 0.5 * a * t_acc * t_acc
        t_flat = 0.0
    else:
        t_flat = (D - 2.0 * d_acc) / v
    total = 2.0 * t_acc + t_flat
    out: List[Tuple[int, ...]] = []
    t = 0.0
    while t < total:
        if t < t_acc:
            d = 0.5 * a * t * t
        elif t < t_acc + t_flat:
            d = d_acc + v * (t - t_acc)
        else:
            td = t - t_acc - t_flat
            d = d_acc + v * t_flat + v * td - 0.5 * a * td * td
        frac = d / D                          # shared fraction -> synchronized
        out.append(tuple(int(s) + int(round(frac * dl))
                         for s, dl in zip(starts, ds)))
        t += dt
    out.append(tuple(int(s) + dl for s, dl in zip(starts, ds)))   # exact endpoints
    return out


class TrajectoryError(Exception):
    """A path could not be planned (unreachable/near-singular sample, or bad
    limits). Raised at plan time so nothing partial is ever streamed."""


@dataclass(frozen=True)
class TrajectoryLimits:
    """Cartesian motion limits + the EtherCAT cycle time.

    Defaults are conservative for a bench bring-up; tune once the drives are
    tuned. ``cycle_dt_s`` must match the DC cycle the master runs at.
    """

    speed_mm_s: float = 200.0
    accel_mm_s2: float = 2000.0
    cycle_dt_s: float = 0.002        # 2 ms = 500 Hz DC cycle
    # Optional jerk limit (mm/s^3). None/0 = trapezoidal (acceleration steps on
    # instantly = infinite jerk, which pings mechanical resonances: the audible
    # chirp + following-error spike at move start). A finite jerk eases the
    # acceleration in over ~accel/jerk seconds (an S-curve), killing that impulse
    # with almost no loss of cruise speed.
    jerk_mm_s3: float | None = None
    # Optional guard: reject a plan whose per-cycle shoulder step exceeds this
    # (a proxy for joint-velocity blow-up near a singularity). None = no cap
    # (the workspace validator already excludes the near-singular band).
    max_joint_step_deg: float | None = None

    def __post_init__(self) -> None:
        if self.speed_mm_s <= 0 or self.accel_mm_s2 <= 0:
            raise ValueError("speed and accel must be positive")
        if self.cycle_dt_s <= 0:
            raise ValueError("cycle_dt_s must be positive")
        if self.jerk_mm_s3 is not None and self.jerk_mm_s3 < 0:
            raise ValueError("jerk_mm_s3 must be positive (or None to disable)")


@dataclass(frozen=True)
class JointSetpoint:
    """One cycle's commanded pose: shoulder angles (deg) + drive counts."""

    left_deg: float
    right_deg: float
    left_counts: int
    right_counts: int


@dataclass(frozen=True)
class Trajectory:
    """A time-parameterized joint path, one setpoint per EtherCAT cycle."""

    setpoints: List[JointSetpoint]
    cycle_dt_s: float
    start_xy: Point
    goal_xy: Point
    max_joint_step_deg: float

    def __len__(self) -> int:
        return len(self.setpoints)

    @property
    def duration_s(self) -> float:
        return max(len(self.setpoints) - 1, 0) * self.cycle_dt_s

    @property
    def final(self) -> JointSetpoint:
        return self.setpoints[-1]


def _trapezoid(length: float, v: float, a: float):
    """Return (total_time, t_acc, t_cruise, v_peak) for a trapezoidal (or, when
    the segment is too short to reach ``v``, triangular) speed profile over
    ``length``, ramping at accel ``a`` up to cruise speed ``v``."""
    d_ramp = v * v / (2.0 * a)          # distance to reach v from rest
    if 2.0 * d_ramp <= length:          # trapezoid: room to cruise
        t_acc = v / a
        d_cruise = length - 2.0 * d_ramp
        t_cruise = d_cruise / v
        return (2.0 * t_acc + t_cruise, t_acc, t_cruise, v)
    # triangle: peak below v
    v_peak = math.sqrt(length * a)
    t_acc = v_peak / a
    return (2.0 * t_acc, t_acc, 0.0, v_peak)


def _distance_at(t: float, t_acc: float, t_cruise: float, v_peak: float, a: float) -> float:
    """Distance travelled along the path at time ``t`` for the profile."""
    if t <= t_acc:
        return 0.5 * a * t * t
    d_acc = 0.5 * a * t_acc * t_acc
    if t <= t_acc + t_cruise:
        return d_acc + v_peak * (t - t_acc)
    td = t - t_acc - t_cruise
    d_cruise = v_peak * t_cruise
    return d_acc + d_cruise + v_peak * td - 0.5 * a * td * td


def setpoint_velocities(setpoints: List[JointSetpoint], dt: float,
                        scale: float = 1.0) -> List[Tuple[int, int]]:
    """Per-cycle joint velocity (drive counts/s) for each setpoint — the central
    difference of the count path, ``scale``-trimmed. Streamed as the 0x60B1
    velocity offset so the drive feedforwards OUR trajectory velocity instead of
    differentiating the position steps (which chirps). ``scale`` lets the bench
    trim for the drive's velocity-offset units (1.0 = counts/s)."""
    n = len(setpoints)
    if n == 0 or dt <= 0:
        return []
    out: List[Tuple[int, int]] = []
    for i in range(n):
        lo, hi = max(0, i - 1), min(n - 1, i + 1)
        span = (hi - lo) * dt or dt
        vl = (setpoints[hi].left_counts - setpoints[lo].left_counts) / span
        vr = (setpoints[hi].right_counts - setpoints[lo].right_counts) / span
        out.append((int(round(vl * scale)), int(round(vr * scale))))
    return out


def _trap_velocity(t: float, t_acc: float, t_cruise: float, v_peak: float,
                   a: float) -> float:
    """Speed along the path at time ``t`` for the trapezoidal profile."""
    if t < t_acc:
        return a * t
    if t < t_acc + t_cruise:
        return v_peak
    td = t - t_acc - t_cruise
    return max(0.0, v_peak - a * td)


def _scurve_distances(length: float, v: float, a: float, jerk: float,
                      dt: float) -> List[float]:
    """Per-cycle cumulative distances for a JERK-LIMITED (S-curve) move.

    Built by box-filtering the trapezoidal velocity profile with a window of
    width ``Tj = a / jerk``: convolving a trapezoid (piecewise-constant
    acceleration) with a rectangle yields piecewise-linear acceleration, i.e.
    bounded jerk. The filtered velocity is integrated to distance and scaled so
    the last sample lands exactly on ``length``. Returns ``[0.0, ..., length]``.
    """
    total_t, t_acc, t_cruise, v_peak = _trapezoid(length, v, a)
    k = max(1, int(round((a / jerk) / dt))) if jerk and jerk > 0 else 1
    n = max(1, int(math.ceil(total_t / dt)))
    # trapezoidal velocity samples + (k-1) zero-flush so the trailing average
    # eases back to rest (and total area — distance — is preserved).
    vel = [_trap_velocity(i * dt, t_acc, t_cruise, v_peak, a) for i in range(n)]
    vel += [0.0] * (k - 1)
    dists: List[float] = [0.0]
    window = 0.0
    s = 0.0
    for i, vv in enumerate(vel):
        window += vv
        if i >= k:
            window -= vel[i - k]
        s += (window / k) * dt          # box-filtered (jerk-limited) velocity
        dists.append(s)
    if dists[-1] > 0:                    # correct tiny discretisation drift
        scale = length / dists[-1]
        dists = [d * scale for d in dists]
    dists[-1] = length
    return dists


def plan_linear_move(
    kin: FiveBarKinematics,
    validator: WorkspaceValidator,
    start_xy: Point,
    goal_xy: Point,
    limits: TrajectoryLimits | None = None,
) -> Trajectory:
    """Plan a validated straight-line TCP move from ``start_xy`` to ``goal_xy``.

    Every sampled point is workspace-validated; the first failure raises
    ``TrajectoryError`` and no trajectory is returned. The final setpoint is
    forced to land exactly on ``goal_xy``.
    """
    limits = limits or TrajectoryLimits()
    sx, sy = float(start_xy[0]), float(start_xy[1])
    gx, gy = float(goal_xy[0]), float(goal_xy[1])
    length = math.hypot(gx - sx, gy - sy)

    def setpoint_at(x: float, y: float) -> JointSetpoint:
        try:
            res = validator.validate(x, y)
        except Exception as exc:  # noqa: BLE001 - validator shouldn't raise, but be safe
            raise TrajectoryError(f"validation error at ({x:.1f},{y:.1f}): {exc}") from exc
        if not res.ok:
            raise TrajectoryError(
                f"path point ({x:.1f}, {y:.1f}) rejected: {res.reason}"
            )
        try:
            jt = kin.inverse(x, y)
        except KinematicsError as exc:
            raise TrajectoryError(str(exc)) from exc
        return JointSetpoint(jt.left_deg, jt.right_deg, jt.left_pulses, jt.right_pulses)

    # Degenerate move: no travel -> a single validated hold at the start.
    if length < 1e-9:
        sp = setpoint_at(sx, sy)
        return Trajectory([sp], limits.cycle_dt_s, (sx, sy), (gx, gy), 0.0)

    ux, uy = (gx - sx) / length, (gy - sy) / length   # unit direction
    setpoints: List[JointSetpoint] = []
    if limits.jerk_mm_s3:
        # Jerk-limited (S-curve): pre-computed per-cycle distances.
        dists = _scurve_distances(length, limits.speed_mm_s, limits.accel_mm_s2,
                                  limits.jerk_mm_s3, limits.cycle_dt_s)
        last = len(dists) - 1
        for i, s in enumerate(dists):
            if i == last:
                x, y = gx, gy                          # land exactly on goal
            else:
                s = min(s, length)
                x, y = sx + ux * s, sy + uy * s
            setpoints.append(setpoint_at(x, y))
    else:
        total_t, t_acc, t_cruise, v_peak = _trapezoid(
            length, limits.speed_mm_s, limits.accel_mm_s2
        )
        n_steps = max(1, math.ceil(total_t / limits.cycle_dt_s))
        for i in range(n_steps + 1):
            if i == n_steps:
                x, y = gx, gy                          # land exactly on goal
            else:
                t = i * limits.cycle_dt_s
                s = _distance_at(t, t_acc, t_cruise, v_peak, limits.accel_mm_s2)
                s = min(s, length)
                x, y = sx + ux * s, sy + uy * s
            setpoints.append(setpoint_at(x, y))

    # Largest per-cycle shoulder step — a proxy for joint-velocity spikes.
    max_step = 0.0
    for a_sp, b_sp in zip(setpoints, setpoints[1:]):
        step = max(abs(b_sp.left_deg - a_sp.left_deg),
                   abs(b_sp.right_deg - a_sp.right_deg))
        max_step = max(max_step, step)
    if limits.max_joint_step_deg is not None and max_step > limits.max_joint_step_deg:
        raise TrajectoryError(
            f"per-cycle shoulder step {max_step:.3f} deg exceeds cap "
            f"{limits.max_joint_step_deg:.3f} deg — slow the move or the path nears "
            "a singularity"
        )
    return Trajectory(setpoints, limits.cycle_dt_s, (sx, sy), (gx, gy), max_step)
