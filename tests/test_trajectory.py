"""Cartesian straight-line CSP trajectory planning."""

import math

import pytest

from bung_cover_robot.ethercat.trajectory import (
    TrajectoryError,
    TrajectoryLimits,
    plan_linear_move,
)
from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics
from bung_cover_robot.robot.workspace import WorkspaceValidator


@pytest.fixture
def kv():
    kin = FiveBarKinematics()
    return kin, WorkspaceValidator(kin)


def test_endpoints_are_exact(kv):
    kin, val = kv
    start, goal = (0.0, 250.0), (60.0, 250.0)
    traj = plan_linear_move(kin, val, start, goal)
    # first sample IK-solves the start, last lands exactly on the goal
    assert traj.setpoints[0].left_deg == pytest.approx(kin.inverse(*start).left_deg)
    assert traj.final.left_deg == pytest.approx(kin.inverse(*goal).left_deg)
    assert traj.final.right_deg == pytest.approx(kin.inverse(*goal).right_deg)


def test_path_is_a_straight_line(kv):
    kin, val = kv
    start, goal = (-80.0, 240.0), (80.0, 260.0)
    traj = plan_linear_move(kin, val, start, goal)
    # forward-solve each setpoint's TCP and check it lies on the A->B segment
    for sp in traj.setpoints:
        x, y = kin.forward(sp.left_deg, sp.right_deg)
        # distance from point to the infinite line through start->goal
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        L = math.hypot(dx, dy)
        cross = abs((x - start[0]) * dy - (y - start[1]) * dx) / L
        assert cross < 0.05          # within 50 microns of the straight line


def test_samples_at_cycle_rate_and_duration(kv):
    kin, val = kv
    limits = TrajectoryLimits(speed_mm_s=100.0, accel_mm_s2=1000.0, cycle_dt_s=0.002)
    traj = plan_linear_move(kin, val, (0.0, 250.0), (100.0, 250.0), limits)
    # setpoints are one cycle apart; duration is a multiple of dt
    assert traj.cycle_dt_s == 0.002
    assert len(traj) >= 2
    assert traj.duration_s == pytest.approx((len(traj) - 1) * 0.002)


def test_trapezoid_longer_than_triangle(kv):
    kin, val = kv
    # a long move reaches cruise speed; total time > a pure triangle would give
    limits = TrajectoryLimits(speed_mm_s=50.0, accel_mm_s2=1000.0, cycle_dt_s=0.002)
    short = plan_linear_move(kin, val, (0.0, 250.0), (10.0, 250.0), limits)
    long = plan_linear_move(kin, val, (-90.0, 250.0), (90.0, 250.0), limits)
    assert long.duration_s > short.duration_s
    # cruise-limited: long move's average speed approaches the speed cap
    avg_v = 180.0 / long.duration_s
    assert avg_v <= 50.0 + 1e-6


def test_zero_length_move_is_single_hold(kv):
    kin, val = kv
    traj = plan_linear_move(kin, val, (0.0, 250.0), (0.0, 250.0))
    assert len(traj) == 1
    assert traj.duration_s == 0.0


def test_unreachable_goal_raises_before_streaming(kv):
    kin, val = kv
    with pytest.raises(TrajectoryError):
        plan_linear_move(kin, val, (0.0, 250.0), (0.0, 9999.0))


def test_path_through_unsafe_region_raises(kv):
    kin, _ = kv
    from bung_cover_robot.robot.workspace import ValidationResult

    class _BandRejectingValidator:
        """Valid everywhere except a vertical band the straight path must cross,
        so an interior sample (not an endpoint) is the one rejected."""

        def validate(self, x, y):
            if -20.0 <= x <= 20.0:
                return ValidationResult(False, "in the forbidden band", {})
            return ValidationResult(True, "ok", {})

    with pytest.raises(TrajectoryError, match="forbidden band"):
        plan_linear_move(kin, _BandRejectingValidator(), (-60.0, 250.0), (60.0, 250.0))


def test_counts_match_kinematics(kv):
    kin, val = kv
    traj = plan_linear_move(kin, val, (0.0, 250.0), (30.0, 250.0))
    sp = traj.final
    assert sp.left_counts == kin.degrees_to_pulses(sp.left_deg)
    assert sp.right_counts == kin.degrees_to_pulses(sp.right_deg)


def test_max_joint_step_guard_can_reject(kv):
    kin, val = kv
    # an absurdly tight per-cycle cap trips the guard on any real move
    limits = TrajectoryLimits(max_joint_step_deg=1e-4)
    with pytest.raises(TrajectoryError):
        plan_linear_move(kin, val, (0.0, 250.0), (80.0, 250.0), limits)


# --- S-curve (jerk-limited) profile ---------------------------------------- #
def _accels(dists, dt):
    return [(dists[i + 1] - 2 * dists[i] + dists[i - 1]) / (dt * dt)
            for i in range(1, len(dists) - 1)]


def test_scurve_distances_endpoints_and_monotonic():
    from bung_cover_robot.ethercat.trajectory import _scurve_distances
    s = _scurve_distances(180.0, 1000.0, 2000.0, 80000.0, 0.002)
    assert s[0] == 0.0
    assert s[-1] == pytest.approx(180.0)
    assert all(s[i + 1] >= s[i] - 1e-9 for i in range(len(s) - 1))   # monotone


def test_scurve_softens_start_and_bounds_jerk():
    from bung_cover_robot.ethercat.trajectory import _scurve_distances
    dt, a, jerk = 0.002, 2000.0, 80000.0
    s = _scurve_distances(180.0, 1000.0, a, jerk, dt)
    acc = _accels(s, dt)
    peak = max(abs(x) for x in acc)
    # ease-in: acceleration at the first interior cycle is a small fraction of
    # peak (a hard trapezoid would already be at full accel there).
    assert abs(acc[0]) < 0.3 * peak
    # realized jerk is near the limit and FAR below the trapezoid's a/dt step.
    jerks = [abs(acc[i + 1] - acc[i]) / dt for i in range(len(acc) - 1)]
    assert max(jerks) < 4.0 * jerk
    assert max(jerks) < (a / dt) / 3.0        # much softer than a hard step


def test_scurve_reaches_goal_and_stays_straight(kv):
    kin, val = kv
    start, goal = (-80.0, 240.0), (80.0, 260.0)
    lim = TrajectoryLimits(speed_mm_s=1000.0, accel_mm_s2=2000.0,
                           cycle_dt_s=0.002, jerk_mm_s3=80000.0)
    traj = plan_linear_move(kin, val, start, goal, lim)
    assert traj.final.left_deg == pytest.approx(kin.inverse(*goal).left_deg)
    assert traj.final.right_deg == pytest.approx(kin.inverse(*goal).right_deg)
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    L = math.hypot(dx, dy)
    for sp in traj.setpoints:
        x, y = kin.forward(sp.left_deg, sp.right_deg)
        cross = abs((x - start[0]) * dy - (y - start[1]) * dx) / L
        assert cross < 0.05


def test_scurve_no_longer_than_reasonable_and_off_matches_trapezoid(kv):
    kin, val = kv
    start, goal = (0.0, 250.0), (120.0, 250.0)
    base = TrajectoryLimits(speed_mm_s=1000.0, accel_mm_s2=2000.0, cycle_dt_s=0.002)
    trap = plan_linear_move(kin, val, start, goal, base)
    scrv = plan_linear_move(kin, val, start, goal,
                            TrajectoryLimits(speed_mm_s=1000.0, accel_mm_s2=2000.0,
                                             cycle_dt_s=0.002, jerk_mm_s3=80000.0))
    # jerk=None keeps the trapezoid path (default behaviour unchanged)
    off = plan_linear_move(kin, val, start, goal,
                           TrajectoryLimits(speed_mm_s=1000.0, accel_mm_s2=2000.0,
                                            cycle_dt_s=0.002, jerk_mm_s3=None))
    assert len(off) == len(trap)
    # S-curve adds smoothing time, but not excessively (< 60 % longer here)
    assert len(trap) <= len(scrv) < 1.6 * len(trap)


def test_negative_jerk_rejected():
    with pytest.raises(ValueError):
        TrajectoryLimits(jerk_mm_s3=-1.0)


def test_setpoint_velocities_is_count_derivative(kv):
    from bung_cover_robot.ethercat.trajectory import setpoint_velocities
    kin, val = kv
    lim = TrajectoryLimits(speed_mm_s=500.0, accel_mm_s2=2000.0, cycle_dt_s=0.002)
    traj = plan_linear_move(kin, val, (0.0, 250.0), (60.0, 250.0), lim)
    sp = traj.setpoints
    vel = setpoint_velocities(sp, 0.002)
    assert len(vel) == len(sp)
    i = len(sp) // 2                                   # interior: central diff
    assert vel[i][0] == round((sp[i + 1].left_counts - sp[i - 1].left_counts) / (2 * 0.002))
    assert max(abs(v[0]) for v in vel) > 0             # a real move has velocity
    # scale trims proportionally; empty / bad-dt guards
    v2 = setpoint_velocities(sp, 0.002, scale=2.0)
    assert abs(v2[i][0] - 2 * vel[i][0]) <= 1
    assert setpoint_velocities([], 0.002) == []
    assert setpoint_velocities(sp, 0.0) == []
