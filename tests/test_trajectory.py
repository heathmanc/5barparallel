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
