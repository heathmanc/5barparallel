"""Home-offset solver: bridge a known TCP pose + commanded step count -> offset."""

import pytest

from bung_cover_robot.robot import (
    FiveBarKinematics,
    KinematicsError,
    solve_home_offsets,
)


def test_offsets_equal_true_angle_when_posn_zero():
    kin = FiveBarKinematics()
    spd = kin.config.pulses_per_degree
    jt = kin.inverse(0.0, 250.0)
    sol = solve_home_offsets(kin, 0.0, 250.0, 0, 0)
    # posn 0 (arm exactly at the datum) -> offset is just theta * steps/deg
    assert sol.offset_left == round(jt.left_deg * spd)
    assert sol.offset_right == round(jt.right_deg * spd)
    assert sol.theta_left_deg == pytest.approx(jt.left_deg)


def test_commanded_posn_shifts_offset_one_for_one():
    kin = FiveBarKinematics()
    spd = kin.config.pulses_per_degree
    jt = kin.inverse(0.0, 250.0)
    sol = solve_home_offsets(kin, 0.0, 250.0, 42, -33)
    assert sol.offset_left == round(jt.left_deg * spd - 42)
    assert sol.offset_right == round(jt.right_deg * spd + 33)
    # the whole point: ActualDeg = (posn + offset)/spd recovers the true angle
    assert (42 + sol.offset_left) / spd == pytest.approx(jt.left_deg, abs=0.02)
    assert (-33 + sol.offset_right) / spd == pytest.approx(jt.right_deg, abs=0.02)


def test_unreachable_jig_point_raises():
    kin = FiveBarKinematics()
    with pytest.raises(KinematicsError):
        solve_home_offsets(kin, 0.0, 9999.0, 0, 0)
