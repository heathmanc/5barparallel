"""RobotTestController: home (teach + go) and jog, all gated by the workspace
validator."""

import pytest

from bung_cover_robot.app.robot_test_controller import (
    DEFAULT_HOME_XY,
    RobotTestController,
)
from bung_cover_robot.robot import DryRunRobotDriver


def make() -> RobotTestController:
    return RobotTestController(DryRunRobotDriver())


# --------------------------------------------------------------------------- #
# Initial state
# --------------------------------------------------------------------------- #
def test_initial_state_is_valid_home_not_enabled_not_homed():
    c = make()
    assert c.state.tcp == DEFAULT_HOME_XY
    assert not c.is_enabled
    assert not c.is_homed
    # A resolved, validated pose from the start.
    assert c.state.metrics["reach_fraction"] < 0.85


def test_invalid_home_rejected_at_construction():
    with pytest.raises(ValueError):
        RobotTestController(DryRunRobotDriver(), home_xy=(0.0, 900.0))


# --------------------------------------------------------------------------- #
# Motion gating
# --------------------------------------------------------------------------- #
def test_jog_blocked_until_enabled():
    c = make()
    res = c.jog_joint("left", 1.0)
    assert not res.ok and "disabled" in res.reason


def test_jog_blocked_until_homed():
    c = make()
    c.enable()
    res = c.jog_joint("left", 1.0)
    assert not res.ok and "homed" in res.reason


def test_go_home_blocked_until_enabled():
    c = make()
    res = c.go_home()
    assert not res.ok and "disabled" in res.reason


# --------------------------------------------------------------------------- #
# Home
# --------------------------------------------------------------------------- #
def test_go_home_sets_homed_and_commands_driver():
    c = make()
    c.enable()
    res = c.go_home()
    assert res.ok
    assert c.is_homed
    assert c.driver.read_angles() == (c.state.left_deg, c.state.right_deg)


def test_set_home_teaches_current_pose():
    c = make()
    c.enable()
    c.go_home()
    c.jog_cartesian("y", 10.0)  # move to (0, 260)
    taught = c.set_home()
    assert taught == c.state.tcp == (0.0, 260.0)
    # Move away, then Go Home returns to the taught point.
    c.jog_cartesian("x", 20.0)
    c.go_home()
    assert c.state.tcp == pytest.approx((0.0, 260.0))


# --------------------------------------------------------------------------- #
# Jog
# --------------------------------------------------------------------------- #
def test_jog_joint_moves_one_shoulder():
    c = make()
    c.enable()
    c.go_home()
    before = c.state
    res = c.jog_joint("left", 2.0)
    assert res.ok
    assert c.state.left_deg == pytest.approx(before.left_deg + 2.0)
    assert c.state.right_deg == pytest.approx(before.right_deg)


def test_jog_cartesian_moves_tcp():
    c = make()
    c.enable()
    c.go_home()
    res = c.jog_cartesian("x", 15.0)
    assert res.ok
    assert c.state.tcp == pytest.approx((15.0, 250.0))


def test_jog_out_of_workspace_is_rejected_and_pose_unchanged():
    c = make()
    c.enable()
    c.go_home()
    before = c.state
    n_cmds = len(c.driver.command_log)
    res = c.move_to_xy(0.0, 430.0)  # past the 85% reach cap
    assert not res.ok
    assert "reach" in res.reason
    assert c.state == before                       # pose did not change
    assert len(c.driver.command_log) == n_cmds     # nothing sent to the driver


def test_jog_past_joint_limit_is_rejected():
    c = make()
    c.enable()
    c.go_home()
    res = c.jog_joint("left", 500.0)
    assert not res.ok


def test_bad_joint_and_axis_raise():
    c = make()
    with pytest.raises(ValueError):
        c.jog_joint("middle", 1.0)
    with pytest.raises(ValueError):
        c.jog_cartesian("z", 1.0)


def test_disable_blocks_jog_again():
    c = make()
    c.enable()
    c.go_home()
    c.disable()
    res = c.jog_joint("left", 1.0)
    assert not res.ok and "disabled" in res.reason
