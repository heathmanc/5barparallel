"""RobotTestController: reference (find home switches) + teach/go-home + jog,
all gated by the workspace validator."""

import pytest

from bung_cover_robot.app.robot_test_controller import (
    DEFAULT_HOME_XY,
    RobotTestController,
    build_dry_run_controller,
)
from bung_cover_robot.robot import DryRunRobotDriver


def make() -> RobotTestController:
    # Dry-run driver referenced to a valid home pose.
    return build_dry_run_controller()


def ready() -> RobotTestController:
    c = make()
    c.enable()
    c.home_reference()
    return c


def test_disable_clears_reference_and_forces_rehome():
    c = ready()
    assert c.is_referenced
    c.disable()
    assert not c.is_referenced                 # open-loop: disable loses the datum
    c.enable()                                 # re-enable alone does NOT restore it
    assert not c.is_referenced
    res = c.jog_joint("left", 1.0)             # motion refused until re-homed
    assert not res.ok and "referenced" in res.reason
    c.home_reference()
    assert c.is_referenced


# --------------------------------------------------------------------------- #
# Initial state
# --------------------------------------------------------------------------- #
def test_initial_state_is_valid_home_not_enabled_not_referenced():
    c = make()
    assert c.state.tcp == DEFAULT_HOME_XY
    assert not c.is_enabled
    assert not c.is_referenced
    assert c.state.metrics["reach_fraction"] < 0.85


def test_invalid_home_rejected_at_construction():
    with pytest.raises(ValueError):
        RobotTestController(DryRunRobotDriver(), home_xy=(0.0, 900.0))


# --------------------------------------------------------------------------- #
# Referencing (hardware home)
# --------------------------------------------------------------------------- #
def test_reference_requires_enable():
    c = make()
    res = c.home_reference()
    assert not res.ok and "disabled" in res.reason
    assert not c.is_referenced


def test_reference_adopts_driver_reported_pose():
    c = make()
    c.enable()
    res = c.home_reference()
    assert res.ok
    assert c.is_referenced
    assert c.driver.read_angles() == (c.state.left_deg, c.state.right_deg)


# --------------------------------------------------------------------------- #
# Motion gating
# --------------------------------------------------------------------------- #
def test_jog_blocked_until_enabled():
    c = make()
    res = c.jog_joint("left", 1.0)
    assert not res.ok and "disabled" in res.reason


def test_jog_blocked_until_referenced():
    c = make()
    c.enable()
    res = c.jog_joint("left", 1.0)
    assert not res.ok and "referenced" in res.reason


def test_go_home_blocked_until_referenced():
    c = make()
    c.enable()
    res = c.go_home()
    assert not res.ok and "referenced" in res.reason


# --------------------------------------------------------------------------- #
# Home (teach + go)
# --------------------------------------------------------------------------- #
def test_go_home_commands_driver():
    c = ready()
    res = c.go_home()
    assert res.ok
    assert c.driver.read_angles() == (c.state.left_deg, c.state.right_deg)


def test_set_home_teaches_current_pose():
    c = ready()
    c.jog_cartesian("y", 10.0)  # move to ~(0, 260)
    taught = c.set_home()
    assert taught == c.state.tcp
    assert c.state.tcp == pytest.approx((0.0, 260.0), abs=2e-3)
    c.jog_cartesian("x", 20.0)
    c.go_home()
    assert c.state.tcp == pytest.approx((0.0, 260.0), abs=2e-3)


# --------------------------------------------------------------------------- #
# Jog
# --------------------------------------------------------------------------- #
def test_jog_joint_moves_one_shoulder():
    c = ready()
    before = c.state
    res = c.jog_joint("left", 2.0)
    assert res.ok
    assert c.state.left_deg == pytest.approx(before.left_deg + 2.0)
    assert c.state.right_deg == pytest.approx(before.right_deg)


def test_jog_cartesian_moves_tcp():
    c = ready()
    res = c.jog_cartesian("x", 15.0)
    assert res.ok
    assert c.state.tcp == pytest.approx((15.0, 250.0), abs=2e-3)


def test_jog_out_of_workspace_is_rejected_and_pose_unchanged():
    c = ready()
    before = c.state
    n_cmds = len(c.driver.command_log)
    res = c.move_to_xy(0.0, 430.0)  # past the 85% reach cap
    assert not res.ok
    assert "reach" in res.reason
    assert c.state == before
    assert len(c.driver.command_log) == n_cmds  # nothing sent to the driver


def test_jog_past_joint_limit_is_rejected():
    c = ready()
    res = c.jog_joint("left", 500.0)
    assert not res.ok


def test_bad_joint_and_axis_raise():
    c = make()
    with pytest.raises(ValueError):
        c.jog_joint("middle", 1.0)
    with pytest.raises(ValueError):
        c.jog_cartesian("z", 1.0)


def test_disable_blocks_jog_again():
    c = ready()
    c.disable()
    res = c.jog_joint("left", 1.0)
    assert not res.ok and "disabled" in res.reason


# --------------------------------------------------------------------------- #
# Geometry swap (Settings tab)
# --------------------------------------------------------------------------- #
def test_update_geometry_reresolves_pose():
    from bung_cover_robot.robot.fivebar_kinematics import FiveBarConfig

    c = ready()
    c.update_geometry(FiveBarConfig(l1_mm=225.0))  # small change, still valid
    assert c.kin.config.l1_mm == 225.0
    # Current pose remains resolved/valid.
    assert 0.0 < c.state.metrics["reach_fraction"] <= 0.85


# --------------------------------------------------------------------------- #
# Homing config wiring
# --------------------------------------------------------------------------- #
def test_set_driver_swaps_and_requires_rehome():
    c = ready()  # dry-run, enabled + referenced
    assert c.is_referenced and c.is_enabled
    old = c.driver
    new = DryRunRobotDriver(home_angles=old.read_angles())
    c.set_driver(new)
    assert c.driver is new
    assert not c.is_referenced   # fresh driver -> must re-home
    assert not c.is_enabled      # new dry-run starts disabled
    assert not old.is_enabled    # old driver was disabled on swap


def test_factory_uses_configured_home_reference():
    from bung_cover_robot.robot import HomingConfig

    homing = HomingConfig()  # verified reference
    c = build_dry_run_controller(homing=homing)
    assert c.home_xy == homing.home_tcp_mm
    c.enable()
    c.home_reference()
    # Driver reports the configured home angles; controller adopts them.
    assert c.driver.read_angles() == pytest.approx(homing.home_angles)
    assert c.state.tcp == pytest.approx(homing.home_tcp_mm, abs=1e-3)


# --------------------------------------------------------------------------- #
# Reset + fault surfacing (a driver fault becomes a MoveResult, never an escape)
# --------------------------------------------------------------------------- #
def test_reset_returns_ok_on_dry_run():
    c = make()
    assert c.reset().ok
    assert not c.is_faulted and c.fault_code() is None


def test_home_reference_reports_driver_fault_as_reject():
    from bung_cover_robot.robot.driver import DryRunRobotDriver, RobotDriverError

    class FaultingDriver(DryRunRobotDriver):
        def home(self):
            raise RobotDriverError("home / find reference: PLC faulted (code 4)")

    c = RobotTestController(FaultingDriver())
    c.enable()
    res = c.home_reference()
    assert not res.ok
    assert "code 4" in res.reason        # message surfaced, not an exception
    assert not c.is_referenced           # stays unreferenced after a failed home


def test_enable_reports_driver_fault_as_reject():
    from bung_cover_robot.robot.driver import DryRunRobotDriver, RobotDriverError

    class FaultingDriver(DryRunRobotDriver):
        def enable(self):
            raise RobotDriverError("PLC is faulted (code 4); reset before enabling")

    c = RobotTestController(FaultingDriver())
    res = c.enable()
    assert not res.ok and "reset before enabling" in res.reason
