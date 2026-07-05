"""DryRunRobotDriver behavior + HomingConfig loading."""

from pathlib import Path

import pytest

from bung_cover_robot.robot import DryRunRobotDriver, HomingConfig, RobotDriverError
from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics

CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot_config.yaml"


def test_starts_disabled_and_position_unknown():
    d = DryRunRobotDriver()
    assert not d.is_enabled
    assert d.read_angles() is None


def test_move_requires_enable():
    d = DryRunRobotDriver()
    with pytest.raises(RobotDriverError):
        d.move_to_angles(90.0, 90.0)


def test_move_records_and_reads_back():
    d = DryRunRobotDriver()
    d.enable()
    d.move_to_angles(120.0, 60.0)
    assert d.read_angles() == (120.0, 60.0)
    assert d.command_log == [(120.0, 60.0)]


def test_home_uses_home_angles():
    d = DryRunRobotDriver(home_angles=(135.0, 45.0))
    d.enable()
    d.home()
    assert d.read_angles() == (135.0, 45.0)


def test_home_requires_enable():
    d = DryRunRobotDriver()
    with pytest.raises(RobotDriverError):
        d.home()


def test_disable_blocks_further_motion():
    d = DryRunRobotDriver()
    d.enable()
    d.move_to_angles(90.0, 90.0)
    d.disable()
    with pytest.raises(RobotDriverError):
        d.move_to_angles(91.0, 90.0)


# --------------------------------------------------------------------------- #
# HomingConfig
# --------------------------------------------------------------------------- #
def test_homing_config_defaults():
    h = HomingConfig()
    assert h.home_angles == (135.8504, 44.1496)
    assert h.flag_radius_mm == 40.0
    assert h.limit_min_deg == -20.0 and h.limit_max_deg == 200.0
    assert h.home_tcp_mm == (0.0, 250.0)


def test_homing_config_from_yaml():
    h = HomingConfig.from_yaml(CONFIG)
    assert h.home_left_deg == pytest.approx(135.8504)
    assert h.home_right_deg == pytest.approx(44.1496)
    assert h.flag_radius_mm == 40.0
    assert h.home_tcp_mm == (0.0, 250.0)


def test_homing_angles_match_home_tcp_under_verified_geometry():
    # The stored home angles must be the IK of the stored home TCP.
    h = HomingConfig.from_yaml(CONFIG)
    jt = FiveBarKinematics().inverse(*h.home_tcp_mm)
    assert jt.left_deg == pytest.approx(h.home_left_deg, abs=1e-3)
    assert jt.right_deg == pytest.approx(h.home_right_deg, abs=1e-3)
