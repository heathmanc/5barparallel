"""DryRunRobotDriver behavior."""

import pytest

from bung_cover_robot.robot import DryRunRobotDriver, RobotDriverError


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
