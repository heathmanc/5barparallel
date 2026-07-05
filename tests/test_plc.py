"""PLC layer: tags, the reactive simulator, and PlcRobotDriver end-to-end."""

import pytest

from bung_cover_robot.app.robot_test_controller import (
    DEFAULT_HOME_XY,
    RobotTestController,
)
from bung_cover_robot.plc import (
    PlcRobotDriver,
    SimulatedPlcClient,
    tags,
)
from bung_cover_robot.robot import RobotDriverError
from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics


# --------------------------------------------------------------------------- #
# Tags
# --------------------------------------------------------------------------- #
def test_tags_are_namespaced_and_complete():
    assert tags.Manual.ENABLE == "VisionRobot.Manual.Enable"
    assert tags.Status.COMPLETE_COMMAND_ID == "VisionRobot.Status.CompleteCommandID"
    all_tags = tags.all_tags()
    assert tags.Manual.MOVE_TO_TARGET in all_tags
    assert len(all_tags) == len(set(all_tags))  # no duplicates


def test_tag_specs_cover_every_tag_with_valid_metadata():
    # Every tag the code uses is documented, and vice versa.
    class_names = set()
    for group in (tags.Cmd, tags.Target, tags.Manual, tags.Status):
        for name, value in vars(group).items():
            if not name.startswith("_") and isinstance(value, str):
                class_names.add(value)
    spec_names = {s.name for s in tags.TAG_SPECS}
    assert spec_names == class_names
    assert len(spec_names) == len(tags.TAG_SPECS)  # no duplicate specs
    for s in tags.TAG_SPECS:
        assert s.dtype in ("BOOL", "DINT", "REAL")
        assert s.direction in (tags.PC_TO_PLC, tags.PLC_TO_PC)
        assert s.description.strip()


def test_tag_table_csv_has_header_and_all_rows():
    csv = tags.tag_table_csv()
    lines = csv.strip().splitlines()
    assert lines[0] == "Tag,Type,Direction,Group,Description"
    assert len(lines) == len(tags.TAG_SPECS) + 1


# --------------------------------------------------------------------------- #
# Simulated PLC
# --------------------------------------------------------------------------- #
def test_sim_requires_connect():
    from bung_cover_robot.plc import PlcError

    sim = SimulatedPlcClient()
    with pytest.raises(PlcError):
        sim.read(tags.Status.ENABLED)


def test_sim_enable_and_move_handshake():
    sim = SimulatedPlcClient(home_angles=(135.0, 45.0)).connect()
    assert sim.read(tags.Status.ENABLED) == 0
    sim.write(tags.Manual.ENABLE, True)
    assert sim.read(tags.Status.ENABLED) is True

    sim.write(tags.Manual.TARGET_LEFT_DEG, 100.0)
    sim.write(tags.Manual.TARGET_RIGHT_DEG, 80.0)
    sim.write(tags.Manual.COMMAND_ID, 7)
    sim.write(tags.Manual.MOVE_TO_TARGET, True)
    assert sim.read(tags.Status.COMPLETE_COMMAND_ID) == 7
    assert sim.read(tags.Status.IN_POSITION) is True
    assert sim.read(tags.Status.ACTUAL_LEFT_DEG) == 100.0


def test_sim_move_while_disabled_faults():
    sim = SimulatedPlcClient().connect()
    sim.write(tags.Manual.MOVE_TO_TARGET, True)
    assert sim.read(tags.Status.FAULTED) is True


# --------------------------------------------------------------------------- #
# PlcRobotDriver
# --------------------------------------------------------------------------- #
def make_driver(home=(135.0, 45.0)) -> PlcRobotDriver:
    client = SimulatedPlcClient(home_angles=home).connect()
    return PlcRobotDriver(client, command_timeout_s=2.0)


def test_driver_enable_home_move_readback():
    d = make_driver(home=(130.0, 50.0))
    assert not d.is_enabled
    d.enable()
    assert d.is_enabled
    d.home()
    assert d.read_angles() == (130.0, 50.0)
    d.move_to_angles(120.0, 60.0)
    assert d.read_angles() == (120.0, 60.0)


def test_driver_move_requires_enable():
    d = make_driver()
    with pytest.raises(RobotDriverError):
        d.move_to_angles(120.0, 60.0)


def test_driver_read_angles_none_until_homed():
    d = make_driver()
    d.enable()
    assert d.read_angles() is None  # not homed yet


# --------------------------------------------------------------------------- #
# PlcRobotDriver behind the controller (the real-hardware path, simulated)
# --------------------------------------------------------------------------- #
def test_controller_over_plc_driver():
    kin = FiveBarKinematics()
    jt = kin.inverse(*DEFAULT_HOME_XY)
    client = SimulatedPlcClient(home_angles=(jt.left_deg, jt.right_deg)).connect()
    controller = RobotTestController(PlcRobotDriver(client), kin)

    controller.enable()
    assert controller.home_reference().ok
    assert controller.is_referenced

    res = controller.jog_cartesian("x", 10.0)
    assert res.ok
    # The commanded angles reached the (simulated) PLC.
    assert client.read(tags.Status.ACTUAL_LEFT_DEG) == pytest.approx(res.state.left_deg)
    assert client.read(tags.Status.ACTUAL_RIGHT_DEG) == pytest.approx(res.state.right_deg)
