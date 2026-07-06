"""Closing the loop: planner, the §11 pick/place handshake, and CycleManager."""

import numpy as np
import pytest

from bung_cover_robot.app.cycle_manager import (
    CycleManager,
    DryRunJobRunner,
    HandshakeJobRunner,
    make_job_runner,
)
from bung_cover_robot.app.robot_test_controller import (
    RobotTestController,
    build_dry_run_controller,
)
from bung_cover_robot.gui.imaging import demo_frame, demo_transform
from bung_cover_robot.plc import PlcError, PlcRobotDriver, SimulatedPlcClient, tags
from bung_cover_robot.plc.compactlogix_client import PlcClient
from bung_cover_robot.plc.handshake import PickPlaceHandshake
from bung_cover_robot.robot.driver import DryRunRobotDriver
from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics
from bung_cover_robot.robot.planner import (
    PlanningError,
    make_job,
    sort_holes_along_conveyor,
)
from bung_cover_robot.robot.workspace import WorkspaceValidator
from bung_cover_robot.vision.camera import CameraConfig, MockCamera


def _mock_camera():
    return MockCamera(
        CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
    ).open()


# --------------------------------------------------------------------------- #
# planner
# --------------------------------------------------------------------------- #
def test_make_job_builds_validated_targets():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    job = make_job(kin, val, hole_index=2, cover_id=5, pick_xy=(80, 250), drop_xy=(-40, 250))
    assert job.hole_index == 2 and job.cover_id == 5
    assert job.pick_xy == (80, 250) and job.drop_xy == (-40, 250)
    # angles are the IK solution for each pose
    assert job.pick.left_deg == pytest.approx(kin.inverse(80, 250).left_deg)


def test_make_job_rejects_unreachable_target():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    with pytest.raises(PlanningError):
        make_job(kin, val, hole_index=0, cover_id=0, pick_xy=(0, 250), drop_xy=(0, 9999))


def test_sort_holes_along_conveyor():
    holes = [(50, 250), (-100, 250), (10, 250)]
    assert sort_holes_along_conveyor(holes) == [1, 2, 0]


# --------------------------------------------------------------------------- #
# handshake against the simulator
# --------------------------------------------------------------------------- #
def _homed_sim():
    sim = SimulatedPlcClient(home_angles=(135.0, 45.0)).connect()
    sim.write(tags.Manual.ENABLE, True)
    sim.write(tags.Manual.HOME_REQUEST, True)  # sets Enabled + Homed
    return sim


def test_handshake_completes_job():
    sim = _homed_sim()
    hs = PickPlaceHandshake(sim, command_timeout_s=2.0)
    res = hs.send_job_and_wait((120.0, 60.0), (130.0, 50.0), hole_index=1, cover_id=3)
    assert res.ok and res.command_id == 1
    assert sim.read(tags.Status.COMPLETE_COMMAND_ID) == 1
    # ends at the drop pose
    assert sim.read(tags.Status.ACTUAL_LEFT_DEG) == 130.0


def test_handshake_faults_when_not_homed():
    sim = SimulatedPlcClient().connect()
    sim.write(tags.Manual.ENABLE, True)  # enabled but never homed
    hs = PickPlaceHandshake(sim, command_timeout_s=2.0)
    res = hs.send_job_and_wait((120.0, 60.0), (130.0, 50.0), hole_index=0, cover_id=0)
    assert not res.ok and "fault" in res.reason.lower()


class _StuckClient(PlcClient):
    """A PLC that is Ready but never completes a job — exercises the timeout."""

    def __init__(self):
        self._store = {tags.Status.READY: True}
        self.aborted = False

    def connect(self):
        return self

    def close(self):
        pass

    @property
    def is_connected(self):
        return True

    def read(self, tag):
        return self._store.get(tag, 0)

    def write(self, tag, value):
        self._store[tag] = value
        if tag == tags.Cmd.ABORT and value:
            self.aborted = True


def test_handshake_times_out_and_recovers():
    client = _StuckClient()
    hs = PickPlaceHandshake(client, command_timeout_s=0.05, poll_interval_s=0.01)
    res = hs.send_job_and_wait((1, 2), (3, 4), hole_index=0, cover_id=0)
    assert not res.ok and "timed out" in res.reason
    assert client.aborted  # recovery pulsed Cmd.Abort instead of hanging


# --------------------------------------------------------------------------- #
# job runner selection
# --------------------------------------------------------------------------- #
def test_make_job_runner_picks_by_driver():
    assert isinstance(make_job_runner(DryRunRobotDriver()), DryRunJobRunner)
    sim = SimulatedPlcClient().connect()
    assert isinstance(make_job_runner(PlcRobotDriver(sim)), HandshakeJobRunner)


# --------------------------------------------------------------------------- #
# CycleManager
# --------------------------------------------------------------------------- #
def _ready_controller():
    ctrl = build_dry_run_controller()
    ctrl.enable()
    ctrl.home_reference()
    return ctrl


def test_cycle_preflight_blocks():
    ctrl = build_dry_run_controller()
    mgr = CycleManager(ctrl, _mock_camera(), demo_transform())
    assert "disabled" in mgr.preflight()
    ctrl.enable()
    assert "referenced" in mgr.preflight()
    ctrl.home_reference()
    assert mgr.preflight() is None
    # no calibration is also a block
    assert "calibration" in CycleManager(ctrl, _mock_camera(), None).preflight()


def test_cycle_places_reachable_covers_then_stops():
    mgr = CycleManager(_ready_controller(), _mock_camera(), demo_transform())
    res = mgr.run_cycle()
    assert res.ok
    assert len(res.placed) == 3           # 3 reachable covers in the demo scene
    assert "out of reachable covers" in res.reason
    # a picked cover is never re-picked (distinct positions)
    picks = [s.pick_xy for s in res.placed]
    assert len(picks) == len({(round(p[0]), round(p[1])) for p in picks})


def test_cycle_stop_requested_halts_early():
    mgr = CycleManager(_ready_controller(), _mock_camera(), demo_transform())
    res = mgr.run_cycle(should_stop=lambda: True)
    assert res.steps == [] and "stopped" in res.reason.lower()


def test_cycle_over_plc_handshake():
    kin = FiveBarKinematics()
    jt = kin.inverse(0.0, 250.0)
    client = SimulatedPlcClient(home_angles=(jt.left_deg, jt.right_deg)).connect()
    ctrl = RobotTestController(PlcRobotDriver(client), kin)
    ctrl.enable()
    ctrl.home_reference()
    mgr = CycleManager(ctrl, _mock_camera(), demo_transform())
    res = mgr.run_cycle()
    assert res.ok and len(res.placed) == 3
    assert isinstance(mgr.job_runner or make_job_runner(ctrl.driver), HandshakeJobRunner)
    # the last job really ran through the handshake
    assert client.read(tags.Status.COMPLETE_COMMAND_ID) == 3


def test_cycle_reports_when_no_holes():
    blank = np.full((520, 760, 3), (30, 27, 25), np.uint8)
    cam = MockCamera(CameraConfig(mock_width=760, mock_height=520), frames=[blank]).open()
    mgr = CycleManager(_ready_controller(), cam, demo_transform())
    res = mgr.run_cycle()
    assert not res.ok and "hole" in res.reason


# --------------------------------------------------------------------------- #
# Vision bypass — scripted targets
# --------------------------------------------------------------------------- #
def test_default_scripted_targets_are_reachable():
    from bung_cover_robot.app.cycle_manager import default_scripted_targets

    ctrl = _ready_controller()
    holes, covers = default_scripted_targets(ctrl, hole_count=6)
    assert len(holes) == 6 and len(covers) >= 6
    assert all(ctrl.validator.validate(x, y).ok for x, y in holes + covers)


def test_scripted_cycle_needs_no_calibration_or_camera():
    from bung_cover_robot.app.cycle_manager import (
        ScriptedTargetSource, default_scripted_targets)

    ctrl = _ready_controller()
    holes, covers = default_scripted_targets(ctrl)
    # calibration=None and the camera is never grabbed in bypass mode.
    mgr = CycleManager(ctrl, _mock_camera(), None,
                       target_source=ScriptedTargetSource(holes, covers))
    assert mgr.preflight() is None           # vision preflight is skipped
    res = mgr.run_cycle()
    assert res.ok and len(res.placed) == len(holes)   # every hole filled


def test_scripted_cycle_over_plc_handshake():
    from bung_cover_robot.app.cycle_manager import (
        ScriptedTargetSource, default_scripted_targets)

    kin = FiveBarKinematics()
    jt = kin.inverse(0.0, 250.0)
    client = SimulatedPlcClient(home_angles=(jt.left_deg, jt.right_deg)).connect()
    ctrl = RobotTestController(PlcRobotDriver(client), kin)
    ctrl.enable()
    ctrl.home_reference()
    holes, covers = default_scripted_targets(ctrl)
    mgr = CycleManager(ctrl, _mock_camera(), None,
                       target_source=ScriptedTargetSource(holes, covers))
    res = mgr.run_cycle()
    assert res.ok and len(res.placed) == len(holes)
    # every scripted job really ran through the §11 handshake
    assert client.read(tags.Status.COMPLETE_COMMAND_ID) == len(holes)


def test_scripted_cycle_still_requires_referenced():
    from bung_cover_robot.app.cycle_manager import (
        ScriptedTargetSource, default_scripted_targets)

    ctrl = build_dry_run_controller()          # not enabled/homed
    holes, covers = default_scripted_targets(ctrl)
    mgr = CycleManager(ctrl, _mock_camera(), None,
                       target_source=ScriptedTargetSource(holes, covers))
    assert "disabled" in mgr.preflight()       # motion guard still applies


def test_run_single_job():
    mgr = CycleManager(_ready_controller(), _mock_camera(), None,
                       target_source=None)     # source irrelevant for a single job
    step = mgr.run_single_job(pick_xy=(60.0, 205.0), drop_xy=(-60.0, 250.0))
    assert step.ok and step.pick_xy == (60.0, 205.0)
    # an unreachable target is rejected by the workspace guard, not sent
    bad = mgr.run_single_job(pick_xy=(0.0, 250.0), drop_xy=(0.0, 9999.0))
    assert not bad.ok
