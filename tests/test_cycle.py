"""Closing the loop: planner and CycleManager (dry-run direct driver)."""

import numpy as np
import pytest

from bung_cover_robot.app.cycle_manager import (
    CycleManager,
    DirectJobRunner,
    make_job_runner,
)
from bung_cover_robot.app.robot_test_controller import (
    build_dry_run_controller,
)
from bung_cover_robot.gui.imaging import demo_frame, demo_transform
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
# job runner selection
# --------------------------------------------------------------------------- #
def test_make_job_runner_picks_by_driver():
    assert isinstance(make_job_runner(DryRunRobotDriver()), DirectJobRunner)


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


def test_cycle_emits_a_frame_on_each_capture():
    mgr = CycleManager(_ready_controller(), _mock_camera(), demo_transform())
    frames = []
    res = mgr.run_cycle(on_frame=frames.append)
    assert res.ok
    # imaged for the holes once + once per pick attempt (>= number placed)
    assert len(frames) >= 1 + len(res.placed)
    assert all(f is not None for f in frames)
    # the source exposes the last frame it grabbed for the live view
    assert mgr.target_source.last_frame is not None


def test_cycle_single_step_stops_after_one_hole():
    from bung_cover_robot.app.cycle_manager import (
        CycleConfig,
        ScriptedTargetSource,
        default_scripted_targets,
    )

    ctrl = _ready_controller()
    holes, covers = default_scripted_targets(ctrl)
    mgr = CycleManager(
        ctrl, _mock_camera(), demo_transform(),
        target_source=ScriptedTargetSource(holes, covers),
        config=CycleConfig(max_holes=1),   # hardware-shakeout single step
    )
    res = mgr.run_cycle()
    assert len(res.steps) == 1             # exactly one pick/place attempted


def test_cycle_stop_requested_halts_early():
    mgr = CycleManager(_ready_controller(), _mock_camera(), demo_transform())
    res = mgr.run_cycle(should_stop=lambda: True)
    assert res.steps == [] and "stopped" in res.reason.lower()


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
