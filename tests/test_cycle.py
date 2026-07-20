"""Closing the loop: planner and CycleManager (dry-run direct driver)."""

import numpy as np
import pytest

import random

from bung_cover_robot.app.cycle_manager import (
    CycleManager,
    DirectJobRunner,
    JobResult,
    PickSequence,
    _run_with_reenable,
    demo_pick_and_place_targets,
    make_job_runner,
    run_demo_cycle,
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


# --------------------------------------------------------------------------- #
# sample pick&place demo (vision bypass) — fixed nest + variable cover row
# --------------------------------------------------------------------------- #
def test_demo_targets_are_reachable_and_six_wide():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    home_xy = (0.0, 250.0)
    # A range of seeds must all yield a valid nest + 6 in-workspace holes.
    for seed in range(20):
        nest, drops = demo_pick_and_place_targets(
            val, home_xy, rng=random.Random(seed))
        assert val.validate(*nest).ok
        assert len(drops) == 6
        assert all(val.validate(x, y).ok for x, y in drops)


def test_demo_targets_vary_between_runs():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    a = demo_pick_and_place_targets(val, (0.0, 250.0), rng=random.Random(1))[1]
    b = demo_pick_and_place_targets(val, (0.0, 250.0), rng=random.Random(2))[1]
    assert a != b                              # the cover row is variably placed


def test_run_demo_cycle_places_all_holes_and_actuates_head():
    ctrl = _ready_controller()
    nest, drops = demo_pick_and_place_targets(
        ctrl.validator, ctrl.home_xy, rng=random.Random(0))
    steps = []
    res = run_demo_cycle(ctrl, nest, drops,
                         pick_sequence=PickSequence(0, 0, 0),
                         on_step=steps.append)
    assert res.ok
    assert len(res.placed) == len(drops)       # every hole filled
    assert len(steps) == len(drops)
    # the dry-run driver logged one move per pick + per drop (2 per hole)
    assert len(ctrl.driver.command_log) == 2 * len(drops)
    # head left safe at the end (last release retracts the plunger, vents vacuum)
    assert ctrl.driver.plunger_extended is False
    assert ctrl.driver.vacuum_on is False


def test_run_demo_cycle_preflight_guards():
    ctrl = build_dry_run_controller()          # not enabled/referenced
    res = run_demo_cycle(ctrl, (0.0, 205.0), [(0.0, 250.0)])
    assert not res.ok and "disabled" in res.reason
    ctrl.enable()
    res = run_demo_cycle(ctrl, (0.0, 205.0), [(0.0, 250.0)])
    assert not res.ok and "referenced" in res.reason


def test_run_demo_cycle_stops_on_request():
    ctrl = _ready_controller()
    nest, drops = demo_pick_and_place_targets(
        ctrl.validator, ctrl.home_xy, rng=random.Random(3))
    seen = []

    def stop_after_two():
        return len(seen) >= 2

    res = run_demo_cycle(ctrl, nest, drops,
                         pick_sequence=PickSequence(0, 0, 0),
                         should_stop=stop_after_two,
                         on_step=seen.append)
    assert "stopped" in res.reason
    assert len(res.steps) == 2                  # halted early


def test_run_demo_cycle_caps_travel_speed():
    """The demo throttles the travel moves (a big move at full speed can trip an
    excessive-position-deviation alarm on a real servo)."""
    from bung_cover_robot.app.cycle_manager import DEMO_MOVE_SPEED_MM_S

    class _SpeedRec(DryRunRobotDriver):
        def __init__(self):
            super().__init__(home_angles=(140.5406, 39.4594))
            self.speeds = []

        def move_to_angles(self, left_deg, right_deg, speed_mm_s=None):
            super().move_to_angles(left_deg, right_deg, speed_mm_s)
            self.speeds.append(speed_mm_s)

    from bung_cover_robot.app.robot_test_controller import RobotTestController
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    drv = _SpeedRec()
    ctrl = RobotTestController(drv, kin, val)
    ctrl.enable()
    ctrl.home_reference()
    nest, drops = demo_pick_and_place_targets(val, ctrl.home_xy, rng=random.Random(0))
    # default: the gentle DEMO_MOVE_SPEED_MM_S is applied to every travel move
    run_demo_cycle(ctrl, nest, drops, pick_sequence=PickSequence(0, 0, 0))
    assert drv.speeds and all(s == DEMO_MOVE_SPEED_MM_S for s in drv.speeds)
    # explicit override is honoured
    drv.speeds.clear()
    run_demo_cycle(ctrl, nest, drops, pick_sequence=PickSequence(0, 0, 0),
                   move_speed_mm_s=25.0)
    assert all(s == 25.0 for s in drv.speeds)


# --------------------------------------------------------------------------- #
# retry-with-backoff re-enable (transient SWITCH ON DISABLED, no fault)
# --------------------------------------------------------------------------- #
class _FakeDriver:
    """Minimal stand-in for the re-enable path: counts enable() calls and can be
    faulted. ``is_faulted`` gates whether a drop is transient or a real fault."""

    def __init__(self, is_faulted=False, enable_raises=False):
        self.is_faulted = is_faulted
        self.enable_raises = enable_raises
        self.enables = 0

    def enable(self):
        self.enables += 1
        if self.enable_raises:
            raise RuntimeError("contactor open")


def _flaky_runner(fail_count, reason="cannot move: drives are disabled"):
    """A job that fails with ``reason`` its first ``fail_count`` calls, then oks."""
    state = {"n": 0}

    def run():
        state["n"] += 1
        if state["n"] <= fail_count:
            return JobResult(False, reason)
        return JobResult(True, "ok")

    return run


def test_reenable_recovers_after_a_few_transient_drops():
    drv = _FakeDriver()
    res, n = _run_with_reenable(drv, _flaky_runner(2), base_delay=0.0)
    assert res.ok                                 # a later attempt landed clean
    assert n == 2 and drv.enables == 2            # re-armed exactly twice


def test_reenable_leaves_a_real_fault_untouched():
    drv = _FakeDriver(is_faulted=True)
    res, n = _run_with_reenable(drv, _flaky_runner(2), base_delay=0.0)
    assert not res.ok and n == 0                  # a fault must not be papered over
    assert drv.enables == 0


def test_reenable_gives_up_after_exhausting_attempts():
    drv = _FakeDriver()
    res, n = _run_with_reenable(drv, _flaky_runner(99), attempts=3, base_delay=0.0)
    assert not res.ok and "disabled" in res.reason
    assert n == 3 and drv.enables == 3            # tried the cap, then aborted


def test_reenable_ignores_non_disable_failures():
    drv = _FakeDriver()
    res, n = _run_with_reenable(
        drv, _flaky_runner(2, reason="drive 0: FAULT Er.87.1"), base_delay=0.0)
    assert not res.ok and n == 0                  # not our transient — no retry
    assert drv.enables == 0


def test_reenable_honours_a_stop_request():
    drv = _FakeDriver()
    res, n = _run_with_reenable(
        drv, _flaky_runner(99), should_stop=lambda: True, base_delay=0.0)
    assert not res.ok and n == 0                  # operator stop wins over retry


def test_reenable_reports_a_failed_re_arm():
    drv = _FakeDriver(enable_raises=True)
    res, n = _run_with_reenable(drv, _flaky_runner(99), attempts=2, base_delay=0.0)
    assert not res.ok and "re-enable failed" in res.reason
    assert n == 0 and drv.enables == 2            # attempted, but never re-armed


def test_run_demo_cycle_survives_a_transient_disable():
    """A drop to SWITCH ON DISABLED mid-run (no fault) is retried, and the run
    completes with a note in the reason so the operator knows it happened."""
    ctrl = _ready_controller()
    nest, drops = demo_pick_and_place_targets(
        ctrl.validator, ctrl.home_xy, rng=random.Random(0))

    class _BlipRunner:
        def __init__(self):
            self.calls = 0

        def run(self, job):
            self.calls += 1
            if self.calls == 2:                   # blip on the second job only
                return JobResult(False, "cannot move: drives are disabled")
            return JobResult(True, "ok")

    res = run_demo_cycle(ctrl, nest, drops, runner=_BlipRunner(),
                         reenable_delay_s=0.0)
    assert res.ok
    assert len(res.placed) == len(drops)          # the blip didn't cost a cover
    assert "auto re-enabled" in res.reason


# --------------------------------------------------------------------------- #
# battery layout (35 mm pitch) + rolling throughput
# --------------------------------------------------------------------------- #
def _dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def test_demo_row_is_six_holes_35mm_apart_in_a_line():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    for seed in range(20):
        _nest, drops = demo_pick_and_place_targets(
            val, (0.0, 250.0), rng=random.Random(seed))
        assert len(drops) == 6
        gaps = [_dist(drops[i], drops[i + 1]) for i in range(5)]
        assert all(abs(g - 35.0) < 0.3 for g in gaps)     # fixed 35 mm pitch
        # collinear: every gap along the same straight line (equal end-to-end sum)
        assert abs(_dist(drops[0], drops[-1]) - sum(gaps)) < 0.3


def test_demo_pick_nest_is_fixed_across_runs():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    nests = {demo_pick_and_place_targets(val, (0.0, 250.0), rng=random.Random(s))[0]
             for s in range(10)}
    assert len(nests) == 1                                # same pick every time


def test_cycle_rate_tracker_per_minute():
    from bung_cover_robot.app.cycle_manager import CycleRateTracker

    tr = CycleRateTracker(window_s=60.0)
    assert tr.per_minute(0.0) == 0.0                      # nothing yet
    for t in (0.0, 2.0, 4.0, 6.0):
        tr.record(t)
    assert tr.total == 4
    assert tr.per_minute(6.0) == pytest.approx(40.0)      # 4 cycles over 6 s -> 40/min


def test_cycle_rate_tracker_uses_trailing_window():
    from bung_cover_robot.app.cycle_manager import CycleRateTracker

    tr = CycleRateTracker(window_s=3.0)
    for t in (0.0, 1.0, 2.0, 3.0, 4.0):                   # 5 cycles, 1 s apart
        tr.record(t)
    # at t=4 the window is [1,2,3,4] (t=0 is older than 3 s): 4 cycles / 3 s
    assert tr.total == 5
    assert tr.per_minute(4.0) == pytest.approx(80.0)
