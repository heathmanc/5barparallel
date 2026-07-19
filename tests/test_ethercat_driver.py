"""EtherCatRobotDriver against the simulated A6 network — CiA 402 arming, homing,
CSP moves, fault/reset, and a full cycle through CycleManager."""

import pytest

from bung_cover_robot.app.cycle_manager import (
    CycleManager,
    DirectJobRunner,
    PickSequence,
    make_job_runner,
)
from bung_cover_robot.app.robot_test_controller import RobotTestController
from bung_cover_robot.ethercat import (
    EtherCatRobotDriver,
    SimulatedEtherCatMaster,
    cia402,
)
from bung_cover_robot.gui.imaging import demo_frame, demo_transform
from bung_cover_robot.robot.driver import DryRunRobotDriver, RobotDriverError
from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics
from bung_cover_robot.robot.workspace import WorkspaceValidator
from bung_cover_robot.vision.camera import CameraConfig, MockCamera


def _driver(home=(140.5406, 39.4594)):
    kin = FiveBarKinematics()
    master = SimulatedEtherCatMaster().open()
    drv = EtherCatRobotDriver(master, kin, WorkspaceValidator(kin), home_angles=home)
    return drv, master, kin


def test_jog_counts_single_axis_ramps_to_target():
    # Single-drive bench: enable, then jog axis 0 by a raw count delta. The
    # simulated CSP axis follows the streamed ramp exactly.
    master = SimulatedEtherCatMaster(num_drives=1).open()
    drv = EtherCatRobotDriver(master).connect()
    drv.enable()
    assert drv.is_enabled
    drv.jog_counts(0, 3000, speed_counts_s=50000, accel_counts_s2=200000)
    assert master.drives[0].actual_position == 3000
    drv.jog_counts(0, -1000)
    assert master.drives[0].actual_position == 2000


def test_jog_requires_enable():
    master = SimulatedEtherCatMaster(num_drives=1).open()
    drv = EtherCatRobotDriver(master).connect()
    with pytest.raises(RobotDriverError, match="enable"):
        drv.jog_counts(0, 1000)


def test_set_home_then_cartesian_jog():
    # Bench: set home at the current pose, then a small Cartesian jog moves the
    # TCP in a straight line (both drives move through the kinematics).
    master = SimulatedEtherCatMaster(num_drives=2).open()
    drv = EtherCatRobotDriver(master).connect()
    drv.enable()
    assert not drv.is_referenced
    drv.set_home()
    assert drv.is_referenced
    start = [d.actual_position for d in master.drives]
    drv.jog_cartesian(5.0, 0.0, speed_mm_s=50.0)   # +5 mm in X
    end = [d.actual_position for d in master.drives]
    assert end != start                            # the tool moved


def test_cartesian_jog_requires_reference():
    master = SimulatedEtherCatMaster(num_drives=2).open()
    drv = EtherCatRobotDriver(master).connect()
    drv.enable()                                   # enabled but not referenced
    with pytest.raises(RobotDriverError, match="referenced"):
        drv.jog_cartesian(5.0, 0.0)


class _D:
    def __init__(self):
        self.actual_position = 0


def test_settle_waits_for_lagging_servo():
    # A real servo lags its target and only catches up after a few cycles; the
    # settle loop must wait for it instead of failing immediately.
    class LaggingMaster:
        cycle_dt_s = 0.002

        def __init__(self):
            self.drives = [_D(), _D()]
            self._n = 0

        def exchange(self):
            self._n += 1
            if self._n >= 3:                       # settles after a few exchanges
                for d in self.drives:
                    d.actual_position = 1000

    m = LaggingMaster()
    drv = EtherCatRobotDriver(m)
    drv.position_tol_counts = 5
    assert drv._settle((1000, 1000), timeout_s=1.0) == (1000, 1000)
    assert m._n >= 3


def test_settle_times_out_if_never_reached():
    class StuckMaster:
        cycle_dt_s = 0.002

        def __init__(self):
            self.drives = [_D(), _D()]

        def exchange(self):
            pass                                   # never catches up

    drv = EtherCatRobotDriver(StuckMaster())
    drv.position_tol_counts = 5
    assert drv._settle((1000, 1000), timeout_s=0.05) is None


def test_jog_counts_multi_moves_both_axes_together():
    # Two-drive bench: coordinated joint move ramps both axes to their targets
    # off one synchronized profile (opposite signs to prove independent direction).
    master = SimulatedEtherCatMaster(num_drives=2).open()
    drv = EtherCatRobotDriver(master).connect()
    drv.enable()
    drv.jog_counts_multi([3000, -1500], speed_counts_s=50000, accel_counts_s2=200000)
    assert master.drives[0].actual_position == 3000
    assert master.drives[1].actual_position == -1500


def test_jog_counts_multi_requires_enable_and_matching_deltas():
    master = SimulatedEtherCatMaster(num_drives=2).open()
    drv = EtherCatRobotDriver(master).connect()
    with pytest.raises(RobotDriverError, match="enable"):
        drv.jog_counts_multi([1000, 1000])
    drv.enable()
    with pytest.raises(RobotDriverError, match="delta"):
        drv.jog_counts_multi([1000])          # only one delta for two drives


def _mock_camera():
    return MockCamera(
        CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
    ).open()


# --- CiA 402 lifecycle ------------------------------------------------------ #
def test_enable_walks_drives_to_operation_enabled():
    drv, master, _ = _driver()
    assert not drv.is_enabled
    drv.enable()
    assert drv.is_enabled
    assert all(cia402.is_operation_enabled(d.statusword) for d in master.drives)


def test_move_requires_enable():
    drv, _, _ = _driver()
    with pytest.raises(RobotDriverError, match="disabled"):
        drv.move_to_angles(140.0, 40.0)


def test_home_then_read_angles():
    drv, _, _ = _driver(home=(140.5406, 39.4594))
    drv.enable()
    assert drv.read_angles() is None      # not referenced until homed
    drv.home()
    assert drv.is_referenced
    l, r = drv.read_angles()
    assert l == pytest.approx(140.5406, abs=0.02)   # rounding to counts
    assert r == pytest.approx(39.4594, abs=0.02)


def test_absolute_encoder_keeps_reference_across_disable():
    # Unlike the old open-loop steppers, the A6 absolute encoder holds the datum.
    drv, _, _ = _driver()
    drv.enable()
    drv.home()
    drv.disable()
    assert drv.is_referenced                # still referenced after disable
    assert drv.read_angles() is not None


def test_coordinated_move_reaches_target():
    drv, _, kin = _driver()
    drv.enable()
    drv.home()
    # move to the pose implied by a reachable TCP
    jt = kin.inverse(60.0, 250.0)
    drv.move_to_angles(jt.left_deg, jt.right_deg)
    l, r = drv.read_angles()
    assert l == pytest.approx(jt.left_deg, abs=0.05)
    assert r == pytest.approx(jt.right_deg, abs=0.05)


def test_move_streams_multiple_csp_cycles():
    drv, master, kin = _driver()
    drv.enable()
    drv.home()
    # a long move should stream many cycles (trapezoidal profile), not one jump
    jt = kin.inverse(120.0, 250.0)
    # count exchanges by wrapping the master
    n = {"x": 0}
    real = master.exchange
    def counting():
        n["x"] += 1
        real()
    master.exchange = counting
    drv.move_to_angles(jt.left_deg, jt.right_deg)
    assert n["x"] > 5


# --- faults ----------------------------------------------------------------- #
def test_injected_fault_is_reported_and_reset_clears_it():
    drv, master, _ = _driver()
    drv.enable()
    drv.home()
    master.inject_fault(0)
    master.exchange()
    assert drv.is_faulted
    assert not drv.is_referenced           # fault drops the reference
    assert drv.fault_code() == 1           # drive 0
    drv.reset()
    assert not drv.is_faulted


def test_fault_mid_move_raises():
    drv, master, kin = _driver()
    drv.enable()
    drv.home()
    master.inject_fault(1)
    master.exchange()
    jt = kin.inverse(60.0, 250.0)
    with pytest.raises(RobotDriverError, match="fault"):
        drv.move_to_angles(jt.left_deg, jt.right_deg)


# --- end-effector I/O (vacuum + air cylinder) ------------------------------- #
def test_vacuum_and_plunger_toggle_tooling_digital_outputs():
    # Default map: vacuum = bit 0, plunger = bit 1, on drive 0 (0x60FE:1).
    master = SimulatedEtherCatMaster(num_drives=2).open()
    drv = EtherCatRobotDriver(master).connect()
    assert master.drives[0].digital_outputs == 0
    drv.set_vacuum(True)
    assert master.drives[0].digital_outputs & 0b01
    drv.set_plunger(True)
    assert master.drives[0].digital_outputs & 0b10
    drv.set_vacuum(False)                       # clears only the vacuum bit
    assert not master.drives[0].digital_outputs & 0b01
    assert master.drives[0].digital_outputs & 0b10
    drv.set_plunger(False)
    assert master.drives[0].digital_outputs == 0
    assert master.drives[1].digital_outputs == 0   # other drive untouched


def test_tooling_do_bits_are_configurable():
    master = SimulatedEtherCatMaster(num_drives=2).open()
    drv = EtherCatRobotDriver(
        master, vacuum_do_bit=3, plunger_do_bit=5, tooling_drive=1
    ).connect()
    drv.set_vacuum(True)
    drv.set_plunger(True)
    assert master.drives[1].digital_outputs == (1 << 3) | (1 << 5)
    assert master.drives[0].digital_outputs == 0


class _RecordingDriver(DryRunRobotDriver):
    """A dry-run driver that records the actuation trace for order assertions."""

    def __init__(self, home_angles=(140.5406, 39.4594)):
        super().__init__(home_angles=home_angles)
        self.trace = []
        self.move_speeds = []

    def move_to_angles(self, left_deg, right_deg, speed_mm_s=None):
        super().move_to_angles(left_deg, right_deg, speed_mm_s)
        self.move_speeds.append(speed_mm_s)
        self.trace.append(("move", round(left_deg, 2), round(right_deg, 2)))

    def set_vacuum(self, on):
        super().set_vacuum(on)
        self.trace.append(("vacuum", bool(on)))

    def set_plunger(self, extended):
        super().set_plunger(extended)
        self.trace.append(("plunger", bool(extended)))


def _job(kin, val):
    from bung_cover_robot.robot.planner import make_job

    return make_job(kin, val, hole_index=0, cover_id=0,
                    pick_xy=(60.0, 250.0), drop_xy=(-40.0, 250.0))


def test_pick_place_sequence_actuates_head_in_order():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    drv = _RecordingDriver()
    drv.enable()
    drv.home()
    runner = DirectJobRunner(drv, PickSequence(0, 0, 0), sleep=lambda _s: None)
    res = runner.run(_job(kin, val))
    assert res.ok
    # move-to-pick, plunge, grip, lift, move-to-drop, plunge, release, lift.
    kinds = [e[0] if e[0] != "move" else "move" for e in drv.trace]
    assert kinds == [
        "move", "plunger", "vacuum", "plunger",
        "move", "plunger", "vacuum", "plunger",
    ]
    # vacuum ON at the pick, OFF at the drop; cylinder ends retracted both times.
    vac = [e[1] for e in drv.trace if e[0] == "vacuum"]
    assert vac == [True, False]
    plunge = [e[1] for e in drv.trace if e[0] == "plunger"]
    assert plunge == [True, False, True, False]


def test_pick_sequence_dwells_between_actions():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)
    drv = _RecordingDriver()
    drv.enable()
    drv.home()
    slept = []
    runner = DirectJobRunner(
        drv, PickSequence(plunge_dwell_s=0.1, grip_dwell_s=0.2, release_dwell_s=0.3),
        sleep=slept.append,
    )
    runner.run(_job(kin, val))
    # pick: plunge, grip; place: plunge, release.
    assert slept == [0.1, 0.2, 0.1, 0.3]


def test_pick_sequence_vents_head_on_move_failure():
    kin = FiveBarKinematics()
    val = WorkspaceValidator(kin)

    class _Failing(_RecordingDriver):
        def move_to_angles(self, left_deg, right_deg, speed_mm_s=None):
            # fail on the drop move — after vacuum is already ON.
            if self.vacuum_on:
                raise RobotDriverError("simulated fault mid-move")
            super().move_to_angles(left_deg, right_deg, speed_mm_s)

    drv = _Failing()
    drv.enable()
    drv.home()
    runner = DirectJobRunner(drv, PickSequence(0, 0, 0), sleep=lambda _s: None)
    res = runner.run(_job(kin, val))
    assert not res.ok and "fault" in res.reason
    # head must be left safe: vacuum vented, cylinder retracted.
    assert drv.vacuum_on is False
    assert drv.plunger_extended is False


# --- end to end through the cycle ------------------------------------------- #
def test_make_job_runner_is_direct_for_ethercat():
    drv, _, _ = _driver()
    assert isinstance(make_job_runner(drv), DirectJobRunner)


def test_build_controller_sim_ec_backend():
    from bung_cover_robot.app.launch import build_controller

    ctrl = build_controller(sim_ec=True)
    assert isinstance(ctrl.driver, EtherCatRobotDriver)
    # end-to-end: it enables, homes, and moves like any RobotDriver
    ctrl.enable()
    ctrl.home_reference()
    assert ctrl.is_referenced


def test_full_cycle_over_simulated_ethercat():
    kin = FiveBarKinematics()
    master = SimulatedEtherCatMaster().open()
    val = WorkspaceValidator(kin)
    home = kin.inverse(0.0, 250.0)
    drv = EtherCatRobotDriver(master, kin, val, home_angles=(home.left_deg, home.right_deg))
    ctrl = RobotTestController(drv, kin, val)
    ctrl.enable()
    ctrl.home_reference()
    mgr = CycleManager(ctrl, _mock_camera(), demo_transform(),
                       pick_sequence=PickSequence(0, 0, 0))
    res = mgr.run_cycle()
    assert res.ok and len(res.placed) == 3     # the 3 reachable demo covers
