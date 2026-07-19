"""EtherCatRobotDriver against the simulated A6 network — CiA 402 arming, homing,
CSP moves, fault/reset, and a full cycle through CycleManager."""

import pytest

from bung_cover_robot.app.cycle_manager import CycleManager, make_job_runner, DirectJobRunner
from bung_cover_robot.app.robot_test_controller import RobotTestController
from bung_cover_robot.ethercat import (
    EtherCatRobotDriver,
    SimulatedEtherCatMaster,
    cia402,
)
from bung_cover_robot.gui.imaging import demo_frame, demo_transform
from bung_cover_robot.robot.driver import RobotDriverError
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
    mgr = CycleManager(ctrl, _mock_camera(), demo_transform())
    res = mgr.run_cycle()
    assert res.ok and len(res.placed) == 3     # the 3 reachable demo covers
