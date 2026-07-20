"""EtherCAT PDO pack/unpack + run_csp streaming (the parts testable off-HW)."""

import struct

import pytest

from bung_cover_robot.ethercat import cia402
from bung_cover_robot.ethercat.master import (
    RX_SIZE,
    TX_SIZE,
    MasterError,
    PysoemMaster,
    SimulatedEtherCatMaster,
    pack_outputs,
    unpack_inputs,
)


# --- PDO layout (ANCTL AS715N native map: RxPDO 0x1701 12B / TxPDO 0x1B01 28B) - #
def test_pack_outputs_size_and_fields():
    data = pack_outputs(cia402.CW_ENABLE_OPERATION, 123456, digital_outputs=0)
    assert len(data) == RX_SIZE == 12
    cw, target, _tp, _do = struct.unpack("<HiHI", data)
    assert cw == cia402.CW_ENABLE_OPERATION
    assert target == 123456


def test_pack_outputs_encodes_negative_target_signed():
    data = pack_outputs(0, -5000)
    _cw, target, _tp, _do = struct.unpack("<HiHI", data)
    assert target == -5000


def test_unpack_inputs_decodes_txpdo_fields():
    # Build a 0x1B01 image: errcode, status, actual, torque, foll_err, tp_status,
    # tp1, tp2, dig_in — signed fields must come back signed.
    status = cia402.SW_OPERATION_ENABLED | cia402.SW_VOLTAGE_ENABLED
    img = struct.pack("<HHihiHiiI", 0, status, -5000, 12, -3, 0, 0, 0, 0b101)
    assert len(img) == TX_SIZE == 28
    inp = unpack_inputs(img)
    assert inp.statusword == status
    assert inp.actual_position == -5000
    assert inp.following_error == -3
    assert inp.torque_actual == 12
    assert inp.digital_inputs == 0b101
    assert inp.error_code == 0


# --- run_csp on the simulator ----------------------------------------------- #
def _enabled_sim():
    m = SimulatedEtherCatMaster().open()
    # arm both drives to Operation Enabled
    for _ in range(10):
        for d in m.drives:
            d.controlword = cia402.next_controlword(d.statusword, d.controlword)
        m.exchange()
        if all(cia402.is_operation_enabled(d.statusword) for d in m.drives):
            break
    return m


def test_run_csp_streams_targets_in_order():
    m = _enabled_sim()
    targets = [(i * 10, i * 20) for i in range(6)]
    m.run_csp(targets)
    assert m.drives[0].actual_position == 50
    assert m.drives[1].actual_position == 100


def test_run_csp_raises_on_fault():
    m = _enabled_sim()
    m.inject_fault(0)
    with pytest.raises(MasterError, match="fault"):
        m.run_csp([(1, 1), (2, 2), (3, 3)])


# --- pysoem master (no hardware) -------------------------------------------- #
def test_pysoem_master_mode_selects_profile_position_for_bench():
    # Bench single-axis uses Profile Position (async, no SYNC0) to avoid the A6
    # CSP sync fault; the drive images carry that mode for the display.
    pp = PysoemMaster(mode=cia402.MODE_PROFILE_POSITION, num_drives=1)
    assert pp.mode == cia402.MODE_PROFILE_POSITION
    assert pp.drives[0].mode_of_operation == cia402.MODE_PROFILE_POSITION
    # Default is CSP (production coordinated motion).
    assert PysoemMaster().mode == cia402.MODE_CSP


def test_pysoem_master_requires_pysoem_or_reports_clearly():
    m = PysoemMaster(ifname="eth-does-not-exist")
    # Without pysoem installed (or without hardware), open() must fail loudly
    # via MasterError, never silently pretend to be connected.
    assert not m.is_open
    with pytest.raises(MasterError):
        m.open()


# --- single-axis jog ramp --------------------------------------------------- #
def test_ramp_counts_endpoints_and_smoothness():
    from bung_cover_robot.ethercat.trajectory import ramp_counts

    dt = 0.002
    ramp = ramp_counts(1000, 5000, speed=20000, accel=100000, dt=dt)
    assert ramp[0] == 1000 and ramp[-1] == 6000
    assert all(b >= a for a, b in zip(ramp, ramp[1:]))        # monotonic up
    assert max(b - a for a, b in zip(ramp, ramp[1:])) <= 20000 * dt + 2


def test_ramp_counts_negative_and_zero():
    from bung_cover_robot.ethercat.trajectory import ramp_counts

    assert ramp_counts(500, 0, 1000, 1000, 0.002) == [500]
    down = ramp_counts(0, -3000, 20000, 100000, 0.002)
    assert down[0] == 0 and down[-1] == -3000
    assert all(b <= a for a, b in zip(down, down[1:]))        # monotonic down


def test_ramp_counts_multi_synchronized_and_exact():
    from bung_cover_robot.ethercat.trajectory import ramp_counts, ramp_counts_multi

    dt = 0.002
    # Opposite directions, different magnitudes: they must start and finish together.
    ramp = ramp_counts_multi([1000, -500], [4000, -2000],
                             speed=20000, accel=100000, dt=dt)
    assert ramp[0] == (1000, -500)                 # both start at their actuals
    assert ramp[-1] == (5000, -2500)               # both land exactly on target
    assert len(ramp) >= 2
    # Same number of samples for every axis -> perfectly synchronized in time.
    axis0 = [r[0] for r in ramp]
    axis1 = [r[1] for r in ramp]
    assert len(axis0) == len(axis1)
    assert all(b >= a for a, b in zip(axis0, axis0[1:]))     # axis0 monotonic up
    assert all(b <= a for a, b in zip(axis1, axis1[1:]))     # axis1 monotonic down
    # The dominant axis follows the same trapezoid as a lone single-axis ramp.
    solo = ramp_counts(1000, 4000, speed=20000, accel=100000, dt=dt)
    assert axis0 == solo


def test_ramp_counts_multi_shared_fraction():
    from bung_cover_robot.ethercat.trajectory import ramp_counts_multi

    # A half-size second axis should be at ~half the first axis's progress every
    # cycle (shared profile), and the zero-delta axis never moves.
    ramp = ramp_counts_multi([0, 0, 100], [1000, 500, 0],
                             speed=20000, accel=100000, dt=0.002)
    for a0, a1, a2 in ramp:
        assert a2 == 100                            # zero delta axis holds
        assert abs(a1 - a0 / 2) <= 1                # half progress, within rounding


def test_ramp_counts_multi_zero_move():
    from bung_cover_robot.ethercat.trajectory import ramp_counts_multi

    assert ramp_counts_multi([10, 20], [0, 0], 1000, 1000, 0.002) == [(10, 20)]


# --- abort_csp + FF cleanup ---------------------------------------------------
def test_pysoem_abort_csp_stops_the_rt_stream():
    m = PysoemMaster(num_drives=2)          # not opened; just exercise the flag
    m._csp_running = True
    m.abort_csp()
    assert m._csp_running is False


def test_sim_abort_csp_is_a_safe_noop():
    m = SimulatedEtherCatMaster(num_drives=2).open()
    m.abort_csp()                           # synchronous run_csp -> nothing to abort
    assert m.is_open


def test_sim_run_csp_clears_ff_on_fault_no_phantom_following_error():
    """A mid-stream fault must not leave a streamed velocity-FF set: a stale
    ff_counts makes the drive report a phantom following error at rest
    (resid = 0 - ff_counts). Regression for the skipped-cleanup path."""
    m = SimulatedEtherCatMaster(num_drives=2).open()
    m.sdo_write(0x2001, 20, 5, drive=0)     # speed-FF source = communication
    m.inject_fault(0)                       # faults on the first exchange
    with pytest.raises(MasterError):
        m.run_csp([(0, 0), (10, 10)], velocities=[(1000, 1000), (1000, 1000)])
    assert all(sim.ff_counts is None for sim in m._sim)
