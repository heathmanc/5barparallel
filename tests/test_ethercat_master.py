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
