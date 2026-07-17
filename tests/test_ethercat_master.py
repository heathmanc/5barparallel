"""EtherCAT PDO pack/unpack + run_csp streaming (the parts testable off-HW)."""

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


# --- PDO layout ------------------------------------------------------------- #
def test_pack_outputs_size_and_roundtrip():
    data = pack_outputs(cia402.CW_ENABLE_OPERATION, cia402.MODE_CSP, 123456)
    assert len(data) == RX_SIZE == 7
    # the input image has the same layout; unpack recovers the fields
    sw, mode, actual = unpack_inputs(data)
    assert sw == cia402.CW_ENABLE_OPERATION
    assert mode == cia402.MODE_CSP
    assert actual == 123456


def test_unpack_decodes_negative_position():
    # a negative actual position must come back signed
    data = pack_outputs(0, cia402.MODE_CSP, -5000 & 0xFFFFFFFF)
    _, _, actual = unpack_inputs(data)
    assert actual == -5000
    assert TX_SIZE == 7


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
def test_pysoem_master_requires_pysoem_or_reports_clearly():
    m = PysoemMaster(ifname="eth-does-not-exist")
    # Without pysoem installed (or without hardware), open() must fail loudly
    # via MasterError, never silently pretend to be connected.
    assert not m.is_open
    with pytest.raises(MasterError):
        m.open()
