"""IgHMaster — layout/ABI + no-daemon error path (the RT part needs hardware)."""

import struct

import pytest

from bung_cover_robot.ethercat import cia402
from bung_cover_robot.ethercat.igh_master import (
    _CSP_BASE,
    _CSP_MAX,
    _CSP_VEL_BASE,
    _H_CYCLE_DT,
    _DRIVE_BASE,
    _DRIVE_SZ,
    _LINK_RAW,
    _LINK_RAW_SZ,
    _LINK_RESET,
    _LINK_SEQ,
    _MAX_DRIVES,
    _SDO_BASE,
    _SHM_SIZE,
    IgHMaster,
    link_error_total,
    parse_link_raw,
)
from bung_cover_robot.ethercat.master import EtherCatMaster, MasterError


def test_shm_layout_matches_c_struct():
    # Mirror of shm_layout_t in igh/ec_master_daemon.c: 52-byte header, 2x32-byte
    # drive blocks, MAX_DRIVES x CSP_MAX int32 setpoints, a 36-byte SDO channel,
    # then (ABI 4) the ESC link/CRC error-counter block (u32 reset flag, u32 seq
    # per drive, 20 raw register bytes per drive), then (ABI 5) the velocity-
    # offset FF stream: MAX_DRIVES x CSP_MAX int32.
    assert _DRIVE_BASE == 52
    assert _DRIVE_SZ == 32
    assert _CSP_BASE == _DRIVE_BASE + _MAX_DRIVES * _DRIVE_SZ == 116
    assert _SDO_BASE == _CSP_BASE + _MAX_DRIVES * _CSP_MAX * 4 == 524404
    assert _LINK_RESET == _SDO_BASE + 36 == 524440
    assert _LINK_SEQ == _SDO_BASE + 40
    assert _LINK_RAW == _SDO_BASE + 48
    assert _CSP_VEL_BASE == _LINK_RAW + _MAX_DRIVES * _LINK_RAW_SZ == 524492
    assert _SHM_SIZE == _CSP_VEL_BASE + _MAX_DRIVES * _CSP_MAX * 4 == 1048780


def test_is_a_master_with_requested_drives():
    m = IgHMaster(num_drives=1, auto_launch=False)
    assert isinstance(m, EtherCatMaster)
    assert m.num_drives == 1
    assert len(m.drives) == 1
    assert not m.is_open


def test_open_without_daemon_reports_clearly(tmp_path, monkeypatch):
    # No shared memory + no auto-launch -> a clear MasterError, never a raw OSError.
    import bung_cover_robot.ethercat.igh_master as igh

    monkeypatch.setattr(igh, "_SHM_PATH", str(tmp_path / "nope"))
    m = IgHMaster(num_drives=1, auto_launch=False)
    with pytest.raises(MasterError):
        m.open()


def test_pack_helpers_roundtrip_through_a_fake_buffer():
    # Exercise the output/input offset math against a bytearray standing in for shm.
    m = IgHMaster(num_drives=1, auto_launch=False)
    m._mm = bytearray(_SHM_SIZE)
    pd = m.drives[0]
    pd.controlword = cia402.CW_ENABLE_OPERATION
    pd.target_position = -12345
    pd.digital_outputs = 0b10
    m._write_outputs(0, pd)
    base = _DRIVE_BASE
    assert struct.unpack_from("<H", m._mm, base + 0)[0] == cia402.CW_ENABLE_OPERATION
    assert struct.unpack_from("<i", m._mm, base + 4)[0] == -12345
    # forge an input image and read it back
    struct.pack_into("<H", m._mm, base + 12, cia402.SW_OPERATION_ENABLED)
    struct.pack_into("<i", m._mm, base + 16, 6789)
    struct.pack_into("<i", m._mm, base + 20, -3)
    m._read_inputs(0, pd)
    assert pd.statusword == cia402.SW_OPERATION_ENABLED
    assert pd.actual_position == 6789
    assert pd.following_error == -3


def _raw20(**kv):
    b = bytearray(20)
    at = {"inv0": 0, "rx0": 1, "inv1": 2, "rx1": 3, "fwd0": 8, "fwd1": 9,
          "pu": 12, "pdi": 13, "lost0": 16, "lost1": 17}
    for k, v in kv.items():
        b[at[k]] = v
    return bytes(b)


def test_parse_link_raw_field_map():
    c = parse_link_raw(_raw20(inv0=3, rx0=7, inv1=1, rx1=2, fwd0=4, fwd1=5,
                              pu=6, pdi=8, lost0=9, lost1=10))
    assert c["ports"][0] == {"invalid_frame": 3, "rx_error": 7,
                             "forwarded": 4, "lost_link": 9}
    assert c["ports"][1] == {"invalid_frame": 1, "rx_error": 2,
                             "forwarded": 5, "lost_link": 10}
    assert c["pu_error"] == 6 and c["pdi_error"] == 8
    assert link_error_total(c) == 55
    assert link_error_total(parse_link_raw(bytes(20))) == 0
    assert link_error_total(parse_link_raw(b"")) == 0     # short input tolerated


def test_link_counters_and_reset_through_fake_shm():
    m = IgHMaster(num_drives=2, auto_launch=False)
    m._mm = bytearray(_SHM_SIZE)
    m._num = 2
    m._mm[_LINK_RAW:_LINK_RAW + 20] = _raw20(rx0=37, inv0=37, lost0=1)
    counters = m.link_counters()
    assert link_error_total(counters[0]) == 75
    assert link_error_total(counters[1]) == 0
    # _read_inputs publishes the parsed dict onto the drive process data
    m._read_inputs(0, m.drives[0])
    assert link_error_total(m.drives[0].link_errors) == 75
    # reset raises the flag the daemon consumes
    m.reset_link_counters()
    assert struct.unpack_from("<I", m._mm, _LINK_RESET)[0] == 1


def test_open_guards_cycle_time_mismatch():
    """A daemon running at a different SYNC0 cycle than the trajectory plans for
    scales every speed (an Er.87.1 risk) and must be rejected."""
    m = IgHMaster(num_drives=2, auto_launch=False, cycle_dt_s=0.002)
    m._mm = bytearray(_SHM_SIZE)
    struct.pack_into("<I", m._mm, _H_CYCLE_DT, 1_000_000)   # daemon at 1 ms, we want 2 ms
    with pytest.raises(MasterError):
        m._check_cycle_match()
    struct.pack_into("<I", m._mm, _H_CYCLE_DT, 2_000_000)   # matched -> no raise
    m._check_cycle_match()
    struct.pack_into("<I", m._mm, _H_CYCLE_DT, 0)           # uninitialised -> tolerated
    m._check_cycle_match()
