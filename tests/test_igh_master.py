"""IgHMaster — layout/ABI + no-daemon error path (the RT part needs hardware)."""

import struct

import pytest

from bung_cover_robot.ethercat import cia402
from bung_cover_robot.ethercat.igh_master import (
    _CSP_BASE,
    _CSP_MAX,
    _DRIVE_BASE,
    _DRIVE_SZ,
    _MAX_DRIVES,
    _SDO_BASE,
    _SHM_SIZE,
    IgHMaster,
)
from bung_cover_robot.ethercat.master import EtherCatMaster, MasterError


def test_shm_layout_matches_c_struct():
    # Mirror of shm_layout_t in igh/ec_master_daemon.c: 52-byte header, 2x32-byte
    # drive blocks, MAX_DRIVES x CSP_MAX int32 setpoints, then a 28-byte SDO
    # request/result channel (ABI 2).
    assert _DRIVE_BASE == 52
    assert _DRIVE_SZ == 32
    assert _CSP_BASE == _DRIVE_BASE + _MAX_DRIVES * _DRIVE_SZ == 116
    assert _SDO_BASE == _CSP_BASE + _MAX_DRIVES * _CSP_MAX * 4 == 524404
    assert _SHM_SIZE == _SDO_BASE + 28 == 524432


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
