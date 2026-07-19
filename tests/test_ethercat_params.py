"""Drive parameter registry/store + the Drives tab."""

import pytest

from bung_cover_robot.ethercat.parameters import (
    PARAMETERS,
    ParameterStore,
    default_values,
    parse_drive_address,
)


def test_defaults_complete_and_typed():
    vals = default_values()
    assert set(vals) == {p.name for p in PARAMETERS}
    assert vals["homing_method"] == 24 and isinstance(vals["homing_method"], (int, float))


def test_store_roundtrip(tmp_path):
    p = tmp_path / "drive_parameters.yaml"
    s = ParameterStore.load(p)                 # missing file -> defaults
    s.set("speed_mm_s", 123.5)
    s.set("following_error_window", 8000.7)    # int param coerces
    s.save()
    s2 = ParameterStore.load(p)
    assert s2.get("speed_mm_s") == 123.5
    assert s2.get("following_error_window") == 8001
    assert s2.get("accel_mm_s2") == 2000.0     # untouched -> default


def test_store_rejects_unknown():
    s = ParameterStore()
    with pytest.raises(KeyError):
        s.set("nope", 1)


def test_trajectory_limits_from_store():
    s = ParameterStore()
    s.set("speed_mm_s", 100.0)
    s.set("max_joint_step_deg", 0.0)           # 0 -> disabled (None)
    lim = s.trajectory_limits()
    assert lim.speed_mm_s == 100.0 and lim.max_joint_step_deg is None


def test_apply_to_sim_driver_updates_limits():
    from bung_cover_robot.ethercat import EtherCatRobotDriver, SimulatedEtherCatMaster

    drv = EtherCatRobotDriver(SimulatedEtherCatMaster(num_drives=2).open())
    s = ParameterStore()
    s.set("speed_mm_s", 75.0)
    s.set("position_tol_counts", 9)
    notes = s.apply(drv)
    assert drv.limits.speed_mm_s == 75.0 and drv.position_tol_counts == 9
    assert notes[0].startswith("motion limits")


def test_apply_writes_both_drives_and_reads_back(tmp_path):
    from bung_cover_robot.ethercat import EtherCatRobotDriver, SimulatedEtherCatMaster

    drv = EtherCatRobotDriver(SimulatedEtherCatMaster(num_drives=2).open())
    s = ParameterStore.load(tmp_path / "p.yaml")   # preloaded tuning params
    s.set_custom_value("machine_stiffness", 17)
    s.apply(drv)
    # both drives got the value over SDO
    idx, sub = 0x2000, 0x04                          # C00.03 -> 0x2000:(3+1)
    assert drv.master.sdo_read(idx, sub, drive=0) == 17
    assert drv.master.sdo_read(idx, sub, drive=1) == 17
    # read-back surfaces per-drive actuals for the table
    rb = s.read_custom_from_drives(drv)
    assert rb["machine_stiffness"] == [17, 17]


def test_apply_only_writes_changed_parameters(tmp_path):
    """Apply reads each object first and writes only what differs. A second
    Apply with no edits must write nothing."""
    from bung_cover_robot.ethercat import EtherCatRobotDriver, SimulatedEtherCatMaster

    class CountingSim(SimulatedEtherCatMaster):
        writes = 0

        def sdo_write(self, index, sub, value, size=4, drive=0):
            type(self).writes += 1
            super().sdo_write(index, sub, value, size=size, drive=drive)

    drv = EtherCatRobotDriver(CountingSim(num_drives=2).open())
    s = ParameterStore.load(tmp_path / "p.yaml")
    notes1 = s.apply(drv)
    first = CountingSim.writes
    assert first > 0 and notes1[-1].endswith("unchanged")
    assert "0 written" not in notes1[-1]              # first pass wrote things
    notes2 = s.apply(drv)                              # nothing changed
    assert CountingSim.writes == first                # no new writes
    assert notes2[-1].startswith("0 written")


def test_parse_drive_address_forms():
    # Cxx.NN maps to 0x20xx : NN+1 (the A6-EC rule, e.g. C0A.08 -> 0x200A:09).
    assert parse_drive_address("C0A.08") == (0x200A, 0x09)
    assert parse_drive_address("C10.00") == (0x2010, 0x01)
    # Raw CoE addresses pass through (0x optional, second field hex).
    assert parse_drive_address("0x6098:00") == (0x6098, 0)
    assert parse_drive_address("6041:0") == (0x6041, 0)
    with pytest.raises(ValueError):
        parse_drive_address("garbage")


def test_custom_parameters_roundtrip_and_apply(tmp_path):
    from bung_cover_robot.ethercat import EtherCatRobotDriver, SimulatedEtherCatMaster

    p = tmp_path / "drive_parameters.yaml"
    s = ParameterStore.load(p)
    cp = s.add_custom("rigidity", "C09.00", 12, "int", desc="test")
    assert cp.index == 0x2009 and cp.sub == 0x01 and cp.address == "0x2009:1"
    assert cp.desc == "test"
    s.set_custom_value("rigidity", 15)
    s.save()
    s2 = ParameterStore.load(p)
    names = [c.name for c in s2.custom_parameters()]
    assert "rigidity" in names
    assert next(c for c in s2.custom_parameters() if c.name == "rigidity").value == 15
    # Applies alongside the built-ins; the sim master now round-trips SDO.
    drv = EtherCatRobotDriver(SimulatedEtherCatMaster(num_drives=2).open())
    notes = s2.apply(drv)
    assert any("rigidity" in n for n in notes)
    assert drv.master.sdo_read(0x2009, 0x01, drive=0) == 15
    s2.remove_custom("rigidity")
    assert "rigidity" not in [c.name for c in s2.custom_parameters()]


def test_tuning_parameters_are_preloaded_and_seed_is_sticky(tmp_path):
    from bung_cover_robot.ethercat.parameters import DEFAULT_TUNING

    p = tmp_path / "drive_parameters.yaml"
    s = ParameterStore.load(p)                     # fresh -> preloaded
    names = {c.name for c in s.custom_parameters()}
    assert {n for n, *_ in DEFAULT_TUNING} <= names
    assert any(c.name == "machine_stiffness" and c.desc for c in s.custom_parameters())
    # Removing a preloaded param and saving must NOT bring it back on reload.
    s.remove_custom("machine_stiffness")
    s.save()
    s2 = ParameterStore.load(p)
    assert "machine_stiffness" not in {c.name for c in s2.custom_parameters()}


def test_apply_retries_on_length_abort():
    """A drive that rejects the assumed width with CoE abort 0x06070012 must be
    retried at another width instead of failing (0x6098 is 1 byte; a vendor gain
    might be 2). Non-length aborts still fail."""
    from bung_cover_robot.ethercat.master import MasterError

    class SizePicky:
        def __init__(self):
            self.drives = [1, 2]
            self.written = {}

        def sdo_write(self, index, sub, value, size=4, drive=0):
            if index == 0x2009 and size != 2:          # this object is 2 bytes only
                raise MasterError(f"SDO write 0x{index:04X}:{sub} (drive {drive}) "
                                  "failed (code 0x06070012)")
            self.written[(drive, index)] = (value, size)

    class Drv:
        def __init__(self, m):
            self.master = m
            self.limits = None
            self.position_tol_counts = 0

    s = ParameterStore()                                # curated params only
    s.add_custom("gain", "C09.00", 7, "int")            # 0x2009:1, assumed 4 bytes
    drv = Drv(SizePicky())
    notes = s.apply(drv)
    assert drv.master.written[(0, 0x2009)] == (7, 2)    # retried down to 2 on both
    assert drv.master.written[(1, 0x2009)] == (7, 2)
    assert not any("FAILED" in n for n in notes)


def test_add_custom_requires_name():
    s = ParameterStore()                            # bare store: not seeded
    assert s.custom_parameters() == []
    with pytest.raises(ValueError):
        s.add_custom("  ", "C09.00", 1)
