"""Drive parameter registry/store + the Drives tab."""

import pytest

from bung_cover_robot.ethercat.parameters import (
    PARAMETERS,
    ParameterStore,
    default_values,
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

    drv = EtherCatRobotDriver(SimulatedEtherCatMaster().open())
    s = ParameterStore()
    s.set("speed_mm_s", 75.0)
    s.set("position_tol_counts", 9)
    notes = s.apply(drv)
    assert drv.limits.speed_mm_s == 75.0 and drv.position_tol_counts == 9
    assert any("sim master" in n for n in notes)  # honest about no SDO channel
