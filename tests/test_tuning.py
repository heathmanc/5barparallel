"""Servo tuning assistant: FF-parameter wiring, the following-error grading
helpers, and the end-to-end characterize move (peak FE, and FF cutting it) on
the simulated network."""

import pytest

from bung_cover_robot.app.robot_test_controller import build_dry_run_controller
from bung_cover_robot.ethercat.ethercat_driver import EtherCatRobotDriver
from bung_cover_robot.ethercat.master import SimulatedEtherCatMaster
from bung_cover_robot.ethercat.parameters import (
    DEFAULT_TUNING,
    ParameterStore,
    parse_drive_address,
)
from bung_cover_robot.ethercat.tuning import fe_degrees, fe_margin_pct, grade
from bung_cover_robot.robot.driver import HomingConfig


def test_fe_helpers():
    assert fe_degrees(1092, 1092.0) == pytest.approx(1.0)
    assert fe_degrees(-2184, 1092.0) == pytest.approx(2.0)
    assert fe_degrees(500, 0) == 0.0                      # no scale -> 0, not a crash
    assert fe_margin_pct(2000, 4000) == 50.0
    assert fe_margin_pct(500, 0) == float("inf")          # unknown window
    assert grade(1000, 4000) == "good"                    # 25%
    assert grade(2600, 4000) == "ok"                      # 65%
    assert grade(3800, 4000) == "marginal"                # 95%
    assert grade(5000, 4000) == "TRIPPING"                # >100%


def test_speed_ff_objects_seeded_at_esi_addresses():
    names = {n: (addr, val) for n, addr, val, *_ in DEFAULT_TUNING}
    # Confirmed against STEPPERONLINE_A6_Servo ESI DT2001 (object 0x2001 "Gain
    # Parameter"): decimal subindices 20/21 (speed FF sel/gain), 23/24 (torque).
    assert parse_drive_address(names["speed_ff_source"][0]) == (0x2001, 20)
    assert parse_drive_address(names["speed_ff_gain"][0]) == (0x2001, 21)
    assert parse_drive_address(names["torque_ff_source"][0]) == (0x2001, 23)
    assert parse_drive_address(names["torque_ff_gain"][0]) == (0x2001, 24)
    assert names["speed_ff_source"][1] == 1        # seeded ON (internal ref)
    assert names["torque_ff_source"][1] == 0        # torque FF off by default
    # a fresh store seeds them as custom drive objects at the right CoE address
    store = ParameterStore()
    store._seed_default_tuning()
    src = next(c for c in store.custom_parameters() if c.name == "speed_ff_source")
    assert (src.index, src.sub) == (0x2001, 20)


def _connected_driver(num_drives=2):
    ctrl = build_dry_run_controller()
    m = SimulatedEtherCatMaster(num_drives=num_drives).open()
    drv = EtherCatRobotDriver(m, ctrl.kin, ctrl.validator,
                              home_angles=HomingConfig().home_angles).connect()
    drv.enable()
    drv.set_home()
    return drv, m


def _set_ff(master, pct):
    for d in range(len(master.drives)):
        master.sdo_write(0x2001, 20, 1 if pct > 0 else 0, drive=d)
        master.sdo_write(0x2001, 21, int(pct * 10), drive=d)


def test_characterize_returns_peak_fe_per_drive():
    drv, m = _connected_driver()
    _set_ff(m, 0)
    peaks = drv.characterize(80.0, 40.0, 1500)
    assert len(peaks) == 2
    assert max(peaks) > 0                    # a real move produces following error
    assert all(p >= 0 for p in peaks)


def test_speed_feedforward_cuts_following_error():
    drv, m = _connected_driver()
    _set_ff(m, 0)
    off = max(drv.characterize(80.0, 40.0, 1500))
    _set_ff(m, 100)
    on = max(drv.characterize(80.0, 40.0, 1500))
    assert on < off                          # feedforward reduces the peak lag
    assert on == 0                           # full FF cancels it in the ideal model


def test_csp_fe_peak_resets_each_run():
    drv, m = _connected_driver()
    _set_ff(m, 0)
    drv.characterize(80.0, 40.0, 1500)
    big = max(m.csp_fe_peak())
    # a tiny move must not inherit the previous big peak (reset per run_csp)
    drv.characterize(2.0, 0.0, 100)
    assert max(m.csp_fe_peak()) < big
