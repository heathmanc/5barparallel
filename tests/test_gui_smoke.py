"""Offscreen smoke test for the Qt tabs.

Runs headless via the Qt 'offscreen' platform. Skipped if PySide6 isn't
installed, so the rest of the suite doesn't depend on Qt.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication  # noqa: E402

from bung_cover_robot.app.robot_test_controller import (  # noqa: E402
    build_dry_run_controller,
)
from bung_cover_robot.gui.main_window import MainWindow  # noqa: E402
from bung_cover_robot.gui.robot_test_tab import RobotTestTab  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


def test_main_window_has_all_tabs(qapp):
    win = MainWindow()
    assert [win.tabs.tabText(i) for i in range(win.tabs.count())] == [
        "Robot Test",
        "Settings",
        "PLC",
    ]


def test_plc_tab_lists_every_tag(qapp):
    from bung_cover_robot.plc import tags as T

    win = MainWindow()
    table = win.plc_tab.table
    assert table.rowCount() == len(T.TAG_SPECS)
    assert table.columnCount() == 5
    # Spot-check a known row's tag + type appear.
    cells = {
        table.item(r, 1).text(): table.item(r, 2).text()
        for r in range(table.rowCount())
    }
    assert cells["VisionRobot.Manual.MoveToTarget"] == "BOOL"
    assert cells["VisionRobot.Status.ActualLeftDeg"] == "REAL"


def test_plc_tab_connect_simulated_and_disconnect(qapp):
    from bung_cover_robot.plc import PlcRobotDriver, SimulatedPlcClient
    from bung_cover_robot.robot.driver import DryRunRobotDriver

    win = MainWindow()  # dry-run by default
    plc, rt = win.plc_tab, win.robot_test_tab
    assert isinstance(win.controller.driver, DryRunRobotDriver)

    # Reference against the dry-run driver first...
    rt.enable_btn.click()
    rt._on_home_reference()
    assert win.controller.is_referenced

    # ...then connecting the simulated PLC swaps the driver and forces re-home.
    plc._on_connect_sim()
    assert isinstance(win.controller.driver, PlcRobotDriver)
    assert isinstance(win.controller.driver.client, SimulatedPlcClient)
    assert "simulated PLC" in plc.status_label.text()
    assert not win.controller.is_referenced
    assert not rt.enable_btn.isChecked()  # refreshed after swap

    # And it really drives: enable + home over the PLC handshake.
    rt.enable_btn.click()
    rt._on_home_reference()
    assert win.controller.is_referenced

    plc._on_disconnect()
    assert isinstance(win.controller.driver, DryRunRobotDriver)
    assert "dry-run" in plc.status_label.text()


def test_plc_tab_connect_real_without_pycomm3_shows_error(qapp):
    from bung_cover_robot.robot.driver import DryRunRobotDriver

    win = MainWindow()
    plc = win.plc_tab
    plc.path_edit.setText("192.168.1.10/0")
    plc._on_connect_real()
    # pycomm3 isn't installed here -> graceful error, driver unchanged.
    assert "failed" in plc.status_label.text().lower()
    assert isinstance(win.controller.driver, DryRunRobotDriver)


def test_plc_tab_connect_real_requires_path(qapp):
    win = MainWindow()
    plc = win.plc_tab
    plc.path_edit.setText("")
    plc._on_connect_real()
    assert "IP/slot" in plc.status_label.text()


def test_jog_disabled_until_enabled_and_referenced(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    assert all(not b.isEnabled() for b in tab._jog_buttons)
    tab.enable_btn.click()
    assert tab.controller.is_enabled
    assert all(not b.isEnabled() for b in tab._jog_buttons)  # still not referenced
    tab._on_home_reference()
    assert tab.controller.is_referenced
    assert tab.referenced_label.text() == "REFERENCED"
    assert all(b.isEnabled() for b in tab._jog_buttons)


def test_reference_then_jog_updates_readout(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    tab.enable_btn.click()
    tab._on_home_reference()
    tab.joint_step.setValue(1.0)
    before = tab._value_labels["left_deg"].text()
    tab._jog_joint("left", +1)
    assert tab._value_labels["left_deg"].text() != before
    assert "OK" in tab.status_label.text()


def test_rejected_move_shows_reason(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    tab.enable_btn.click()
    tab._on_home_reference()
    tab.cart_step.setValue(50.0)
    for _ in range(5):
        tab._jog_cart("y", +1)
    assert "Rejected" in tab.status_label.text()


# --------------------------------------------------------------------------- #
# Settings tab
# --------------------------------------------------------------------------- #
def test_settings_loads_current_geometry(qapp):
    win = MainWindow()
    st = win.settings_tab
    assert st._floats["l1_mm"].value() == pytest.approx(220.0)
    assert st._floats["l2_mm"].value() == pytest.approx(230.0)


def test_settings_apply_valid_geometry(qapp):
    win = MainWindow()
    st = win.settings_tab
    st._floats["l1_mm"].setValue(225.0)
    assert st._on_apply() is True
    assert win.controller.kin.config.l1_mm == pytest.approx(225.0)


def test_settings_refuses_geometry_that_breaks_workzone(qapp):
    win = MainWindow()
    st = win.settings_tab
    before = win.controller.kin.config.l1_mm
    st._floats["l1_mm"].setValue(60.0)  # far too short to cover the work zone
    st._floats["l2_mm"].setValue(60.0)
    assert st._on_apply() is False
    assert "Refused" in st.status_label.text()
    assert win.controller.kin.config.l1_mm == before  # unchanged


def test_settings_save_preserves_homing_block(qapp, tmp_path):
    import shutil

    import yaml

    from pathlib import Path

    from bung_cover_robot.app.robot_test_controller import build_dry_run_controller
    from bung_cover_robot.gui.settings_tab import SettingsTab

    src = Path(__file__).resolve().parents[1] / "config" / "robot_config.yaml"
    dst = tmp_path / "robot_config.yaml"
    shutil.copy(src, dst)

    st = SettingsTab(build_dry_run_controller(), config_path=dst)
    st._floats["l1_mm"].setValue(221.0)  # a valid tweak
    st._on_save()

    data = yaml.safe_load(dst.read_text())
    assert data["geometry"]["l1_mm"] == pytest.approx(221.0)
    assert "homing" in data  # not clobbered
    assert data["homing"]["flag_radius_mm"] == 40.0
