"""Offscreen smoke test for the Qt Robot Test tab.

Runs headless via the Qt 'offscreen' platform. Skipped entirely if PySide6 isn't
installed, so the rest of the suite doesn't depend on Qt.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication  # noqa: E402

from bung_cover_robot.app.robot_test_controller import RobotTestController  # noqa: E402
from bung_cover_robot.gui.main_window import MainWindow  # noqa: E402
from bung_cover_robot.gui.robot_test_tab import RobotTestTab  # noqa: E402
from bung_cover_robot.robot import DryRunRobotDriver  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


def test_main_window_has_robot_test_tab(qapp):
    win = MainWindow()
    assert win.tabs.count() == 1
    assert win.tabs.tabText(0) == "Robot Test"


def test_jog_buttons_disabled_until_enabled(qapp):
    tab = RobotTestTab(RobotTestController(DryRunRobotDriver()))
    assert all(not b.isEnabled() for b in tab._jog_buttons)
    tab.enable_btn.click()  # toggles + emits clicked(checked=True)
    assert tab.controller.is_enabled
    assert all(b.isEnabled() for b in tab._jog_buttons)


def test_home_then_jog_updates_readout(qapp):
    tab = RobotTestTab(RobotTestController(DryRunRobotDriver()))
    tab.enable_btn.click()
    tab._on_go_home()
    assert tab.controller.is_homed
    assert tab.homed_label.text() == "HOMED"

    tab.joint_step.setValue(1.0)
    before = tab._value_labels["left_deg"].text()
    tab._jog_joint("left", +1)
    after = tab._value_labels["left_deg"].text()
    assert before != after
    assert "OK" in tab.status_label.text()


def test_rejected_move_shows_reason(qapp):
    tab = RobotTestTab(RobotTestController(DryRunRobotDriver()))
    tab.enable_btn.click()
    tab._on_go_home()
    # A giant cartesian step leaves the workspace -> rejection surfaced in status.
    tab.cart_step.setValue(50.0)
    for _ in range(5):
        tab._jog_cart("y", +1)
    assert "Rejected" in tab.status_label.text()
