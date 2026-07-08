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
        "Vision",
        "Camera",
        "Calibration",
        "Robot Test",
        "Settings",
        "PLC",
        "Bypass",
        "Diagnostics",
    ]


def test_diagnostics_tab_formats_and_updates(qapp):
    from bung_cover_robot.app.robot_test_controller import build_dry_run_controller
    from bung_cover_robot.gui.diagnostics_tab import DiagnosticsTab

    tab = DiagnosticsTab(build_dry_run_controller())
    # No PLC client on the dry-run driver -> no poller thread started.
    assert tab._poller is None

    assert DiagnosticsTab._format(True, "bool") == "1"
    assert DiagnosticsTab._format(None, "num") == "—"
    assert DiagnosticsTab._format(4, "faultcode") == "4 (homing fail/timeout)"
    assert "f85149" in DiagnosticsTab._color(1, "fault")          # fault true = red
    assert "f85149" in DiagnosticsTab._color(0, "numwarn0")       # HOME_VEL 0 = red
    assert DiagnosticsTab._color(2000, "numwarn0") == ""          # nonzero = plain

    # Feeding a poll result updates the value labels off any thread.
    tab._on_polled({"HOME_VEL_0": 0, "VisionRobot.Status.Faulted": True})
    assert tab._value_labels["HOME_VEL_0"][0].text() == "0"
    assert tab._value_labels["VisionRobot.Status.Faulted"][0].text() == "1"


def test_theme_applies(qapp):
    from bung_cover_robot.gui.theme import apply_theme

    apply_theme(qapp)
    assert "QTabBar::tab" in qapp.styleSheet()


def test_vision_tab_capture_and_status(qapp):
    win = MainWindow()
    vt = win.vision_tab
    # A frame was captured on construction -> the view has a pixmap.
    assert vt.view.pixmap() is not None and not vt.view.pixmap().isNull()
    # Status pills reflect the (dry-run) controller state.
    vt.refresh()
    assert vt.pill_drives.text() == "DISABLED"
    assert vt.pill_home.text() == "NOT REFERENCED"
    assert vt.pill_plc.text() == "DRY-RUN"
    assert vt.pill_camera.text() == "ONLINE"
    win.controller.enable()
    win.controller.home_reference()
    vt.refresh()
    assert vt.pill_drives.text() == "ENABLED"
    assert vt.pill_home.text() == "REFERENCED"


def test_vision_tab_detect_overlay(qapp):
    win = MainWindow()
    vt = win.vision_tab
    vt._on_detect()
    assert "holes" in vt.status_label.text()
    assert "6 holes" in vt.status_label.text()


def test_vision_tab_start_requires_enable_and_home(qapp):
    win = MainWindow()
    vt = win.vision_tab
    vt._on_start()  # not enabled/referenced yet
    assert "Cannot start" in vt.status_label.text()


def _wait_until(qapp, predicate, timeout_s=5.0):
    import time

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not predicate():
        qapp.processEvents()
    return predicate()


def test_vision_tab_start_runs_cycle(qapp):
    win = MainWindow()
    vt = win.vision_tab
    win.controller.enable()
    win.controller.home_reference()
    vt._on_start()
    # The cycle runs on a worker thread; wait for it to finish.
    assert _wait_until(qapp, lambda: not vt._running)
    assert "placed" in vt.status_label.text()
    assert vt.start_btn.isEnabled()  # re-enabled after the run
    assert vt._thread is None  # worker thread torn down cleanly


def test_vision_tab_bypass_runs_without_calibration(qapp):
    win = MainWindow()
    vt = win.vision_tab
    win.controller.enable()
    win.controller.home_reference()
    vt.set_calibration(None)          # no calibration at all
    vt.bypass_chk.setChecked(True)    # scripted targets
    vt._on_start()
    assert _wait_until(qapp, lambda: not vt._running)
    # cycle ran and placed covers despite no calibration/detection
    assert "placed" in vt.status_label.text()
    assert "0 cover" not in vt.status_label.text()


def test_cycle_worker_stop_before_run(qapp):
    from bung_cover_robot.app.cycle_manager import CycleManager
    from bung_cover_robot.app.robot_test_controller import build_dry_run_controller
    from bung_cover_robot.gui.cycle_worker import CycleWorker
    from bung_cover_robot.gui.imaging import demo_frame, demo_transform
    from bung_cover_robot.vision.camera import CameraConfig, MockCamera

    ctrl = build_dry_run_controller()
    ctrl.enable()
    ctrl.home_reference()
    cam = MockCamera(
        CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
    ).open()
    worker = CycleWorker(CycleManager(ctrl, cam, demo_transform()))
    results = []
    worker.finished.connect(results.append)
    worker.request_stop()
    worker.run()  # synchronous; should_stop is honored between holes
    assert len(results) == 1
    assert results[0].steps == [] and "stopped" in results[0].reason.lower()


def test_camera_tab_controls_and_grab(qapp):
    win = MainWindow()
    ct = win.camera_tab
    assert ct.view.pixmap() is not None and not ct.view.pixmap().isNull()
    # Moving a control slider pushes it to the camera and re-renders.
    ct._sliders["brightness"].setValue(50)
    assert win.camera.get_control("brightness") == pytest.approx(0.5)
    # Grab refreshes info label with the frame size.
    ct._grab()
    assert "×" in ct.info_label.text()


def test_camera_tab_use_mock_reconnect(qapp):
    win = MainWindow()
    ct = win.camera_tab
    ct._on_use_mock()  # emits cameraChanged -> main window updates vision tab
    assert win.vision_tab.camera is win.camera_tab.camera


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
# Calibration tab
# --------------------------------------------------------------------------- #
def _cal_camera():
    from bung_cover_robot.gui.imaging import demo_frame
    from bung_cover_robot.vision.camera import CameraConfig, MockCamera

    return MockCamera(
        CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
    ).open()


def _click_points(tab, pixel_pts, robot_pts):
    """Simulate clicking each pixel point and typing its robot XY."""
    from PySide6.QtWidgets import QTableWidgetItem

    for (px, py), (rx, ry) in zip(pixel_pts, robot_pts):
        tab._on_pixel_clicked(float(px), float(py))
        r = tab.table.rowCount() - 1
        tab.table.setItem(r, 2, QTableWidgetItem(str(rx)))
        tab.table.setItem(r, 3, QTableWidgetItem(str(ry)))


def test_clickable_view_maps_widget_to_source(qapp):
    from PySide6.QtGui import QPixmap
    from bung_cover_robot.gui.widgets import ClickableImageView

    view = ClickableImageView()
    view.resize(400, 400)
    view.set_pixmap(QPixmap(200, 100))  # 2:1 image letterboxed in a square widget
    # Image is scaled x2 -> 400x200, centered vertically (offset y = 100).
    assert view.widget_to_source(0.0, 100.0) == pytest.approx((0.0, 0.0), abs=0.5)
    assert view.widget_to_source(200.0, 200.0) == pytest.approx((100.0, 50.0), abs=0.5)
    # A click in the letterbox (above the image) is rejected.
    assert view.widget_to_source(200.0, 10.0) is None


def _rect_correspondences():
    pix = [[100, 100], [600, 100], [600, 400], [100, 400]]
    rob = [[-150, 300], [150, 300], [150, 200], [-150, 200]]
    return pix, rob


def _cal_tab(tmp_path):
    from bung_cover_robot.app.recipes import RecipeStore
    from bung_cover_robot.gui.calibration_tab import CalibrationTab
    from bung_cover_robot.vision.calibration import CalibrationManager

    mgr = CalibrationManager(tmp_path)
    recipes = RecipeStore(path=tmp_path / "recipes.yaml")  # defaults g31-6/g24-6
    return CalibrationTab(_cal_camera(), mgr, recipes), mgr, recipes


def test_calibration_tab_fit_and_save_per_recipe(qapp, tmp_path):
    tab, mgr, _ = _cal_tab(tmp_path)
    pix, rob = _rect_correspondences()

    assert not tab.fit_btn.isEnabled()  # nothing entered yet
    _click_points(tab, pix, rob)
    assert tab.fit_btn.isEnabled()

    tab._on_fit()
    assert tab._fitted is not None
    assert "residual" in tab.residual_label.text()
    assert tab.save_btn.isEnabled()  # a recipe is always selected

    saved = []
    tab.calibrationSaved.connect(lambda k, t: saved.append((k, t)))
    key = tab._selected_recipe_key()
    tab._on_save()
    assert mgr.has(key)
    assert len(saved) == 1 and saved[0][0] == key  # (recipe_key, transform)
    loaded = mgr.get(key)
    assert loaded.pixel_to_robot(350, 250) == pytest.approx((0.0, 250.0), abs=1e-3)


def test_calibration_tab_add_recipe(qapp, tmp_path):
    tab, mgr, recipes = _cal_tab(tmp_path)
    before = tab.recipe_combo.count()
    tab.new_recipe_edit.setText("Group 65 8-vent")
    tab._on_add_recipe()
    assert recipes.has("group-65-8-vent")           # slugified + persisted
    assert tab.recipe_combo.count() == before + 1
    assert tab._selected_recipe_key() == "group-65-8-vent"  # auto-selected
    # Calibrate the freshly added recipe.
    pix, rob = _rect_correspondences()
    _click_points(tab, pix, rob)
    tab._on_fit()
    tab._on_save()
    assert mgr.has("group-65-8-vent")


def test_calibration_tab_remove_and_clear(qapp, tmp_path):
    tab, _, _ = _cal_tab(tmp_path)
    _click_points(tab, [[10, 10], [20, 20], [30, 30]], [[0, 0], [1, 1], [2, 2]])
    assert tab.table.rowCount() == 3 and len(tab._points) == 3
    tab.table.selectRow(1)
    tab._on_remove()
    assert tab.table.rowCount() == 2 and len(tab._points) == 2
    tab._on_clear()
    assert tab.table.rowCount() == 0 and not tab._points


def test_calibration_flows_into_vision_tab_when_active(qapp, tmp_path):
    from bung_cover_robot.vision.calibration import CalibrationManager

    win = MainWindow()
    ct = win.calibration_tab
    ct.manager = win.calibration_manager = CalibrationManager(tmp_path)
    # Calibrate the recipe the Vision tab is currently showing.
    active = win.vision_tab.active_recipe_key()
    ct.recipe_combo.setCurrentIndex(ct.recipe_combo.findData(active))
    pix, rob = _rect_correspondences()
    _click_points(ct, pix, rob)
    ct._on_fit()
    fitted = ct._fitted
    ct._on_save()
    # The saved transform is now the Vision tab's active calibration.
    assert win.vision_tab.calibration is fitted


def test_vision_recipe_changeover_loads_calibration(qapp, tmp_path):
    from bung_cover_robot.vision.calibration import CalibrationManager, HomographyTransform

    win = MainWindow()
    win.calibration_manager = CalibrationManager(tmp_path)
    pix, rob = _rect_correspondences()
    # Give the second recipe (only) a real calibration on disk.
    second = win.recipes.list()[1].key
    saved = HomographyTransform.from_correspondences(pix, rob)
    win.calibration_manager.save(second, saved)
    # Changeover to it -> the Vision tab loads that recipe's transform.
    win.vision_tab.recipe_combo.setCurrentIndex(
        win.vision_tab.recipe_combo.findData(second)
    )
    assert win.vision_tab.active_recipe_key() == second
    assert win.vision_tab.calibration.pixel_to_robot(350, 250) == pytest.approx(
        (0.0, 250.0), abs=1e-3
    )


def test_changeover_applies_recipe_cover_size(qapp):
    win = MainWindow()
    # The active recipe's nominal cover size gates the Vision tab's detector.
    active = win.recipes.get(win.vision_tab.active_recipe_key())
    assert win.vision_tab.cover_detector.config.expected_diameter_mm == pytest.approx(
        active.cover_diameter_mm
    )


def test_adding_recipe_refreshes_vision_dropdown(qapp, tmp_path):
    from bung_cover_robot.vision.calibration import CalibrationManager

    win = MainWindow()
    win.recipes.path = tmp_path / "recipes.yaml"  # don't touch the real config
    win.calibration_manager = win.calibration_tab.manager = CalibrationManager(tmp_path)
    before = win.vision_tab.recipe_combo.count()
    win.calibration_tab.new_recipe_edit.setText("Group 65 8-vent")
    win.calibration_tab._on_add_recipe()
    # recipesChanged -> the Vision changeover dropdown gains the new recipe live.
    assert win.vision_tab.recipe_combo.count() == before + 1
    assert win.vision_tab.recipe_combo.findData("group-65-8-vent") >= 0


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


def test_vision_pick_roi_gates_and_persists(qapp, tmp_path):
    from bung_cover_robot.vision.calibration import CalibrationManager

    win = MainWindow()
    vt = win.vision_tab
    vt.calibration_manager = CalibrationManager(tmp_path)
    # the demo scene has a calibration, so drawing is enabled and clear is not
    vt.refresh()
    assert vt.draw_roi_btn.isEnabled()
    assert not vt.clear_roi_btn.isEnabled()

    key = vt.active_recipe_key()
    roi = (50, 40, 300, 260)
    vt._on_roi_changed(*roi)             # as RoiImageView.roiChanged would fire
    assert vt._pick_roi == roi
    assert vt.cover_detector.config.pick_roi == roi
    assert vt.clear_roi_btn.isEnabled()
    assert vt.calibration_manager.get_roi(key) == roi   # persisted per recipe

    # a fresh load re-applies the saved region to detector + view
    vt._apply_pick_roi(None)
    vt.view.set_roi(None)
    vt._load_pick_roi()
    assert vt.cover_detector.config.pick_roi == roi
    assert vt.view.roi() == roi

    # clearing removes it from the detector, the view, and disk
    vt._on_clear_roi()
    assert vt._pick_roi is None
    assert vt.cover_detector.config.pick_roi is None
    assert vt.calibration_manager.get_roi(key) is None
    assert not vt.clear_roi_btn.isEnabled()


def test_vision_pick_roi_draw_gated_on_calibration(qapp):
    win = MainWindow()
    vt = win.vision_tab
    vt.set_calibration(None)             # no calibration -> can't draw
    assert not vt.draw_roi_btn.isEnabled()
    vt._on_draw_roi()
    assert not vt.view.is_drawing()
    assert "Calibrate" in vt.status_label.text()


def test_roi_image_view_set_and_clear(qapp):
    from bung_cover_robot.gui.imaging import demo_frame, ndarray_to_qpixmap
    from bung_cover_robot.gui.widgets import RoiImageView

    v = RoiImageView()
    v.resize(400, 300)
    v.set_pixmap(ndarray_to_qpixmap(demo_frame(760, 520)))
    v.set_roi((10, 20, 100, 80))
    assert v.roi() == (10.0, 20.0, 100.0, 80.0)
    # source<->widget mapping round-trips
    wpt = v.source_to_widget(10.0, 20.0)
    assert wpt is not None
    back = v.widget_to_source(*wpt)
    assert back == pytest.approx((10.0, 20.0), abs=1.0)
    v.clear_roi()
    assert v.roi() is None


def test_robot_test_reset_button_and_fault_banner(qapp):
    from bung_cover_robot.app.robot_test_controller import RobotTestController
    from bung_cover_robot.gui.robot_test_tab import RobotTestTab
    from bung_cover_robot.plc import PlcRobotDriver, SimulatedPlcClient
    from bung_cover_robot.plc import tags as T
    from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics

    kin = FiveBarKinematics()
    jt = kin.inverse(0.0, 250.0)
    sim = SimulatedPlcClient(home_angles=(jt.left_deg, jt.right_deg)).connect()
    drv = PlcRobotDriver(sim, command_timeout_s=1.0, pulse_hold_s=0.0)
    tab = RobotTestTab(RobotTestController(drv, kin))

    # latch a homing fault as the PLC would
    sim.write(T.Status.FAULTED, True)
    sim.write(T.Status.FAULT_CODE, 4)
    tab._update_enable_state()
    # isHidden() reflects the explicit show/hide flag (isVisible() is False for an
    # unmapped widget in a headless test).
    assert not tab.fault_banner.isHidden()
    assert "FAULT 4" in tab.fault_banner.text()
    assert tab.reset_btn.isEnabled()          # reset offered
    assert not tab.enable_btn.isEnabled()     # enable blocked until reset

    # forcing an enable while faulted must not leave the button stuck highlighted
    tab.enable_btn.setChecked(True)
    tab._on_enable_toggled(True)
    assert not tab.enable_btn.isChecked()     # reverted to reality (still disabled)
    assert not tab.controller.is_enabled

    # reset clears the fault and re-arms enable
    tab._on_reset()
    assert not tab.controller.is_faulted
    assert tab.fault_banner.isHidden()
    assert tab.enable_btn.isEnabled()
    assert "reset ok" in tab.status_label.text().lower()


def test_robot_test_home_fault_shows_message(qapp):
    from bung_cover_robot.app.robot_test_controller import RobotTestController
    from bung_cover_robot.gui.robot_test_tab import RobotTestTab
    from bung_cover_robot.robot.driver import DryRunRobotDriver, RobotDriverError

    class FaultingHome(DryRunRobotDriver):
        def home(self):
            raise RobotDriverError("home / find reference: PLC faulted (code 4)")

    tab = RobotTestTab(RobotTestController(FaultingHome()))
    tab._on_enable_toggled(True)
    tab._on_home_reference()                  # must not raise; shows a message
    assert "code 4" in tab.status_label.text()
    assert not tab.controller.is_referenced


def test_bypass_tab_writes_bench_overrides_to_plc(qapp):
    from bung_cover_robot.app.robot_test_controller import RobotTestController
    from bung_cover_robot.gui.bypass_tab import BypassTab, SAFE_INPUTS
    from bung_cover_robot.plc import PlcRobotDriver, SimulatedPlcClient
    from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics

    kin = FiveBarKinematics()
    jt = kin.inverse(0.0, 250.0)
    sim = SimulatedPlcClient(home_angles=(jt.left_deg, jt.right_deg)).connect()
    tab = BypassTab(RobotTestController(PlcRobotDriver(sim), kin))
    tab.refresh()
    assert tab.force_safe_btn.isEnabled()          # a client is connected

    tab._on_force_safe()
    for tag, val in SAFE_INPUTS:                    # every safety input forced safe
        assert bool(sim.read(tag)) == val

    tab.homing_chk.setChecked(True)                 # toggled -> writes the tag
    assert bool(sim.read("Bypass_Homing"))
    tab.vision_chk.setChecked(True)
    assert bool(sim.read("Bypass_Vision"))
    tab.homing_chk.setChecked(False)
    assert not bool(sim.read("Bypass_Homing"))


def test_bypass_tab_disabled_without_plc(qapp):
    from bung_cover_robot.app.robot_test_controller import build_dry_run_controller
    from bung_cover_robot.gui.bypass_tab import BypassTab

    tab = BypassTab(build_dry_run_controller())   # dry-run driver has no client
    tab.refresh()
    assert not tab.force_safe_btn.isEnabled()
    assert not tab.homing_chk.isEnabled()
    assert "No PLC" in tab.status_label.text()
