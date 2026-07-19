"""Offscreen smoke test for the Qt tabs.

Runs headless via the Qt 'offscreen' platform. Skipped if PySide6 isn't
installed, so the rest of the suite doesn't depend on Qt.
"""

import os
import time

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication  # noqa: E402

from bung_cover_robot.app.robot_test_controller import (  # noqa: E402
    build_dry_run_controller,
)
from bung_cover_robot.gui.camera_tab import CameraTab  # noqa: E402
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
        "Drives",
        "Settings",
    ]


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
    assert "covers" in vt.status_label.text()      # Hough cover detection
    assert vt.cover_detector.config.method == "hough"


def test_vision_tab_start_enabled_when_idle_and_running(qapp):
    win = MainWindow()
    vt = win.vision_tab
    win.controller.enable()
    win.controller.home_reference()
    vt.refresh()
    # No Auto/Manual gate now: Cycle Start is available when idle, Stop is not.
    assert vt.start_btn.isEnabled()
    assert not vt.stop_btn.isEnabled()
    # While a cycle is running, Start is inert and Stop is offered.
    vt._running = True
    vt._sync_run_buttons()
    assert not vt.start_btn.isEnabled()
    assert vt.stop_btn.isEnabled()
    vt._running = False


def test_vision_tab_start_requires_enable_and_home(qapp):
    win = MainWindow()
    vt = win.vision_tab
    vt._on_start()                            # not enabled/referenced yet
    assert "Cannot start" in vt.status_label.text()


def _wait_until(qapp, predicate, timeout_s=5.0):
    import time

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not predicate():
        qapp.processEvents()
    return predicate()


def test_camera_settings_save_and_restore(qapp, tmp_path):
    win = MainWindow(config_dir=tmp_path)
    ct = win.camera_tab
    ct._sliders["brightness"].setValue(40)     # 40/100 = 0.4
    ct._sliders["contrast"].setValue(150)      # 150/100 = 1.5
    ct._on_save_settings()
    assert (tmp_path / "camera_settings.yaml").exists()
    # a fresh Camera tab restores the saved slider values
    ct2 = CameraTab(win.camera, settings=win.app_settings, config_dir=tmp_path)
    assert ct2._sliders["brightness"].value() == 40
    assert ct2._sliders["contrast"].value() == 150


def test_vision_live_position_readout(qapp):
    import re

    win = MainWindow()
    vt = win.vision_tab
    # not referenced -> dashes + hint
    vt._on_position(None)
    assert "—" in vt.pos_deg_label.text()
    assert "not referenced" in vt.pos_xy_label.text().lower()
    # a referenced pose -> live angles + forward-kinematics TCP in mm
    win.controller.enable()
    win.controller.home_reference()
    angles = vt._read_angles()
    assert angles is not None
    vt._on_position(angles)
    assert re.search(r"\d", vt.pos_deg_label.text())      # shows angle numbers
    assert "mm" in vt.pos_xy_label.text()
    assert re.search(r"\d", vt.pos_xy_label.text())       # shows an X/Y value


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


def test_camera_tab_live_grabber_streams_frames(qapp):
    win = MainWindow()
    ct = win.camera_tab
    ct._frame = None
    ct._visible = True
    ct._start_live()               # background grabber streams the mock frame
    try:
        deadline = time.monotonic() + 3.0
        while ct._frame is None and time.monotonic() < deadline:
            qapp.processEvents()
            time.sleep(0.02)
        assert ct._frame is not None          # a live frame arrived on the GUI thread
        assert "live" in ct.info_label.text()
    finally:
        ct._stop_live()
    assert ct._grabber is None                # stops cleanly


def test_camera_tab_auto_exposure_toggle(qapp):
    win = MainWindow()
    ct = win.camera_tab
    # Auto on by default -> exposure slider disabled, ExposureAuto=Continuous.
    assert ct._auto_checks["exposure_time_us"].isChecked()
    assert not ct._sliders["exposure_time_us"].isEnabled()
    assert win.camera.get_control("exposure_auto") == "Continuous"
    # Turning Auto off enables the manual slider and writes ExposureAuto=Off.
    ct._auto_checks["exposure_time_us"].setChecked(False)
    assert ct._sliders["exposure_time_us"].isEnabled()
    assert win.camera.get_control("exposure_auto") == "Off"


def test_jog_disabled_until_enabled_and_referenced(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    assert all(not b.isEnabled() for b in tab._jog_buttons)
    tab.enable_btn.click()
    assert tab.controller.is_enabled
    assert all(not b.isEnabled() for b in tab._jog_buttons)  # still not referenced
    tab._on_home_reference()
    tab._await_command()
    assert tab.controller.is_referenced
    assert tab.referenced_label.text() == "REFERENCED"
    assert all(b.isEnabled() for b in tab._jog_buttons)


def test_reference_then_jog_updates_readout(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    tab.enable_btn.click()
    tab._on_home_reference()
    tab._await_command()
    tab.joint_step.setValue(1.0)
    before = tab._value_labels["left_deg"].text()
    tab._jog_joint("left", +1)
    tab._await_command()
    assert tab._value_labels["left_deg"].text() != before
    assert "OK" in tab.status_label.text()


def test_referenced_label_tracks_live_reference_loss(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    tab.enable_btn.click()
    tab._on_home_reference()
    tab._await_command()
    assert tab.referenced_label.text() == "REFERENCED"
    # A disable loses the datum; the live poll (_update_enable_state, not a
    # command's _refresh) must update the label, not just the buttons.
    tab.controller.disable()
    tab._update_enable_state()
    assert tab.referenced_label.text() == "NOT REFERENCED"
    assert all(not b.isEnabled() for b in tab._jog_buttons)


def test_rejected_move_shows_reason(qapp):
    tab = RobotTestTab(build_dry_run_controller())
    tab.enable_btn.click()
    tab._on_home_reference()
    tab._await_command()
    tab.cart_step.setValue(50.0)
    for _ in range(5):
        tab._jog_cart("y", +1)
        tab._await_command()
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


def test_zoompan_view_roundtrip_and_click(qapp):
    from PySide6.QtGui import QPixmap
    from bung_cover_robot.gui.widgets import ZoomPanImageView

    v = ZoomPanImageView()
    v.resize(400, 300)
    v.set_pixmap(QPixmap(800, 600))
    got = []
    v.pixelClicked.connect(lambda x, y: got.append((x, y)))
    # place_source_point is the test seam: emits in source coords, no geometry.
    v.place_source_point(123.0, 456.0)
    assert got and got[-1] == (123.0, 456.0)
    # source<->widget round-trips exactly across zoom + pan states.
    for z in (1.0, 3.0, 12.0):
        v._z = z
        v._clamp_pan()
        w = v.source_to_widget(400, 300)
        b = v.widget_to_source(w[0], w[1])
        assert b == pytest.approx((400.0, 300.0), abs=1e-6)
    v.one_to_one()
    assert v._scale() == pytest.approx(1.0, abs=1e-6)  # 1:1 -> one screen px/source px
    v.fit()
    assert v._z == pytest.approx(1.0)
    v.set_loupe(False)
    v.set_points([(10, 10), (700, 500)])
    v.set_active(1)  # must not raise without a shown widget


def test_calibration_coach_advances_through_steps(qapp, tmp_path):
    tab, _, _ = _cal_tab(tmp_path)
    # Recipe selected (default) + frame captured on construct + 0 points -> mark step.
    assert tab._coach_step(0) == 3
    # No recipe -> step 1; no frame -> step 2.
    tab.recipe_combo.setCurrentIndex(-1)
    assert tab._coach_step(0) == 1
    tab.recipe_combo.setCurrentIndex(0)
    saved_frame = tab._frame
    tab._frame = None
    assert tab._coach_step(0) == 2
    tab._frame = saved_frame
    # 4 complete points -> fit; after fit -> review; after save -> done.
    pix, rob = _rect_correspondences()
    _click_points(tab, pix, rob)
    assert tab._coach_step(4) == 4
    tab._on_fit()
    assert tab._coach_step(4) == 5
    tab._on_save()
    assert tab._coach_step(4) == 6


def test_calibration_recipe_params_editor(qapp, tmp_path):
    tab, _, recipes = _cal_tab(tmp_path)
    key = tab._selected_recipe_key()
    # Editor populated from the selected recipe (defaults: 6 holes, 18 mm).
    assert tab.hole_count_spin.value() == recipes.get(key).hole_count
    # Edit + save -> persisted onto the recipe (upsert).
    tab.hole_count_spin.setValue(8)
    tab.bung_dia_spin.setValue(20.0)
    tab.tol_spin.setValue(15)  # percent -> 0.15
    tab._on_save_recipe_params()
    r = recipes.get(key)
    assert r.hole_count == 8
    assert r.cover_diameter_mm == pytest.approx(20.0)
    assert r.diameter_tolerance == pytest.approx(0.15)


def test_vision_single_step_builds_one_hole_config(qapp):
    win = MainWindow()
    vt = win.vision_tab
    assert hasattr(vt, "single_step_chk")
    vt.single_step_chk.setChecked(True)
    # The recipe->detector tolerance wiring is live (cover detector configured).
    assert vt.cover_detector.config.diameter_tolerance <= 0.25


def test_calibration_help_html_has_key_sections():
    from bung_cover_robot.gui.calibration_help import HELP_HTML

    for needle in ("homography", "residual", "parallax", "corners"):
        assert needle.lower() in HELP_HTML.lower()


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
    assert st._floats["l1_mm"].value() == pytest.approx(200.0)
    assert st._floats["l2_mm"].value() == pytest.approx(230.0)


def test_settings_apply_valid_geometry(qapp):
    win = MainWindow()
    st = win.settings_tab
    st._floats["l1_mm"].setValue(225.0)
    assert st._on_apply() is True
    assert win.controller.kin.config.l1_mm == pytest.approx(225.0)


def test_settings_refuses_when_home_unreachable(qapp):
    win = MainWindow()
    st = win.settings_tab
    before = win.controller.kin.config.l1_mm
    # A tiny robot can't reach the default home (0, 250); Apply must refuse it
    # (rather than silently accepting an unreferenceable home).
    st._floats["l1_mm"].setValue(60.0)
    st._floats["l2_mm"].setValue(60.0)
    assert st._on_apply() is False
    assert "Refused" in st.status_label.text()
    assert win.controller.kin.config.l1_mm == before  # unchanged


def test_settings_small_robot_with_reachable_home_applies(qapp):
    win = MainWindow()
    st = win.settings_tab
    # Shrink the robot AND move the home somewhere the small arms can reach.
    st._floats["l1_mm"].setValue(100.0)
    st._floats["l2_mm"].setValue(115.0)
    st._floats["base_spacing_mm"].setValue(40.0)
    st._floats["home_x"].setValue(0.0)
    st._floats["home_y"].setValue(120.0)
    assert st._on_apply() is True
    assert win.controller.kin.config.l1_mm == pytest.approx(100.0)
    assert win.controller.home_xy == pytest.approx((0.0, 120.0))
    assert "Work area" in st.status_label.text()


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


def test_vision_tuning_sliders_and_save_frame(qapp, tmp_path, monkeypatch):
    from PySide6.QtWidgets import QFileDialog

    win = MainWindow()
    vt = win.vision_tab
    vt._capture()
    assert vt._frame is not None

    # sliders push straight into the Hough config and re-detect (no crash)
    vt.tune_max.setValue(360)
    assert vt.cover_detector.config.max_diameter_px == 360
    vt.tune_edge.setValue(95)                     # more sensitive => softer edge
    assert vt.cover_detector.config.hough_param1 < 60
    vt.tune_votes.setValue(50)
    assert vt.cover_detector.config.hough_param2 == pytest.approx(50)

    # Save frame writes a PNG at the chosen path
    out = tmp_path / "frame.png"
    monkeypatch.setattr(
        QFileDialog, "getSaveFileName", lambda *a, **k: (str(out), "PNG (*.png)"))
    vt._on_save_frame()
    assert out.exists()


def test_vision_save_diagnostics_writes_png_and_report(qapp, tmp_path, monkeypatch):
    from PySide6.QtWidgets import QFileDialog

    win = MainWindow()
    vt = win.vision_tab
    vt._on_detect()                          # populate last-detection results

    out = tmp_path / "diag.png"
    monkeypatch.setattr(
        QFileDialog, "getSaveFileName", lambda *a, **k: (str(out), "PNG (*.png)"))
    vt._on_save_diag()
    assert out.exists()
    report = (tmp_path / "diag.txt")
    assert report.exists()
    text = report.read_text()
    assert "Cover detector:" in text and "Hole detector:" in text
    assert "Covers:" in text                 # detection section present


def test_holes_searched_full_frame_outside_pick_region(qapp):
    win = MainWindow()
    vt = win.vision_tab
    # Drop-hole search must not confine to a bright battery blob (that grabs the
    # covers); it spans the frame, keeps the straight row, and skips the chute.
    hc = vt.hole_detector.config
    assert hc.auto_battery_roi is False
    assert hc.select_collinear_subset is True
    # Drawing a pick region wires it into the hole detector as an exclusion zone.
    vt._apply_pick_roi((10, 20, 100, 80))
    assert vt.hole_detector.config.exclude_roi == (10, 20, 100, 80)


def test_detection_sliders_save_and_load_per_recipe(qapp, tmp_path):
    from bung_cover_robot.app.recipes import RecipeStore

    win = MainWindow(config_dir=tmp_path)          # recipes.yaml persists under tmp
    vt = win.vision_tab
    key = win.recipes.list()[0].key
    vt.select_recipe(key)
    win._apply_recipe(key)

    vt.tune_min.setValue(300)
    vt.tune_hole_max.setValue(180)
    vt._save_tuning_to_recipe()                    # normally on sliderReleased

    # persisted into the recipe + written to disk
    assert win.recipes.get(key).cover_min_px == 300
    reloaded = RecipeStore.load(tmp_path / "recipes.yaml").get(key)
    assert reloaded.cover_min_px == 300 and reloaded.hole_max_px == 180

    # a changeover restores the saved windows into the sliders and detectors
    vt._set_slider(vt.tune_min, 250)               # scramble first
    vt.set_detection_tuning(reloaded)
    assert vt.tune_min.value() == 300 and vt.tune_hole_max.value() == 180
    assert vt.cover_detector.config.min_diameter_px == pytest.approx(300)
    assert vt.hole_detector.config.max_diameter_px == pytest.approx(180)


def test_calibration_param_save_preserves_detection_tuning(qapp, tmp_path):
    win = MainWindow(config_dir=tmp_path)
    vt, ct = win.vision_tab, win.calibration_tab
    key = win.recipes.list()[0].key
    vt.select_recipe(key)
    win._apply_recipe(key)

    # dial a custom cover window on the Vision sliders and save it to the recipe
    vt.tune_min.setValue(310)
    vt._save_tuning_to_recipe()
    assert win.recipes.get(key).cover_min_px == 310

    # editing the diameters/count in the Calibration tab must NOT reset the px tuning
    ct.recipe_combo.setCurrentIndex(ct.recipe_combo.findData(key))
    ct.bung_dia_spin.setValue(20.0)
    ct._on_save_recipe_params()
    assert win.recipes.get(key).cover_min_px == 310          # tuning preserved
    assert win.recipes.get(key).cover_diameter_mm == 20.0    # edit applied
    assert vt.tune_min.value() == 310                        # slider didn't revert


def test_hole_size_sliders_drive_hole_detector(qapp):
    win = MainWindow()
    vt = win.vision_tab
    vt._capture()
    # Drop holes have their own pixel-Ø window, independent of the cover size.
    vt.tune_hole_min.setValue(40)
    vt.tune_hole_max.setValue(180)
    assert vt.hole_detector.config.min_diameter_px == pytest.approx(40)
    assert vt.hole_detector.config.max_diameter_px == pytest.approx(180)
    # a recipe re-apply rebuilds the detector but keeps the slider window
    vt.set_hole_count(6, diameter_mm=0.0)
    assert vt.hole_detector.config.max_diameter_px == pytest.approx(180)


def test_recipe_save_propagates_bung_size_to_detector(qapp):
    win = MainWindow()
    ct, vt = win.calibration_tab, win.vision_tab
    key = win.recipes.list()[0].key

    ct.recipe_combo.setCurrentIndex(ct.recipe_combo.findData(key))
    ct.bung_dia_spin.setValue(22.5)
    ct.hole_dia_spin.setValue(16.0)        # separate drop-hole size
    ct.hole_count_spin.setValue(6)
    ct.tol_spin.setValue(20)               # 20 %
    ct._on_save_recipe_params()

    # the saved recipe is now active in the Vision tab and drives both detectors
    assert vt.active_recipe_key() == key
    assert vt.cover_detector.config.expected_diameter_mm == pytest.approx(22.5)
    assert vt.cover_detector.config.diameter_tolerance == pytest.approx(0.20)
    assert vt.hole_detector.config.expected_diameter_mm == pytest.approx(16.0)


def test_vision_pick_roi_draw_without_calibration(qapp):
    win = MainWindow()
    vt = win.vision_tab
    vt.set_calibration(None)             # a pixel box needs no calibration
    vt._update_roi_buttons()
    assert vt.draw_roi_btn.isEnabled()   # camera is open -> can draw
    vt._on_draw_roi()
    assert vt.view.is_drawing()
    assert vt._frame is not None


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
    tab._await_command()
    assert "code 4" in tab.status_label.text()
    assert not tab.controller.is_referenced


def test_drives_tab_sim_connect_params_and_disconnect(qapp, tmp_path):
    from bung_cover_robot.ethercat.ethercat_driver import EtherCatRobotDriver
    from bung_cover_robot.gui.ethercat_tab import EtherCatTab
    from bung_cover_robot.robot.driver import DryRunRobotDriver

    ctrl = build_dry_run_controller()
    tab = EtherCatTab(ctrl, settings=None, config_dir=tmp_path)
    # connect the simulated network -> real EtherCAT driver behind the seam
    tab._on_connect_sim()
    assert isinstance(ctrl.driver, EtherCatRobotDriver)
    ctrl.enable()
    ctrl.home_reference()
    # snapshot path: encoder counts + CiA402 state render into the panels
    master = ctrl.driver.master
    snap = [dict(sw=d.statusword, mode=d.mode_display, act=d.actual_position,
                 tgt=d.target_position, di=d.digital_inputs) for d in master.drives]
    tab._on_snapshot(snap)
    assert "OPERATION ENABLED" in tab._drive_panels[0][1]["state"].text()
    assert "counts" in tab._drive_panels[0][1]["counts"].text()
    # parameter table: edit a value, save -> YAML persisted with the new value
    tab.table.item(0, 1).setText("150")        # speed_mm_s
    tab._on_save_params()
    assert (tmp_path / "drive_parameters.yaml").exists()
    from bung_cover_robot.ethercat.parameters import ParameterStore
    assert ParameterStore.load(tmp_path / "drive_parameters.yaml").get("speed_mm_s") == 150.0
    # apply pushes motion limits into the live driver
    tab._on_apply_params()
    assert ctrl.driver.limits.speed_mm_s == 150.0
    # disconnect falls back to the dry-run driver
    tab._on_disconnect()
    assert isinstance(ctrl.driver, DryRunRobotDriver)
    tab._stop_poller()


def test_drives_tab_single_axis_bench_mode(qapp, tmp_path):
    """Drives=1 (single-axis bench): the master has one drive, panel 0 shows it
    live, and the absent second panel is marked 'not on bus' — no stale data."""
    from bung_cover_robot.gui.ethercat_tab import EtherCatTab

    ctrl = build_dry_run_controller()
    tab = EtherCatTab(ctrl, settings=None, config_dir=tmp_path)
    tab.drives_spin.setValue(1)
    tab._on_connect_sim()
    assert ctrl.driver.master.num_drives == 1
    snap = [dict(sw=d.statusword, mode=d.mode_display, act=d.actual_position,
                 tgt=d.target_position, di=d.digital_inputs)
            for d in ctrl.driver.master.drives]
    assert len(snap) == 1
    tab._on_snapshot(snap)
    assert "counts" in tab._drive_panels[0][1]["counts"].text()
    assert "not on bus" in tab._drive_panels[1][1]["state"].text()
    tab._on_disconnect()
    tab._stop_poller()


def test_drives_tab_bench_jog(qapp, tmp_path):
    """Single-axis bench jog: connect (sim, 1 drive), enable, jog +, position moves."""
    from bung_cover_robot.gui.ethercat_tab import EtherCatTab

    ctrl = build_dry_run_controller()
    tab = EtherCatTab(ctrl, settings=None, config_dir=tmp_path)
    tab.drives_spin.setValue(1)
    tab._on_connect_sim()
    tab._on_enable()
    assert ctrl.driver.is_enabled
    tab.jog_step.setValue(1500)
    tab._on_jog(+1)                       # runs on a worker thread now
    assert tab._jog_worker.wait(2000)
    qapp.processEvents()
    assert ctrl.driver.master.drives[0].actual_position == 1500
    tab._on_jog(-1)
    assert tab._jog_worker.wait(2000)
    qapp.processEvents()
    assert ctrl.driver.master.drives[0].actual_position == 0
    tab._on_disconnect()
    tab._stop_poller()


def test_drives_tab_jog_targets_selected_axis(qapp, tmp_path):
    """Two-drive bench: the axis selector routes the jog to that drive only, so
    each motor can be confirmed to map to the axis you expect."""
    from bung_cover_robot.gui.ethercat_tab import EtherCatTab

    ctrl = build_dry_run_controller()
    tab = EtherCatTab(ctrl, settings=None, config_dir=tmp_path)
    tab.drives_spin.setValue(2)
    tab._on_connect_sim()
    tab._on_enable()
    assert ctrl.driver.master.num_drives == 2
    tab.jog_step.setValue(1500)
    tab.jog_axis.setValue(1)              # jog the second drive
    tab._on_jog(+1)
    assert tab._jog_worker.wait(2000)
    qapp.processEvents()
    drives = ctrl.driver.master.drives
    assert drives[1].actual_position == 1500   # axis 1 moved
    assert drives[0].actual_position == 0       # axis 0 held
    tab._on_disconnect()
    tab._stop_poller()


def test_drives_tab_disconnect_disables_and_stops_master(qapp, tmp_path):
    """Safety: disconnect (and app close) must disable the drive and close the
    master, not just swap drivers."""
    from bung_cover_robot.gui.ethercat_tab import EtherCatTab
    from bung_cover_robot.robot.driver import DryRunRobotDriver

    ctrl = build_dry_run_controller()
    tab = EtherCatTab(ctrl, settings=None, config_dir=tmp_path)
    tab.drives_spin.setValue(1)
    tab._on_connect_sim()
    tab._on_enable()
    master = ctrl.driver.master
    assert ctrl.driver.is_enabled
    tab._on_disconnect()
    assert not master.is_open                       # master/daemon stopped
    assert isinstance(ctrl.driver, DryRunRobotDriver)
    # shutdown() is safe to call again (idempotent, not connected)
    tab.shutdown()
