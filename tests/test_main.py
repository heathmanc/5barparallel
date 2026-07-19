"""CLI entry point: arg parsing, backend/camera selection, window wiring."""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("PySide6")

from PySide6.QtWidgets import QApplication  # noqa: E402

from bung_cover_robot import main as M  # noqa: E402
from bung_cover_robot.app.launch import build_controller  # noqa: E402
from bung_cover_robot.robot.driver import DryRunRobotDriver  # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    return QApplication.instance() or QApplication([])


# --------------------------------------------------------------------------- #
# arg parsing / resolution
# --------------------------------------------------------------------------- #
def test_parse_defaults():
    a = M.parse_args([])
    assert a.camera == "auto"


def test_camera_mode_auto_follows_backend():
    # --camera auto always means the mock demo scene now (no PLC backend).
    assert M._camera_mode(M.parse_args([])) == "mock"
    assert M._camera_mode(M.parse_args(["--camera", "basler"])) == "basler"
    assert M._camera_mode(M.parse_args(["--camera", "mock"])) == "mock"


def test_config_dir_accepts_dir_or_file(tmp_path):
    assert M._config_dir(M.parse_args(["--config", str(tmp_path)])) == tmp_path
    f = tmp_path / "robot_config.yaml"
    f.write_text("geometry: {}\n")
    assert M._config_dir(M.parse_args(["--config", str(f)])) == tmp_path


# --------------------------------------------------------------------------- #
# backend / camera construction
# --------------------------------------------------------------------------- #
def test_build_controller_backends():
    # Only the dry-run backend exists now; config_path is optional.
    assert isinstance(build_controller().driver, DryRunRobotDriver)
    assert isinstance(build_controller(config_path=None).driver, DryRunRobotDriver)


def test_ethercat_num_drives_reads_settings(tmp_path):
    # --ethercat drive count comes from app_settings (IgH gets its NIC from
    # ethercat.conf, so no interface name is needed).
    from bung_cover_robot.app.launch import _ethercat_num_drives

    (tmp_path / "app_settings.yaml").write_text("ethercat_num_drives: 1\n")
    cfg = tmp_path / "robot_config.yaml"
    cfg.write_text("{}\n")
    assert _ethercat_num_drives(cfg) == 1


def test_build_camera_mock_is_none():
    # mock mode leaves camera to the window (which builds the demo scene)
    assert M.build_camera(M.parse_args([])) is None


# --------------------------------------------------------------------------- #
# end-to-end window wiring (no event loop)
# --------------------------------------------------------------------------- #
def test_main_builds_window(qapp, monkeypatch):
    monkeypatch.setattr(QApplication, "exec", lambda self: 0)
    built = {}
    import bung_cover_robot.gui.main_window as mw

    orig = mw.MainWindow.show

    def spy(self):
        built["window"] = self
        return orig(self)

    monkeypatch.setattr(mw.MainWindow, "show", spy)
    assert M.main([]) == 0
    win = built["window"]
    assert isinstance(win.controller.driver, DryRunRobotDriver)  # dry-run backend
    assert win.recipes.list()                                    # recipes loaded


def test_main_window_reads_config_dir(qapp, tmp_path):
    import yaml

    (tmp_path / "recipes.yaml").write_text(
        yaml.safe_dump({"recipes": [{"key": "custom-1", "name": "Custom", "hole_count": 4}]})
    )
    from bung_cover_robot.gui.main_window import MainWindow

    win = MainWindow(config_dir=tmp_path)
    assert win.recipes.has("custom-1")
    assert win.vision_tab.recipe_combo.findData("custom-1") >= 0
    assert win.vision_tab.hole_detector.config.expected_count == 4  # applied on load
