"""Main application window — a tabbed dark-theme HMI.

Vision (main screen) · Camera · Robot Test · Settings · PLC.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtWidgets import QMainWindow, QTabWidget, QWidget

from ..app.recipes import RecipeStore
from ..app.robot_test_controller import RobotTestController, build_dry_run_controller
from ..vision.calibration import CalibrationManager, CameraIntrinsics
from ..vision.camera import Camera, CameraConfig, MockCamera
from .calibration_tab import CalibrationTab
from .camera_tab import CameraTab
from .imaging import demo_frame, demo_transform
from .plc_tab import PlcTab
from .robot_test_tab import RobotTestTab
from .settings_tab import SettingsTab
from .vision_tab import VisionTab

_CONFIG_DIR = Path(__file__).resolve().parents[3] / "config"


def _demo_camera() -> Camera:
    cam = MockCamera(
        CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
    )
    return cam.open()


def _load_intrinsics() -> Optional[CameraIntrinsics]:
    """Best-effort load of lens intrinsics from config/camera_config.yaml."""
    try:
        return CameraIntrinsics.from_yaml(_CONFIG_DIR / "camera_config.yaml")
    except (OSError, ValueError, KeyError):
        return None


def _load_recipes() -> RecipeStore:
    try:
        return RecipeStore.load(_CONFIG_DIR / "recipes.yaml")
    except (OSError, ValueError, KeyError):
        return RecipeStore()


class MainWindow(QMainWindow):
    def __init__(
        self,
        controller: Optional[RobotTestController] = None,
        camera: Optional[Camera] = None,
        config_path: Optional[str | Path] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("5-Bar Bung-Cover Robot — HMI")
        self.resize(1160, 780)

        self.controller = controller or build_dry_run_controller()
        self.camera = camera or _demo_camera()
        self.calibration_manager = CalibrationManager(intrinsics=_load_intrinsics())
        self.recipes = _load_recipes()

        self.tabs = QTabWidget()
        self.vision_tab = VisionTab(self.controller, self.camera, None, self.recipes)
        self.camera_tab = CameraTab(self.camera)
        self.calibration_tab = CalibrationTab(
            self.camera, self.calibration_manager, self.recipes
        )
        self.robot_test_tab = RobotTestTab(self.controller)
        self.settings_tab = SettingsTab(self.controller, config_path=config_path)
        self.plc_tab = PlcTab(self.controller)
        self.tabs.addTab(self.vision_tab, "Vision")
        self.tabs.addTab(self.camera_tab, "Camera")
        self.tabs.addTab(self.calibration_tab, "Calibration")
        self.tabs.addTab(self.robot_test_tab, "Robot Test")
        self.tabs.addTab(self.settings_tab, "Settings")
        self.tabs.addTab(self.plc_tab, "PLC")
        self.setCentralWidget(self.tabs)

        # Cross-tab refresh.
        self.settings_tab.geometryChanged.connect(self.robot_test_tab.refresh_all)
        self.plc_tab.connectionChanged.connect(self.robot_test_tab.refresh_all)
        self.plc_tab.connectionChanged.connect(self.vision_tab.refresh)
        self.camera_tab.cameraChanged.connect(self._on_camera_changed)
        self.calibration_tab.calibrationSaved.connect(self._on_calibration_saved)
        self.calibration_tab.recipesChanged.connect(self.vision_tab.reload_recipes)
        self.vision_tab.recipeChanged.connect(self._apply_recipe)
        self.tabs.currentChanged.connect(self._on_tab_changed)

        # Load the first recipe's calibration into the Vision tab.
        if self.recipes.list():
            self._apply_recipe(self.recipes.list()[0].key)

    def _apply_recipe(self, key: str) -> None:
        """Changeover: load the recipe's hole count + its own calibration."""
        recipe = self.recipes.get(key)
        self.vision_tab.set_hole_count(recipe.hole_count)
        self.vision_tab.set_cover_diameter_mm(recipe.cover_diameter_mm)
        self.vision_tab.set_calibration(self._recipe_calibration(key))
        self.vision_tab.refresh()

    def _recipe_calibration(self, key: str):
        """The recipe's saved calibration, or a demo transform for the mock scene
        so reachability is visible before a real calibration exists."""
        if self.calibration_manager.has(key):
            return self.calibration_manager.get(key)
        if isinstance(self.camera, MockCamera):
            return demo_transform()
        return None

    def _on_camera_changed(self) -> None:
        self.camera = self.camera_tab.camera
        self.vision_tab.set_camera(self.camera)
        self.calibration_tab.set_camera(self.camera)
        key = self.vision_tab.active_recipe_key()
        if key is not None:
            self.vision_tab.set_calibration(self._recipe_calibration(key))

    def _on_calibration_saved(self, key: str, transform) -> None:
        """A recipe was (re)calibrated — adopt it live if it's the active one."""
        if key == self.vision_tab.active_recipe_key():
            self.vision_tab.set_calibration(transform)
            self.vision_tab.refresh()

    def _on_tab_changed(self, index: int) -> None:
        if self.tabs.widget(index) is self.vision_tab:
            self.vision_tab.refresh()
