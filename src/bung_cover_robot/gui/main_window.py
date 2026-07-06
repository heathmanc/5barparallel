"""Main application window — a tabbed dark-theme HMI.

Vision (main screen) · Camera · Robot Test · Settings · PLC.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtWidgets import QMainWindow, QTabWidget, QWidget

from ..app.robot_test_controller import RobotTestController, build_dry_run_controller
from ..vision.camera import Camera, CameraConfig, MockCamera
from .camera_tab import CameraTab
from .imaging import demo_frame, demo_transform
from .plc_tab import PlcTab
from .robot_test_tab import RobotTestTab
from .settings_tab import SettingsTab
from .vision_tab import VisionTab


def _demo_camera() -> Camera:
    cam = MockCamera(
        CameraConfig(mock_width=760, mock_height=520), frames=[demo_frame(760, 520)]
    )
    return cam.open()


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

        # A demo pixel->robot transform lets the Vision tab show reachability with
        # the mock scene; a real Basler needs a real calibration (cleared on swap).
        calibration = demo_transform() if isinstance(self.camera, MockCamera) else None

        self.tabs = QTabWidget()
        self.vision_tab = VisionTab(self.controller, self.camera, calibration)
        self.camera_tab = CameraTab(self.camera)
        self.robot_test_tab = RobotTestTab(self.controller)
        self.settings_tab = SettingsTab(self.controller, config_path=config_path)
        self.plc_tab = PlcTab(self.controller)
        self.tabs.addTab(self.vision_tab, "Vision")
        self.tabs.addTab(self.camera_tab, "Camera")
        self.tabs.addTab(self.robot_test_tab, "Robot Test")
        self.tabs.addTab(self.settings_tab, "Settings")
        self.tabs.addTab(self.plc_tab, "PLC")
        self.setCentralWidget(self.tabs)

        # Cross-tab refresh.
        self.settings_tab.geometryChanged.connect(self.robot_test_tab.refresh_all)
        self.plc_tab.connectionChanged.connect(self.robot_test_tab.refresh_all)
        self.plc_tab.connectionChanged.connect(self.vision_tab.refresh)
        self.camera_tab.cameraChanged.connect(self._on_camera_changed)
        self.tabs.currentChanged.connect(self._on_tab_changed)

    def _on_camera_changed(self) -> None:
        self.camera = self.camera_tab.camera
        self.vision_tab.set_camera(self.camera)
        # The demo calibration only matches the mock scene.
        self.vision_tab.set_calibration(
            demo_transform() if isinstance(self.camera, MockCamera) else None
        )

    def _on_tab_changed(self, index: int) -> None:
        if self.tabs.widget(index) is self.vision_tab:
            self.vision_tab.refresh()
