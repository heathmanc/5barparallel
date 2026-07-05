"""Main application window — a tabbed HMI. Robot Test + Settings today;
Vision/Calibration/Production tabs slot in alongside them later."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtWidgets import QMainWindow, QTabWidget, QWidget

from ..app.robot_test_controller import RobotTestController, build_dry_run_controller
from .robot_test_tab import RobotTestTab
from .settings_tab import SettingsTab


class MainWindow(QMainWindow):
    def __init__(
        self,
        controller: Optional[RobotTestController] = None,
        config_path: Optional[str | Path] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("5-Bar Bung-Cover Robot")
        self.resize(760, 640)

        self.controller = controller or build_dry_run_controller()

        self.tabs = QTabWidget()
        self.robot_test_tab = RobotTestTab(self.controller)
        self.settings_tab = SettingsTab(self.controller, config_path=config_path)
        self.tabs.addTab(self.robot_test_tab, "Robot Test")
        self.tabs.addTab(self.settings_tab, "Settings")
        self.setCentralWidget(self.tabs)

        # Applying new geometry in Settings must refresh the Robot Test readout.
        self.settings_tab.geometryChanged.connect(self.robot_test_tab.refresh_all)
