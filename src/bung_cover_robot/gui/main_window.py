"""Main application window — a tabbed HMI. Only the Robot Test tab exists today;
Vision/Calibration/Production tabs slot in alongside it later."""

from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import QMainWindow, QTabWidget, QWidget

from ..app.robot_test_controller import RobotTestController
from ..robot.driver import DryRunRobotDriver, RobotDriver
from .robot_test_tab import RobotTestTab


class MainWindow(QMainWindow):
    def __init__(
        self,
        controller: Optional[RobotTestController] = None,
        driver: Optional[RobotDriver] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("5-Bar Bung-Cover Robot")
        self.resize(720, 560)

        if controller is None:
            controller = RobotTestController(driver or DryRunRobotDriver())
        self.controller = controller

        self.tabs = QTabWidget()
        self.robot_test_tab = RobotTestTab(controller)
        self.tabs.addTab(self.robot_test_tab, "Robot Test")
        self.setCentralWidget(self.tabs)
