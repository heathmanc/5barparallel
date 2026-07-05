"""PySide6 GUI for the robot HMI (Claude.md §14 app layer).

Run with:  python -m bung_cover_robot.gui

The GUI imports PySide6 lazily inside run_gui(), so importing this package (e.g.
in a headless test collector) does not require Qt to be installed.
"""

from .app import run_gui

__all__ = ["run_gui"]
