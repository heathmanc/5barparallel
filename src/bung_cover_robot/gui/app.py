"""GUI entry point.

    python -m bung_cover_robot.gui            # dry-run (no hardware)

Today only a dry-run driver exists, so the GUI always runs simulated. When a
PLC-backed RobotDriver lands, wire it here behind a --dry-run/--live flag.
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

from ..app.robot_test_controller import RobotTestController
from ..robot.driver import DryRunRobotDriver


def run_gui(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="bung_cover_robot.gui")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        default=True,
        help="run with the simulated driver (default; only mode available today)",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    # Import Qt lazily so `--help` and import of this module don't require a
    # display/Qt libraries.
    from PySide6.QtWidgets import QApplication

    from .main_window import MainWindow

    app = QApplication.instance() or QApplication(sys.argv[:1])
    controller = RobotTestController(DryRunRobotDriver())
    window = MainWindow(controller)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run_gui())
