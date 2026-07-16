"""GUI entry point.

    python -m bung_cover_robot.gui                       # dry-run sim driver
    python -m bung_cover_robot.gui --sim-ec              # EtherCAT vs simulated A6
    python -m bung_cover_robot.gui --config config/robot_config.yaml

--dry-run (default) is the in-process driver; --sim-ec runs the real
EtherCatRobotDriver against an in-memory A6 network (no hardware).
"""

from __future__ import annotations

import argparse
import logging
import sys
from typing import List, Optional

from ..app.launch import build_controller
from ..app.robot_test_controller import RobotTestController


def _build_controller(args) -> RobotTestController:
    return build_controller(
        config_path=args.config, sim_ec=args.sim_ec, ethercat=args.ethercat
    )


def run_gui(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="bung_cover_robot.gui")
    parser.add_argument("--config", help="path to robot_config.yaml")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="in-process sim (default)")
    mode.add_argument("--sim-ec", action="store_true",
                      help="EtherCAT driver against a simulated A6 network (no HW)")
    mode.add_argument("--ethercat", action="store_true",
                      help="real EtherCAT drives over pysoem (Stage 4)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    # Import Qt lazily so --help doesn't require a display/Qt libraries.
    from PySide6.QtWidgets import QApplication

    from .main_window import MainWindow
    from .theme import apply_theme

    app = QApplication.instance() or QApplication(sys.argv[:1])
    apply_theme(app)
    controller = _build_controller(args)
    window = MainWindow(controller, config_path=args.config)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(run_gui())
