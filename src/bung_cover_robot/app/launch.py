"""Headless backend construction for the CLI / GUI entry points.

Builds a `RobotTestController` from an optional ``robot_config.yaml``. No Qt
import, so it stays unit-testable.

The motion backend is the dry-run (in-process) driver today. The EtherCAT backend
(pysoem master + A6 servo drives) slots in here once Stage 3 lands, selected by an
``--ethercat`` / ``--sim-ec`` flag, behind the same ``RobotDriver`` seam.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from ..robot.driver import HomingConfig
from ..robot.fivebar_kinematics import FiveBarConfig
from .robot_test_controller import RobotTestController, build_dry_run_controller


def build_controller(
    *,
    config_path: Optional[str | Path] = None,
) -> RobotTestController:
    """Build the controller for the selected backend.

    ``config_path`` is a ``robot_config.yaml`` file (geometry + homing). Only the
    dry-run backend exists today; the EtherCAT backend will be added here.
    """
    config = FiveBarConfig.from_yaml(config_path) if config_path else None
    homing = HomingConfig.from_yaml(config_path) if config_path else HomingConfig()
    return build_dry_run_controller(config=config, homing=homing)
