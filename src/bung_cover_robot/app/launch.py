"""Headless backend construction for the CLI / GUI entry points.

Builds a `RobotTestController` from an optional ``robot_config.yaml``. No Qt
import, so it stays unit-testable.

Backends, all behind the same ``RobotDriver`` seam:
  * dry-run (default)      — in-process instant driver, no motion stack.
  * ``sim_ec=True``        — the real EtherCatRobotDriver against a simulated A6
                             network (exercises CiA 402 + CSP streaming, no HW).
  * ``ethercat=True``      — the real pysoem master (Stage 4; needs drives + RT).
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from ..robot.driver import HomingConfig
from ..robot.fivebar_kinematics import FiveBarConfig, FiveBarKinematics
from ..robot.workspace import WorkspaceValidator
from .robot_test_controller import RobotTestController, build_dry_run_controller


def build_controller(
    *,
    config_path: Optional[str | Path] = None,
    sim_ec: bool = False,
    ethercat: bool = False,
) -> RobotTestController:
    """Build the controller for the selected backend.

    ``config_path`` is a ``robot_config.yaml`` file (geometry + homing).
    """
    config = FiveBarConfig.from_yaml(config_path) if config_path else None
    homing = HomingConfig.from_yaml(config_path) if config_path else HomingConfig()

    if sim_ec or ethercat:
        from ..ethercat import EtherCatRobotDriver, SimulatedEtherCatMaster

        kin = FiveBarKinematics(config) if config else FiveBarKinematics()
        validator = WorkspaceValidator(kin)
        if ethercat:  # pragma: no cover - real hardware path (Stage 4)
            from ..ethercat.master import PysoemMaster

            master = PysoemMaster().open()
        else:
            master = SimulatedEtherCatMaster().open()
        driver = EtherCatRobotDriver(
            master, kin, validator, home_angles=homing.home_angles
        ).connect()
        return RobotTestController(driver, kin, validator, home_xy=homing.home_tcp_mm)

    return build_dry_run_controller(config=config, homing=homing)
