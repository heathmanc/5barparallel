"""Headless backend construction for the CLI / GUI entry points (Claude.md §14).

Builds a `RobotTestController` for the requested motion backend — dry-run,
simulated PLC, or a real CompactLogix — from an optional ``robot_config.yaml``.
No Qt import, so it stays unit-testable.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from ..robot.driver import HomingConfig
from ..robot.fivebar_kinematics import FiveBarConfig, FiveBarKinematics
from .robot_test_controller import RobotTestController, build_dry_run_controller


def build_controller(
    *,
    config_path: Optional[str | Path] = None,
    sim_plc: bool = False,
    plc: Optional[str] = None,
) -> RobotTestController:
    """Build the controller for the selected backend.

    ``plc`` (an ``IP/slot`` path) drives a real CompactLogix; ``sim_plc`` runs the
    real PlcRobotDriver handshake against an in-memory PLC; otherwise a dry-run
    in-process driver is used. ``config_path`` is a ``robot_config.yaml`` file.
    """
    config = FiveBarConfig.from_yaml(config_path) if config_path else None
    homing = HomingConfig.from_yaml(config_path) if config_path else HomingConfig()

    if plc:
        from ..plc import CompactLogixClient, PlcRobotDriver

        kin = FiveBarKinematics(config) if config else FiveBarKinematics()
        driver = PlcRobotDriver(CompactLogixClient(plc))
        driver.connect()
        return RobotTestController(driver, kin, home_xy=homing.home_tcp_mm)

    if sim_plc:
        from ..plc import PlcRobotDriver, SimulatedPlcClient

        kin = FiveBarKinematics(config) if config else FiveBarKinematics()
        client = SimulatedPlcClient(home_angles=homing.home_angles).connect()
        driver = PlcRobotDriver(client)
        return RobotTestController(driver, kin, home_xy=homing.home_tcp_mm)

    return build_dry_run_controller(config=config, homing=homing)
