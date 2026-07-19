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
from .app_settings import AppSettings
from .robot_test_controller import RobotTestController, build_dry_run_controller

_CONFIG_DIR = Path(__file__).resolve().parents[3] / "config"


def _ethercat_num_drives(config_path: Optional[str | Path]) -> int:
    """Drive count for --ethercat, from app_settings (what the Drives tab saves).
    The IgH master gets its NIC from ethercat.conf, so no interface is needed."""
    cfg_dir = Path(config_path).parent if config_path else _CONFIG_DIR
    settings = AppSettings.load(cfg_dir / "app_settings.yaml")
    try:
        n_drives = int(settings.get("ethercat_num_drives", 2) or 2)
    except (TypeError, ValueError):
        n_drives = 2
    return max(1, min(2, n_drives))


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
        if ethercat:
            from ..ethercat.igh_master import IgHMaster

            n_drives = _ethercat_num_drives(config_path)
            master = IgHMaster(  # pragma: no cover - needs real drives + IgH
                num_drives=n_drives).open()
        else:
            master = SimulatedEtherCatMaster().open()
        driver = EtherCatRobotDriver(
            master, kin, validator, home_angles=homing.home_angles
        ).connect()
        return RobotTestController(driver, kin, validator, home_xy=homing.home_tcp_mm)

    return build_dry_run_controller(config=config, homing=homing)
