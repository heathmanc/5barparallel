"""5-Bar bung-cover robot — command-line entry point (Claude.md §14).

    bung-cover-robot                         # dry-run: sim driver + demo camera
    bung-cover-robot --sim-plc               # PLC handshake vs in-memory PLC
    bung-cover-robot --plc 192.168.1.10/0    # real CompactLogix
    bung-cover-robot --camera basler         # real Basler (else the demo scene)
    bung-cover-robot --config /path/to/config

Motion backend and camera are chosen independently. ``--camera auto`` (the
default) uses a real Basler when a real PLC is selected, otherwise the mock demo
scene, so ``--dry-run`` and ``--sim-plc`` run with no hardware at all. ``--config``
is the directory holding robot_config.yaml, camera_config.yaml, and recipes.yaml.
"""

from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import replace
from pathlib import Path
from typing import List, Optional

from .app.launch import build_controller
from .vision.camera import Camera, CameraConfig, CameraControls, open_camera

_DEFAULT_CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="bung-cover-robot")
    parser.add_argument(
        "--config", metavar="DIR", type=Path, default=_DEFAULT_CONFIG_DIR,
        help="config directory (robot_config.yaml, camera_config.yaml, recipes.yaml)",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="in-process sim (default)")
    mode.add_argument("--sim-plc", action="store_true", help="PLC driver + simulated PLC")
    mode.add_argument("--plc", metavar="IP/SLOT", help="PLC driver + real CompactLogix")
    parser.add_argument(
        "--camera", choices=("auto", "mock", "basler"), default="auto",
        help="camera backend (auto: basler with --plc, else mock)",
    )
    parser.add_argument("--camera-serial", help="target a specific Basler by serial")
    parser.add_argument("-v", "--verbose", action="store_true")
    return parser.parse_args(argv)


def _config_dir(args: argparse.Namespace) -> Path:
    """Resolve --config to a directory (accepts a file path too)."""
    p = Path(args.config)
    return p.parent if p.is_file() else p


def _robot_config_path(config_dir: Path) -> Optional[Path]:
    rc = config_dir / "robot_config.yaml"
    return rc if rc.exists() else None


def _camera_mode(args: argparse.Namespace) -> str:
    if args.camera != "auto":
        return args.camera
    return "basler" if args.plc else "mock"


def build_camera(args: argparse.Namespace) -> Optional[Camera]:
    """Real Basler for camera mode 'basler'; None for 'mock' so the window builds
    the demo-scene camera itself."""
    if _camera_mode(args) == "mock":
        return None
    config_dir = _config_dir(args)
    cam_cfg = config_dir / "camera_config.yaml"
    config = CameraConfig.from_yaml(cam_cfg) if cam_cfg.exists() else CameraConfig()
    controls = CameraControls.from_yaml(cam_cfg) if cam_cfg.exists() else CameraControls()
    # Operator's saved runtime settings (from the Camera tab) overlay the tracked
    # config defaults, so exposure/gain/etc. restore across launches.
    settings_file = config_dir / "camera_settings.yaml"
    if settings_file.exists():
        controls = controls.merged_with(CameraControls.from_yaml(settings_file))
        saved_serial = CameraConfig.from_yaml(settings_file).serial_number
        if saved_serial:
            config = replace(config, serial_number=saved_serial)
    if args.camera_serial:
        config = replace(config, serial_number=args.camera_serial)
    return open_camera(config, controls)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    config_dir = _config_dir(args)
    controller = build_controller(
        config_path=_robot_config_path(config_dir),
        sim_plc=args.sim_plc,
        plc=args.plc,
    )
    camera = build_camera(args)

    # Import Qt lazily so --help works with no display / Qt libraries.
    from PySide6.QtWidgets import QApplication

    from .gui.main_window import MainWindow
    from .gui.theme import apply_theme

    app = QApplication.instance() or QApplication(sys.argv[:1])
    apply_theme(app)
    window = MainWindow(controller, camera=camera, config_dir=config_dir)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
