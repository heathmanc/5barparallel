"""Vision layer — camera, calibration, and OpenCV detection (Claude.md §12-§14).

Implemented so far: camera capture + control (Basler via pypylon, plus a mock
for --dry-run). Calibration and detection are still to build.
"""

from .camera import (
    CONTROL_REGISTRY,
    BaslerCamera,
    Camera,
    CameraConfig,
    CameraConnectionError,
    CameraControlError,
    CameraControls,
    CameraError,
    CameraGrabError,
    MockCamera,
    open_camera,
)

__all__ = [
    "CONTROL_REGISTRY",
    "BaslerCamera",
    "Camera",
    "CameraConfig",
    "CameraConnectionError",
    "CameraControlError",
    "CameraControls",
    "CameraError",
    "CameraGrabError",
    "MockCamera",
    "open_camera",
]
