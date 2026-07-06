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
from .detect_covers import (
    CoverDetection,
    CoverDetectionResult,
    CoverDetector,
    CoverDetectorConfig,
)
from .detect_holes import HoleDetectionResult, HoleDetector, HoleDetectorConfig
from .detection import Circle, annotate, find_battery_roi
from .calibration import (
    CalibrationError,
    CalibrationManager,
    CameraIntrinsics,
    HomographyTransform,
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
    "CameraIntrinsics",
    "CalibrationError",
    "CalibrationManager",
    "Circle",
    "CoverDetection",
    "CoverDetectionResult",
    "CoverDetector",
    "CoverDetectorConfig",
    "HoleDetectionResult",
    "HoleDetector",
    "HoleDetectorConfig",
    "HomographyTransform",
    "MockCamera",
    "annotate",
    "find_battery_roi",
    "open_camera",
]
