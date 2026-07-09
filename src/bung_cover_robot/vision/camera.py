"""Basler camera capture + control, with an OpenCV-native frame interface.

Design (Claude.md §12, §15):
  * Native Basler controls (exposure, gain, brightness, contrast, gamma, ROI,
    ...) are only reachable through Basler's GenICam node map, so the real
    camera is driven by Basler's `pypylon` SDK. `pypylon` is imported lazily so
    this module (and its tests) work on machines without the SDK or a camera.
  * Every grabbed frame is returned as an OpenCV-native `numpy` array — BGR
    (H, W, 3) for color output or (H, W) for Mono8 — so detection, calibration,
    and undistortion consume it directly.
  * `MockCamera` provides synthetic frames + in-memory controls so the whole
    pipeline runs with `--dry-run` and no hardware.

Control names are *logical* (e.g. "brightness", "exposure_time_us"). They are
resolved to the correct GenICam node for the connected model via CONTROL_REGISTRY
— brightness is `BslBrightness` on ace2/dart but `Brightness` on older aces,
exposure is `ExposureTime` on SFNC 2.x but `ExposureTimeAbs` on SFNC 1.x, etc.
You may also pass a raw GenICam node name; it is used as-is if it isn't a known
logical name.
"""

from __future__ import annotations

import logging
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------- #
# Exceptions
# --------------------------------------------------------------------------- #
class CameraError(Exception):
    """Base class for all camera failures."""


class CameraConnectionError(CameraError):
    """Camera could not be found, opened, or the SDK is missing."""


class CameraGrabError(CameraError):
    """A frame could not be acquired (timeout, incomplete grab, ...)."""


class CameraControlError(CameraError):
    """A control could not be read/written (absent, read-only, out of range)."""


# --------------------------------------------------------------------------- #
# Logical control -> candidate GenICam node names, in resolution order.
# The first candidate that exists (and is writable) on the connected camera
# wins. This absorbs naming differences across Basler families/SFNC versions.
# --------------------------------------------------------------------------- #
CONTROL_REGISTRY: Dict[str, List[str]] = {
    "exposure_time_us": ["ExposureTime", "ExposureTimeAbs"],
    "exposure_auto": ["ExposureAuto"],
    "gain": ["Gain", "GainRaw"],
    "gain_auto": ["GainAuto"],
    "brightness": ["BslBrightness", "Brightness"],
    "contrast": ["BslContrast", "Contrast"],
    "gamma": ["Gamma"],
    "black_level": ["BlackLevel", "BlackLevelRaw"],
    "saturation": ["BslSaturation", "Saturation"],
    "sharpness": ["BslSharpnessEnhancement", "SharpnessEnhancement"],
    "white_balance_auto": ["BalanceWhiteAuto"],
    "pixel_format": ["PixelFormat"],
    "reverse_x": ["ReverseX"],
    "reverse_y": ["ReverseY"],
    "offset_x": ["OffsetX"],
    "offset_y": ["OffsetY"],
    "width": ["Width"],
    "height": ["Height"],
    "frame_rate_enable": ["AcquisitionFrameRateEnable"],
    "frame_rate": ["AcquisitionFrameRate", "AcquisitionFrameRateAbs"],
}

# Controls are applied in this order: geometry and auto-mode toggles first, so
# that manual exposure/gain actually take effect and the ROI is set before grab.
_APPLY_ORDER: Tuple[str, ...] = (
    "pixel_format",
    "reverse_x",
    "reverse_y",
    "offset_x",
    "offset_y",
    "width",
    "height",
    "exposure_auto",
    "gain_auto",
    "white_balance_auto",
    "exposure_time_us",
    "gain",
    "brightness",
    "contrast",
    "gamma",
    "black_level",
    "saturation",
    "sharpness",
    "frame_rate_enable",
    "frame_rate",
)


# --------------------------------------------------------------------------- #
# Config / controls dataclasses
# --------------------------------------------------------------------------- #
@dataclass
class CameraControls:
    """Desired imaging controls. Every field is optional — a value of ``None``
    means "leave the camera's current setting alone"."""

    exposure_auto: Optional[str] = None          # Off | Once | Continuous
    exposure_time_us: Optional[float] = None
    gain_auto: Optional[str] = None
    gain: Optional[float] = None
    brightness: Optional[float] = None
    contrast: Optional[float] = None
    gamma: Optional[float] = None
    black_level: Optional[float] = None
    saturation: Optional[float] = None
    sharpness: Optional[float] = None
    white_balance_auto: Optional[str] = None
    pixel_format: Optional[str] = None
    reverse_x: Optional[bool] = None
    reverse_y: Optional[bool] = None
    offset_x: Optional[int] = None
    offset_y: Optional[int] = None
    width: Optional[int] = None
    height: Optional[int] = None
    frame_rate_enable: Optional[bool] = None
    frame_rate: Optional[float] = None
    # Anything not covered by a named field: raw GenICam node name -> value.
    extra: Dict[str, Any] = field(default_factory=dict)

    def as_ordered_items(self) -> List[Tuple[str, Any]]:
        """(control_name, value) pairs for every set field, in apply order."""
        items: List[Tuple[str, Any]] = []
        for name in _APPLY_ORDER:
            value = getattr(self, name, None)
            if value is not None:
                items.append((name, value))
        for name, value in self.extra.items():
            if value is not None:
                items.append((name, value))
        return items

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraControls":
        known = {f for f in cls.__dataclass_fields__ if f != "extra"}
        kwargs = {k: v for k, v in data.items() if k in known}
        extra = {k: v for k, v in data.items() if k not in known}
        return cls(extra=extra, **kwargs)

    def merged_with(self, other: "CameraControls") -> "CameraControls":
        """A copy with every set (non-None) field of ``other`` overriding self —
        so a saved operator settings file overlays the tracked config defaults."""
        import dataclasses

        updates = {
            f.name: getattr(other, f.name)
            for f in dataclasses.fields(other)
            if f.name != "extra" and getattr(other, f.name) is not None
        }
        merged = dataclasses.replace(self, **updates)
        merged.extra = {**self.extra, **other.extra}
        return merged

    def to_settings_dict(self) -> Dict[str, Any]:
        """The set fields as a plain dict for YAML persistence."""
        items = {name: value for name, value in self.as_ordered_items()}
        return items

    @classmethod
    def from_yaml(cls, path: str | Path) -> "CameraControls":
        import yaml

        data = yaml.safe_load(Path(path).read_text()) or {}
        return cls.from_dict(data.get("controls", {}))


@dataclass
class CameraConfig:
    """Device-level camera settings."""

    serial_number: Optional[str] = None
    device_index: int = 0
    output_pixel_format: str = "BGR8"   # BGR8 (color) or Mono8 (gray)
    grab_timeout_ms: int = 5000
    mock_width: int = 2592
    mock_height: int = 1944

    @classmethod
    def from_yaml(cls, path: str | Path) -> "CameraConfig":
        import yaml

        data = yaml.safe_load(Path(path).read_text()) or {}
        sec = data.get("camera", {})
        base = cls()
        return cls(
            serial_number=sec.get("serial_number", base.serial_number),
            device_index=int(sec.get("device_index", base.device_index)),
            output_pixel_format=str(
                sec.get("output_pixel_format", base.output_pixel_format)
            ),
            grab_timeout_ms=int(sec.get("grab_timeout_ms", base.grab_timeout_ms)),
            mock_width=int(sec.get("mock_width", base.mock_width)),
            mock_height=int(sec.get("mock_height", base.mock_height)),
        )


# --------------------------------------------------------------------------- #
# Camera interface
# --------------------------------------------------------------------------- #
class Camera(ABC):
    """Common interface for the real and mock cameras."""

    config: CameraConfig

    # --- lifecycle ---
    @abstractmethod
    def open(self) -> "Camera":
        """Connect and prepare for grabbing. Idempotent."""

    @abstractmethod
    def close(self) -> None:
        """Stop grabbing and release the device. Idempotent."""

    @property
    @abstractmethod
    def is_open(self) -> bool:
        ...

    def __enter__(self) -> "Camera":
        return self.open()

    def __exit__(self, *exc: object) -> None:
        self.close()

    # --- capture ---
    @abstractmethod
    def grab(self, timeout_ms: Optional[int] = None) -> np.ndarray:
        """Return the latest frame as an OpenCV BGR (H, W, 3) or Mono8 (H, W)
        uint8 array. Raises CameraGrabError on failure."""

    # --- controls ---
    @abstractmethod
    def set_control(self, name: str, value: Any) -> str:
        """Write one control. ``name`` is a logical name (CONTROL_REGISTRY) or a
        raw GenICam node name. Returns the node name that was written. Raises
        CameraControlError if unavailable/read-only/out of range."""

    @abstractmethod
    def get_control(self, name: str) -> Any:
        """Read one control's current value."""

    def apply_controls(self, controls: CameraControls) -> Dict[str, Any]:
        """Apply every set control, in a safe order. Returns {name: value} for
        the controls that were successfully applied. Logs and skips controls the
        connected model does not expose rather than aborting the batch."""
        applied: Dict[str, Any] = {}
        for name, value in controls.as_ordered_items():
            try:
                self.set_control(name, value)
                applied[name] = value
            except CameraControlError as exc:
                logger.warning("skipping control %s=%r: %s", name, value, exc)
        return applied

    # --- diagnostics ---
    def save_frame(self, frame: np.ndarray, path: str | Path) -> Path:
        """Write a frame to disk (Claude.md §15 diagnostics)."""
        import cv2

        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(out), frame):
            raise CameraError(f"failed to write frame to {out}")
        return out

    def _require_open(self) -> None:
        if not self.is_open:
            raise CameraConnectionError("camera is not open; call open() first")


# --------------------------------------------------------------------------- #
# Real Basler camera (pypylon)
# --------------------------------------------------------------------------- #
class BaslerCamera(Camera):
    """Basler camera driven through pypylon. Frames are converted to the
    configured OpenCV pixel format on grab."""

    def __init__(self, config: Optional[CameraConfig] = None) -> None:
        self.config = config or CameraConfig()
        self._camera = None      # pylon.InstantCamera
        self._converter = None   # pylon.ImageFormatConverter
        self._grabbing = False
        # pypylon's InstantCamera/node map are NOT thread-safe. A live-preview
        # grabber thread and the GUI thread (writing controls) both touch the
        # camera, so serialize every SDK call behind this lock.
        self._lock = threading.RLock()

    # --- lazy SDK import ---
    @staticmethod
    def _import_pylon():
        try:
            from pypylon import genicam, pylon
        except ImportError as exc:  # pragma: no cover - depends on host SDK
            raise CameraConnectionError(
                "pypylon is not installed. Install Basler pylon + "
                "`pip install pypylon` to use BaslerCamera, or use MockCamera / "
                "--dry-run."
            ) from exc
        return pylon, genicam

    @classmethod
    def list_devices(cls) -> List[Dict[str, str]]:
        """Enumerate connected Basler cameras (serial, model, ...)."""
        pylon, _ = cls._import_pylon()
        tl = pylon.TlFactory.GetInstance()
        out: List[Dict[str, str]] = []
        for dev in tl.EnumerateDevices():
            out.append(
                {
                    "serial": dev.GetSerialNumber(),
                    "model": dev.GetModelName(),
                    "vendor": dev.GetVendorName(),
                    "full_name": dev.GetFullName(),
                }
            )
        return out

    # --- lifecycle ---
    def open(self) -> "BaslerCamera":
        if self.is_open:
            return self
        pylon, _ = self._import_pylon()
        tl = pylon.TlFactory.GetInstance()
        devices = tl.EnumerateDevices()
        if not devices:
            raise CameraConnectionError("no Basler cameras found")

        device = self._select_device(devices)
        try:
            self._camera = pylon.InstantCamera(tl.CreateDevice(device))
            self._camera.Open()
        except Exception as exc:  # pragma: no cover - hardware path
            raise CameraConnectionError(f"failed to open Basler camera: {exc}") from exc

        self._auto_select_mono()
        self._converter = self._build_converter(pylon)
        logger.info(
            "opened Basler %s (serial %s)",
            device.GetModelName(),
            device.GetSerialNumber(),
        )
        return self

    def _select_device(self, devices):
        if self.config.serial_number is not None:
            wanted = str(self.config.serial_number)
            for dev in devices:
                if dev.GetSerialNumber() == wanted:
                    return dev
            raise CameraConnectionError(
                f"no Basler camera with serial {wanted!r} "
                f"(found: {[d.GetSerialNumber() for d in devices]})"
            )
        idx = self.config.device_index
        if idx < 0 or idx >= len(devices):
            raise CameraConnectionError(
                f"device_index {idx} out of range (found {len(devices)} camera(s))"
            )
        return devices[idx]

    def _auto_select_mono(self) -> None:
        """A mono sensor cannot produce color, so force Mono8 output (and set the
        sensor to Mono8): converting a mono frame to BGR8 triples the data, costs a
        color conversion per frame, and gives a washed-out 'grayscale'. Detected by
        the absence of any color PixelFormat entry. Best-effort; on any hiccup the
        configured output format is left as-is."""
        _, genicam = self._import_pylon()
        try:
            nodemap = self._camera.GetNodeMap()
            pf = nodemap.GetNode("PixelFormat")
            if pf is None or not genicam.IsAvailable(pf):
                return
            entries = []
            for e in pf.GetEntries():
                try:
                    entries.append(e.GetSymbolic())
                except Exception:  # noqa: BLE001
                    continue
            color = ("Bayer", "RGB", "BGR", "YUV", "YCbCr", "YCC")
            if any(any(c in name for c in color) for name in entries):
                return  # a color camera — keep the configured output format
            self.config.output_pixel_format = "Mono8"
            if genicam.IsWritable(pf) and any("Mono8" == n for n in entries):
                pf.FromString("Mono8")
            logger.info("mono camera detected -> Mono8 output")
        except Exception:  # noqa: BLE001 - detection must never break open()
            pass

    def _build_converter(self, pylon):
        converter = pylon.ImageFormatConverter()
        if self.config.output_pixel_format.upper() == "MONO8":
            converter.OutputPixelFormat = pylon.PixelType_Mono8
        else:
            converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        return converter

    def close(self) -> None:
        with self._lock:
            if self._camera is None:
                return
            try:
                if self._grabbing:
                    self._camera.StopGrabbing()
                if self._camera.IsOpen():
                    self._camera.Close()
            finally:
                self._grabbing = False
                self._camera = None
                self._converter = None

    @property
    def is_open(self) -> bool:
        return self._camera is not None and self._camera.IsOpen()

    # --- capture ---
    def grab(self, timeout_ms: Optional[int] = None) -> np.ndarray:
        pylon, _ = self._import_pylon()
        timeout = self.config.grab_timeout_ms if timeout_ms is None else timeout_ms
        with self._lock:
            self._require_open()
            if not self._grabbing:
                self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                self._grabbing = True
            result = self._camera.RetrieveResult(
                timeout, pylon.TimeoutHandling_ThrowException
            )
            try:
                if not result.GrabSucceeded():
                    raise CameraGrabError(
                        f"grab failed: {result.GetErrorCode()} "
                        f"{result.GetErrorDescription()}"
                    )
                image = self._converter.Convert(result)
                # Copy: the converted buffer is owned by pylon, reused next grab.
                return np.array(image.GetArray(), copy=True)
            finally:
                result.Release()

    # --- controls ---
    def set_control(self, name: str, value: Any) -> str:
        _, genicam = self._import_pylon()
        with self._lock:
            self._require_open()
            nodemap = self._camera.GetNodeMap()
            last_err: Optional[str] = None
            for node_name in CONTROL_REGISTRY.get(name, [name]):
                node = nodemap.GetNode(node_name)
                if node is None or not genicam.IsAvailable(node):
                    continue
                if not genicam.IsWritable(node):
                    last_err = f"{node_name} is not writable"
                    continue
                self._write_node(genicam, node, node_name, value)
                return node_name
            raise CameraControlError(
                f"control {name!r} not available on this camera"
                + (f" ({last_err})" if last_err else "")
            )

    def get_control(self, name: str) -> Any:
        _, genicam = self._import_pylon()
        with self._lock:
            self._require_open()
            nodemap = self._camera.GetNodeMap()
            for node_name in CONTROL_REGISTRY.get(name, [name]):
                node = nodemap.GetNode(node_name)
                if node is None or not genicam.IsReadable(node):
                    continue
                if isinstance(node, genicam.IEnumeration):
                    return node.ToString()
                return node.GetValue()
            raise CameraControlError(f"control {name!r} not readable on this camera")

    def control_range(self, name: str) -> Tuple[float, float]:
        """(min, max) for a numeric control — useful for building UIs/sliders."""
        _, genicam = self._import_pylon()
        with self._lock:
            self._require_open()
            nodemap = self._camera.GetNodeMap()
            for node_name in CONTROL_REGISTRY.get(name, [name]):
                node = nodemap.GetNode(node_name)
                if node is None or not genicam.IsReadable(node):
                    continue
                if isinstance(node, (genicam.IFloat, genicam.IInteger)):
                    return float(node.GetMin()), float(node.GetMax())
            raise CameraControlError(f"control {name!r} has no numeric range")

    @staticmethod
    def _write_node(genicam, node, node_name: str, value: Any) -> None:
        """Type-dispatch a write, clamping numerics into the node's valid range."""
        if isinstance(node, genicam.IEnumeration):
            node.FromString(str(value))
        elif isinstance(node, genicam.IBoolean):
            node.SetValue(bool(value))
        elif isinstance(node, genicam.IFloat):
            lo, hi = node.GetMin(), node.GetMax()
            node.SetValue(min(max(float(value), lo), hi))
        elif isinstance(node, genicam.IInteger):
            lo, hi = node.GetMin(), node.GetMax()
            iv = min(max(int(value), lo), hi)
            inc = node.GetInc()
            if inc and inc > 1:
                iv -= (iv - lo) % inc  # snap to the node's increment grid
            node.SetValue(iv)
        else:  # pragma: no cover - unusual node types
            node.FromString(str(value))


# --------------------------------------------------------------------------- #
# Mock camera (dry-run / tests)
# --------------------------------------------------------------------------- #
class MockCamera(Camera):
    """Synthetic camera for --dry-run and tests. Stores controls in memory and
    returns a deterministic frame (or frames you supply)."""

    def __init__(
        self,
        config: Optional[CameraConfig] = None,
        frames: Optional[List[np.ndarray]] = None,
    ) -> None:
        self.config = config or CameraConfig()
        self._open = False
        self._controls: Dict[str, Any] = {}
        self._frames = frames
        self._frame_idx = 0

    def open(self) -> "MockCamera":
        self._open = True
        return self

    def close(self) -> None:
        self._open = False

    @property
    def is_open(self) -> bool:
        return self._open

    def grab(self, timeout_ms: Optional[int] = None) -> np.ndarray:
        self._require_open()
        if self._frames:
            frame = self._frames[self._frame_idx % len(self._frames)]
            self._frame_idx += 1
            return np.array(frame, copy=True)
        return self._synthetic_frame()

    def _synthetic_frame(self) -> np.ndarray:
        h, w = self.config.mock_height, self.config.mock_width
        # A mid-gray field with a smooth horizontal gradient — deterministic,
        # no RNG, enough structure to exercise a processing pipeline.
        ramp = np.linspace(60, 200, w, dtype=np.uint8)
        gray = np.broadcast_to(ramp, (h, w)).astype(np.uint8)
        if self.config.output_pixel_format.upper() == "MONO8":
            return gray.copy()
        return np.repeat(gray[:, :, None], 3, axis=2)  # (H, W, 3) BGR

    def set_control(self, name: str, value: Any) -> str:
        self._require_open()
        self._controls[name] = value
        return name

    def get_control(self, name: str) -> Any:
        self._require_open()
        if name not in self._controls:
            raise CameraControlError(f"control {name!r} was never set on MockCamera")
        return self._controls[name]

    @property
    def controls(self) -> Dict[str, Any]:
        """All controls set so far (test/introspection helper)."""
        return dict(self._controls)


# --------------------------------------------------------------------------- #
# Factory
# --------------------------------------------------------------------------- #
def open_camera(
    config: Optional[CameraConfig] = None,
    controls: Optional[CameraControls] = None,
    *,
    mock: bool = False,
) -> Camera:
    """Open a camera and apply controls.

    ``mock=True`` (or the app's --dry-run) returns a MockCamera so the pipeline
    runs with no hardware. Otherwise a BaslerCamera is opened; if pypylon/the
    camera is missing a CameraConnectionError is raised with guidance.
    """
    cam: Camera = MockCamera(config) if mock else BaslerCamera(config)
    cam.open()
    if controls is not None:
        cam.apply_controls(controls)
    return cam
