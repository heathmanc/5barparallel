"""Camera-layer tests.

Exercise the parts that don't need real hardware: control dataclasses, the
control-name registry, and the MockCamera used for --dry-run. The pypylon path
in BaslerCamera is covered only for its no-SDK error behavior.
"""

from pathlib import Path

import numpy as np
import pytest

from bung_cover_robot.vision import (
    CONTROL_REGISTRY,
    BaslerCamera,
    CameraConfig,
    CameraConnectionError,
    CameraControlError,
    CameraControls,
    MockCamera,
    open_camera,
)

CAMERA_CONFIG = Path(__file__).resolve().parents[1] / "config" / "camera_config.yaml"


# --------------------------------------------------------------------------- #
# CameraControls
# --------------------------------------------------------------------------- #
def test_controls_from_yaml():
    controls = CameraControls.from_yaml(CAMERA_CONFIG)
    assert controls.exposure_auto == "Off"
    assert controls.exposure_time_us == 8000.0
    assert controls.brightness == 0.0
    assert controls.contrast == 1.0
    assert controls.gamma == 1.0


def test_config_from_yaml():
    cfg = CameraConfig.from_yaml(CAMERA_CONFIG)
    assert cfg.output_pixel_format == "BGR8"
    assert cfg.mock_width == 2592
    assert cfg.mock_height == 1944


def test_unset_controls_are_skipped():
    controls = CameraControls(brightness=0.2, exposure_time_us=5000.0)
    names = [name for name, _ in controls.as_ordered_items()]
    assert names == ["exposure_time_us", "brightness"]  # apply order, no Nones


def test_apply_order_puts_auto_modes_before_manual_values():
    controls = CameraControls(
        exposure_time_us=5000.0, exposure_auto="Off", gain=2.0, gain_auto="Off"
    )
    names = [name for name, _ in controls.as_ordered_items()]
    assert names.index("exposure_auto") < names.index("exposure_time_us")
    assert names.index("gain_auto") < names.index("gain")


def test_extra_controls_pass_through():
    controls = CameraControls.from_dict(
        {"brightness": 0.1, "BslHue": 10, "DeviceLinkThroughputLimit": 100_000_000}
    )
    assert controls.brightness == 0.1
    assert controls.extra == {"BslHue": 10, "DeviceLinkThroughputLimit": 100_000_000}
    items = dict(controls.as_ordered_items())
    assert items["BslHue"] == 10


# --------------------------------------------------------------------------- #
# Control registry
# --------------------------------------------------------------------------- #
def test_registry_covers_the_headline_controls():
    for name in ("brightness", "contrast", "exposure_time_us", "gain", "gamma"):
        assert name in CONTROL_REGISTRY
    # Model-name aliases are present in resolution order.
    assert CONTROL_REGISTRY["brightness"] == ["BslBrightness", "Brightness"]
    assert CONTROL_REGISTRY["exposure_time_us"][0] == "ExposureTime"


# --------------------------------------------------------------------------- #
# MockCamera
# --------------------------------------------------------------------------- #
def test_mock_grab_returns_bgr_frame():
    cam = MockCamera(CameraConfig(mock_width=64, mock_height=48))
    with cam:
        frame = cam.grab()
    assert frame.shape == (48, 64, 3)
    assert frame.dtype == np.uint8


def test_mock_grab_mono():
    cam = MockCamera(
        CameraConfig(mock_width=32, mock_height=24, output_pixel_format="Mono8")
    )
    cam.open()
    assert cam.grab().shape == (24, 32)
    cam.close()


def test_mock_grab_requires_open():
    cam = MockCamera()
    with pytest.raises(CameraConnectionError):
        cam.grab()


def test_mock_supplied_frames_cycle():
    a = np.zeros((4, 4, 3), np.uint8)
    b = np.full((4, 4, 3), 255, np.uint8)
    cam = MockCamera(frames=[a, b])
    cam.open()
    assert cam.grab()[0, 0, 0] == 0
    assert cam.grab()[0, 0, 0] == 255
    assert cam.grab()[0, 0, 0] == 0  # wraps
    cam.close()


def test_mock_set_get_control():
    cam = MockCamera().open()
    cam.set_control("brightness", 0.3)
    assert cam.get_control("brightness") == 0.3
    with pytest.raises(CameraControlError):
        cam.get_control("never_set")
    cam.close()


def test_apply_controls_records_everything_on_mock():
    cam = MockCamera().open()
    applied = cam.apply_controls(
        CameraControls(brightness=0.1, contrast=1.2, exposure_time_us=9000.0)
    )
    assert applied == {
        "exposure_time_us": 9000.0,
        "contrast": 1.2,
        "brightness": 0.1,
    }
    assert cam.controls["brightness"] == 0.1
    cam.close()


def test_save_frame(tmp_path):
    cam = MockCamera(CameraConfig(mock_width=16, mock_height=16)).open()
    frame = cam.grab()
    out = cam.save_frame(frame, tmp_path / "diag" / "frame.png")
    assert out.exists()
    cam.close()


# --------------------------------------------------------------------------- #
# Factory / Basler no-SDK behavior
# --------------------------------------------------------------------------- #
def test_open_camera_mock_applies_controls():
    cam = open_camera(
        CameraConfig(mock_width=8, mock_height=8),
        CameraControls(brightness=0.5),
        mock=True,
    )
    assert isinstance(cam, MockCamera)
    assert cam.is_open
    assert cam.get_control("brightness") == 0.5
    cam.close()


def test_basler_without_pypylon_raises_clear_error():
    # pypylon isn't installed in this environment; the error must be actionable.
    pytest.importorskip  # noqa: B018 - documents intent; we assert the negative below
    try:
        import pypylon  # noqa: F401

        pytest.skip("pypylon is installed; skipping the no-SDK path")
    except ImportError:
        pass
    with pytest.raises(CameraConnectionError) as exc:
        BaslerCamera().open()
    assert "pypylon" in str(exc.value)
