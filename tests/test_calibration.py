"""Pixel->robot calibration: homography fit, undistortion, persistence, manager."""

import numpy as np
import pytest

from bung_cover_robot.vision.calibration import (
    CalibrationError,
    CalibrationManager,
    CameraIntrinsics,
    HomographyTransform,
)

PIX = [[100, 100], [600, 100], [600, 400], [100, 400]]
ROB = [[-150, 300], [150, 300], [150, 200], [-150, 200]]


# --------------------------------------------------------------------------- #
# HomographyTransform
# --------------------------------------------------------------------------- #
def test_from_correspondences_exact():
    t = HomographyTransform.from_correspondences(PIX, ROB, name="t")
    assert t.residual_mm == pytest.approx(0.0, abs=1e-6)
    for p, r in zip(PIX, ROB):
        assert t.pixel_to_robot(*p) == pytest.approx(tuple(r), abs=1e-4)
    # centre of the pixel rect maps to the centre of the robot rect
    assert t.pixel_to_robot(350, 250) == pytest.approx((0.0, 250.0), abs=1e-4)


def test_pixel_to_robot_many_shape():
    t = HomographyTransform.from_correspondences(PIX, ROB)
    out = t.pixel_to_robot_many(PIX)
    assert out.shape == (4, 2)


def test_needs_four_points():
    with pytest.raises(CalibrationError):
        HomographyTransform.from_correspondences(PIX[:3], ROB[:3])


def test_from_matrix_and_inverse():
    t = HomographyTransform.from_correspondences(PIX, ROB)
    t2 = HomographyTransform.from_matrix(t.H)
    assert t2.pixel_to_robot(350, 250) == pytest.approx((0.0, 250.0), abs=1e-4)
    # robot_to_pixel inverts pixel_to_robot
    px = t.robot_to_pixel_many([[0.0, 250.0]])[0]
    assert (float(px[0]), float(px[1])) == pytest.approx((350.0, 250.0), abs=1e-3)


def test_save_load_roundtrip(tmp_path):
    t = HomographyTransform.from_correspondences(PIX, ROB)
    path = t.save(tmp_path / "cover.npy")
    assert path.exists()
    loaded = HomographyTransform.load(path)
    assert loaded.pixel_to_robot(350, 250) == pytest.approx((0.0, 250.0), abs=1e-4)


# --------------------------------------------------------------------------- #
# CameraIntrinsics / undistortion
# --------------------------------------------------------------------------- #
def test_intrinsics_from_dict_none_when_unset():
    assert CameraIntrinsics.from_dict({"fx": None}) is None
    assert CameraIntrinsics.from_dict({}) is None


def test_intrinsics_distortion_flags():
    zero = CameraIntrinsics(1000, 1000, 320, 240)
    assert not zero.has_distortion
    # zero distortion is an identity undistort
    assert zero.undistort_points([[400, 300]])[0] == pytest.approx([400, 300], abs=1e-3)
    dist = CameraIntrinsics(1000, 1000, 320, 240, dist=(-0.2, 0.05, 0, 0, 0))
    assert dist.has_distortion
    moved = dist.undistort_points([[400, 300]])[0]
    assert not np.allclose(moved, [400, 300], atol=1e-3)


def test_homography_with_intrinsics_still_exact():
    intr = CameraIntrinsics(1000, 1000, 350, 250)  # zero distortion
    t = HomographyTransform.from_correspondences(PIX, ROB, intrinsics=intr)
    assert t.residual_mm == pytest.approx(0.0, abs=1e-6)


# --------------------------------------------------------------------------- #
# CalibrationManager
# --------------------------------------------------------------------------- #
def test_manager_cover_roundtrip(tmp_path):
    mgr = CalibrationManager(tmp_path)
    assert not mgr.has_cover_transform()
    with pytest.raises(CalibrationError):
        mgr.get_cover_transform()
    mgr.save_cover_transform(HomographyTransform.from_correspondences(PIX, ROB))
    assert mgr.has_cover_transform()
    assert mgr.get_cover_transform().pixel_to_robot(350, 250) == pytest.approx(
        (0.0, 250.0), abs=1e-4
    )


def test_manager_battery_per_recipe(tmp_path):
    mgr = CalibrationManager(tmp_path)
    assert not mgr.has_battery_transform("g31")
    with pytest.raises(CalibrationError):
        mgr.get_battery_transform("g31")
    mgr.save_battery_transform("g31", HomographyTransform.from_correspondences(PIX, ROB))
    assert mgr.has_battery_transform("g31")
    assert not mgr.has_battery_transform("other")
