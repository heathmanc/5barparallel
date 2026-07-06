"""Pixel -> robot-frame calibration (Claude.md §13).

    pixel point -> (undistort) -> homography (per Z plane) -> ROBOT-frame XY

A `HomographyTransform` maps undistorted image points to robot millimetres for one
plane. `CalibrationManager` holds one transform *per recipe* (battery type) —
holes and covers share a plane that a changeover shifts — persisted as
``calibration/<recipe_key>.npy`` (git-ignored).

Lens undistortion (Brown-Conrady, via the camera intrinsics) is applied *before*
the homography — on a 2592x1944 sensor the corners can be several pixels off,
which can exceed the placement tolerance.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]


class CalibrationError(Exception):
    """Missing/invalid calibration data."""


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics + Brown-Conrady distortion (k1,k2,p1,p2,k3)."""

    fx: float
    fy: float
    cx: float
    cy: float
    dist: Tuple[float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0)

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]]
        )

    @property
    def has_distortion(self) -> bool:
        return any(abs(d) > 1e-12 for d in self.dist)

    def undistort_points(self, pts: np.ndarray) -> np.ndarray:
        """(N,2) distorted pixels -> (N,2) undistorted pixels (in pixel coords)."""
        import cv2

        p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        out = cv2.undistortPoints(p, self.K, np.asarray(self.dist, float), P=self.K)
        return out.reshape(-1, 2)

    @classmethod
    def from_dict(cls, data: dict) -> Optional["CameraIntrinsics"]:
        if not data or data.get("fx") is None:
            return None
        dist = tuple(data.get("dist", (0.0,) * 5)) + (0.0,) * 5
        return cls(
            fx=float(data["fx"]), fy=float(data["fy"]),
            cx=float(data["cx"]), cy=float(data["cy"]),
            dist=tuple(float(d) for d in dist[:5]),
        )

    @classmethod
    def from_yaml(cls, path: str | Path) -> Optional["CameraIntrinsics"]:
        import yaml

        data = yaml.safe_load(Path(path).read_text()) or {}
        return cls.from_dict(data.get("intrinsics", {}))


@dataclass
class HomographyTransform:
    """Maps image pixels to robot-frame millimetres for one Z plane."""

    H: np.ndarray  # 3x3
    intrinsics: Optional[CameraIntrinsics] = None
    name: str = ""
    residual_mm: float = float("nan")

    def pixel_to_robot(self, px: float, py: float) -> Point:
        out = self.pixel_to_robot_many([[px, py]])[0]
        return (float(out[0]), float(out[1]))

    def pixel_to_robot_many(self, pts: Sequence[Sequence[float]]) -> np.ndarray:
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        if self.intrinsics is not None and self.intrinsics.has_distortion:
            pts = self.intrinsics.undistort_points(pts)
        homo = np.hstack([pts, np.ones((pts.shape[0], 1))])   # (N,3)
        out = (self.H @ homo.T).T                              # (N,3)
        return out[:, :2] / out[:, 2:3]

    def robot_to_pixel_many(self, pts: Sequence[Sequence[float]]) -> np.ndarray:
        """Inverse map (ignores re-distortion) — handy for drawing overlays."""
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 2)
        Hinv = np.linalg.inv(self.H)
        homo = np.hstack([pts, np.ones((pts.shape[0], 1))])
        out = (Hinv @ homo.T).T
        return out[:, :2] / out[:, 2:3]

    # --- construction -------------------------------------------------------
    @classmethod
    def from_matrix(
        cls, H, intrinsics: Optional[CameraIntrinsics] = None, name: str = ""
    ) -> "HomographyTransform":
        return cls(np.asarray(H, dtype=np.float64).reshape(3, 3), intrinsics, name)

    @classmethod
    def from_correspondences(
        cls,
        pixel_pts: Sequence[Sequence[float]],
        robot_pts: Sequence[Sequence[float]],
        intrinsics: Optional[CameraIntrinsics] = None,
        name: str = "",
    ) -> "HomographyTransform":
        """Fit from >= 4 pixel<->robot correspondences (least squares)."""
        import cv2

        pixel = np.asarray(pixel_pts, dtype=np.float64).reshape(-1, 2)
        robot = np.asarray(robot_pts, dtype=np.float64).reshape(-1, 2)
        if len(pixel) < 4 or len(pixel) != len(robot):
            raise CalibrationError("need >= 4 matching pixel/robot points")
        src = (
            intrinsics.undistort_points(pixel)
            if (intrinsics is not None and intrinsics.has_distortion)
            else pixel
        )
        H, _ = cv2.findHomography(src.astype(np.float64), robot.astype(np.float64), 0)
        if H is None:
            raise CalibrationError("homography fit failed (degenerate points?)")
        t = cls(H.astype(np.float64), intrinsics, name)
        pred = t.pixel_to_robot_many(pixel)
        t.residual_mm = float(np.sqrt(np.mean(np.sum((pred - robot) ** 2, axis=1))))
        return t

    # --- persistence --------------------------------------------------------
    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.save(str(path), self.H)
        return path

    @classmethod
    def load(
        cls, path: str | Path, intrinsics: Optional[CameraIntrinsics] = None, name: str = ""
    ) -> "HomographyTransform":
        return cls(np.load(str(path)), intrinsics, name)


class CalibrationManager:
    """Owns one pixel->robot transform *per recipe* (battery type).

    Holes and covers share a plane that a changeover shifts, so each recipe has a
    single calibration keyed by its recipe key, at ``calibration/<key>.npy``.
    """

    def __init__(
        self,
        directory: str | Path = "calibration",
        intrinsics: Optional[CameraIntrinsics] = None,
    ) -> None:
        self.dir = Path(directory)
        self.intrinsics = intrinsics

    def path(self, recipe_key: str) -> Path:
        return self.dir / f"{recipe_key}.npy"

    def has(self, recipe_key: str) -> bool:
        return self.path(recipe_key).exists()

    def get(self, recipe_key: str) -> HomographyTransform:
        p = self.path(recipe_key)
        if not p.exists():
            raise CalibrationError(
                f"no calibration for recipe '{recipe_key}' at {p}; "
                "build one in the Calibration tab first"
            )
        return HomographyTransform.load(p, self.intrinsics, recipe_key)

    def save(self, recipe_key: str, transform: HomographyTransform) -> Path:
        return transform.save(self.path(recipe_key))

    def keys(self) -> List[str]:
        """Recipe keys that have a saved calibration on disk."""
        if not self.dir.exists():
            return []
        return sorted(p.stem for p in self.dir.glob("*.npy"))
