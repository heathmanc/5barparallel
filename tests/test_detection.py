"""Vision detection: holes (collinear, correct count) and covers (quality
filters incl. reachability). Uses synthetic OpenCV scenes."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from bung_cover_robot.gui.imaging import demo_frame  # noqa: E402
from bung_cover_robot.robot import WorkspaceValidator  # noqa: E402
from bung_cover_robot.vision.detect_covers import (  # noqa: E402
    CoverDetector,
    CoverDetectorConfig,
)
from bung_cover_robot.vision.detect_holes import (  # noqa: E402
    HoleDetector,
    HoleDetectorConfig,
)
from bung_cover_robot.vision.detection import find_battery_roi  # noqa: E402


def _battery_with_holes(n=6, jitter=0.0):
    """A gray battery on dark bg with n dark holes inset from the edges. ``jitter``
    alternates the row up/down to make them non-collinear."""
    img = np.full((400, 700, 3), 25, np.uint8)
    cv2.rectangle(img, (120, 150), (580, 250), (82, 82, 82), -1)
    x0, x1 = 180, 520
    for i in range(n):
        cx = int(x0 + i * ((x1 - x0) / max(n - 1, 1)))
        cy = int(200 + (jitter if i % 2 else -jitter))
        cv2.circle(img, (cx, cy), 13, (15, 15, 15), -1)
    return img


# --------------------------------------------------------------------------- #
# Holes
# --------------------------------------------------------------------------- #
def test_detects_six_collinear_holes_on_demo():
    res = HoleDetector().detect(demo_frame(760, 520))
    assert res.count == 6
    assert res.ok
    assert res.max_residual_px < 2.0
    # returned sorted left-to-right
    xs = [h.cx for h in res.holes]
    assert xs == sorted(xs)


def test_hole_count_mismatch_reported():
    res = HoleDetector().detect(_battery_with_holes(n=5))
    assert res.count == 5
    assert not res.ok
    assert "expected 6" in res.reason


def test_non_collinear_holes_rejected():
    res = HoleDetector().detect(_battery_with_holes(n=6, jitter=15.0))
    assert res.count == 6
    assert not res.ok
    assert "collinear" in res.reason


def test_battery_roi_found():
    roi = find_battery_roi(demo_frame(760, 520))
    assert roi is not None
    x, y, w, h = roi
    assert w > 300 and 80 < h < 260  # roughly the battery bbox


# --------------------------------------------------------------------------- #
# Covers
# --------------------------------------------------------------------------- #
def test_detects_covers_on_demo():
    res = CoverDetector().detect(demo_frame(760, 520))
    assert res.count == 5
    assert len(res.accepted) == 5


def test_cover_near_edge_rejected():
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.circle(img, (150, 150), 18, (180, 180, 180), -1)   # central, ok
    cv2.circle(img, (24, 150), 18, (180, 180, 180), -1)    # within the edge margin
    res = CoverDetector().detect(img)
    reasons = {round(c.circle.cx): c.reason for c in res.covers}
    assert reasons[150] == "ok"
    assert any("edge" in r for cx, r in reasons.items() if cx < 40)


def test_cover_reachability_filter():
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.circle(img, (150, 150), 20, (180, 180, 180), -1)

    validator = WorkspaceValidator()
    reachable = CoverDetector().detect(
        img, to_robot=lambda px, py: (0.0, 250.0), validator=validator
    )
    assert reachable.covers[0].accepted
    assert reachable.covers[0].robot_xy == (0.0, 250.0)

    unreachable = CoverDetector().detect(
        img, to_robot=lambda px, py: (0.0, 900.0), validator=validator
    )
    assert not unreachable.covers[0].accepted
    assert "unreachable" in unreachable.covers[0].reason


def test_non_circular_blob_ignored():
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.rectangle(img, (60, 60), (240, 110), (180, 180, 180), -1)  # long bar, not round
    cv2.circle(img, (150, 200), 20, (180, 180, 180), -1)           # one real cover
    res = CoverDetector(CoverDetectorConfig(min_circularity=0.8)).detect(img)
    assert res.count == 1


def test_cover_physical_size_gate():
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.circle(img, (90, 150), 18, (180, 180, 180), -1)    # r=18px -> ~35 mm at 1mm/px
    cv2.circle(img, (210, 150), 9, (180, 180, 180), -1)    # r=9px  -> ~17 mm
    # Scale-1 calibration (1 px == 1 mm) centred in the reachable work zone.
    to_robot = lambda px, py: (float(px) - 150.0, float(py) - 150.0 + 250.0)
    validator = WorkspaceValidator()
    cfg = CoverDetectorConfig(expected_diameter_mm=36.0, diameter_tolerance=0.2)
    res = CoverDetector(cfg).detect(img, to_robot=to_robot, validator=validator)
    by_x = {round(c.circle.cx): c for c in res.covers}
    assert by_x[90].accepted                        # ~35 mm matches ~36 mm
    assert not by_x[210].accepted                   # ~17 mm is out of band
    assert "wrong size" in by_x[210].reason


def test_cover_size_gate_disabled_without_calibration():
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.circle(img, (150, 150), 9, (180, 180, 180), -1)
    # expected size set, but no to_robot -> size can't be measured -> not gated.
    cfg = CoverDetectorConfig(expected_diameter_mm=36.0)
    res = CoverDetector(cfg).detect(img)
    assert res.covers[0].accepted
