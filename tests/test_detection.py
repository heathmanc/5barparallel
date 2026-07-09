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


def test_cover_search_confined_to_pick_region():
    # method "auto" + a pick region => color-agnostic shape search, cropped to it.
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.circle(img, (100, 150), 18, (180, 180, 180), -1)   # inside the region
    cv2.circle(img, (240, 150), 18, (180, 180, 180), -1)   # outside the region
    cfg = CoverDetectorConfig(pick_roi=(60, 100, 100, 100))  # x 60..160, y 100..200
    res = CoverDetector(cfg).detect(img)
    xs = sorted(round(c.circle.cx) for c in res.covers)
    assert xs and all(abs(x - 100) <= 5 for x in xs)   # only the in-region cover
    assert res.accepted and res.accepted[0].accepted


def test_cover_outside_pick_roi_rejected_blob():
    # legacy blob path still detects everywhere and rejects outside-region covers.
    img = np.full((300, 300, 3), 20, np.uint8)
    cv2.circle(img, (100, 150), 18, (180, 180, 180), -1)
    cv2.circle(img, (240, 150), 18, (180, 180, 180), -1)
    cfg = CoverDetectorConfig(pick_roi=(60, 100, 100, 100), method="blob")
    res = CoverDetector(cfg).detect(img)
    by_x = {round(c.circle.cx): c for c in res.covers}
    assert by_x[100].accepted
    assert not by_x[240].accepted
    assert by_x[240].reason == "outside pick region"


def test_shape_finder_is_color_and_shape_agnostic():
    # a D-shaped DARK cover and a D-shaped BRIGHT cover (the two color extremes) —
    # both found by their outline regardless of fill, flat side included (solidity
    # tolerates a D). A dark-fill blob detector would miss the bright one, and a
    # bright-fill one would miss the dark one.
    from bung_cover_robot.vision.detection import find_round_objects

    img = np.full((240, 420, 3), 128, np.uint8)
    specs = [(110, 120, (20, 20, 20)), (300, 120, (235, 235, 235))]
    for cx, cy, col in specs:
        cv2.circle(img, (cx, cy), 42, col, -1)
        cv2.rectangle(img, (cx + 28, cy - 42), (cx + 64, cy + 42), (128, 128, 128), -1)  # flat side (D)
    found = find_round_objects(img, 55, 110)
    xs = sorted(round(c.cx) for c in found)
    for cx, _, _ in specs:
        assert any(abs(fx - cx) <= 14 for fx in xs), f"missed object near {cx}: {xs}"


def test_annotate_draws_color_on_a_mono_frame():
    # a single-channel (mono camera) frame: overlays must render in real color,
    # not black (drawing green on a 1-channel image would paint intensity 0).
    from bung_cover_robot.vision.detection import Circle, annotate

    class _Cover:
        def __init__(self, c):
            self.circle = c
            self.accepted = True

    gray = np.full((200, 200), 80, np.uint8)          # H×W, one channel
    out = annotate(gray, covers=[_Cover(Circle(100, 100, 30, 2827.0, 1.0))])
    assert out.ndim == 3 and out.shape[2] == 3
    green = (out[:, :, 0] == 0) & (out[:, :, 1] == 255) & (out[:, :, 2] == 0)
    assert green.any()                                 # real green pixels were drawn


def test_reachable_zone_outline_renders():
    from bung_cover_robot.gui.imaging import demo_frame, demo_transform
    from bung_cover_robot.robot import WorkspaceValidator
    from bung_cover_robot.vision.detection import (
        draw_reachable_zone,
        reachable_zone_contours,
    )

    cons = reachable_zone_contours(WorkspaceValidator().is_safe, -300, 300, 40, 430, 6.0)
    assert cons and sum(len(c) for c in cons) > 20     # a real boundary polygon

    cal = demo_transform()

    def r2p(x, y):
        p = cal.robot_to_pixel_many([[x, y]])[0]
        return (float(p[0]), float(p[1]))

    img = demo_frame(760, 520)
    out = draw_reachable_zone(img, cons, r2p)
    assert out.shape == (520, 760, 3)
    assert np.any(out != img, axis=2).sum() > 500      # bold outline drawn


def test_draw_robot_grid_overlays_a_grid():
    from bung_cover_robot.gui.imaging import demo_frame, demo_transform
    from bung_cover_robot.vision.detection import draw_robot_grid

    cal = demo_transform()

    def r2p(x, y):
        p = cal.robot_to_pixel_many([[x, y]])[0]
        return (float(p[0]), float(p[1]))

    img = demo_frame(760, 520)
    out = draw_robot_grid(img, cal.pixel_to_robot, r2p, 25.0)
    assert out.shape == (520, 760, 3)
    changed = np.any(out != img, axis=2)
    assert changed.sum() > 1000          # a full 25 mm grid touches many pixels


def test_hough_finds_a_clean_disk():
    from bung_cover_robot.vision.detection import find_hough_circles

    img = np.full((260, 260, 3), 60, np.uint8)
    cv2.circle(img, (130, 130), 60, (205, 205, 205), -1)
    found = find_hough_circles(img, 80, 170, param2=25)
    assert found and any(abs(c.cx - 130) <= 8 and abs(c.cy - 130) <= 8 for c in found)


def test_hough_finds_low_contrast_large_cover():
    # big low-contrast dark cover on grainy wood — Hough votes from the circular
    # edge and ignores the linear grain (the real-scene case).
    from bung_cover_robot.vision.detection import find_hough_circles

    h, w = 700, 900
    img = np.full((h, w, 3), 160, np.uint8)
    for y in range(0, h, 3):
        s = 150 + int(14 * np.sin(y * 0.2))
        cv2.line(img, (0, y), (w, y), (s, s, s), 1)          # wood grain
    cv2.circle(img, (450, 360), 150, (95, 95, 95), -1)       # r150 (~300 px) cover
    img = cv2.GaussianBlur(img, (5, 5), 0)                    # soft edges
    found = find_hough_circles(img, 270, 340, blur=5, param1=51, param2=18)
    assert found and any(abs(c.cx - 450) <= 12 and abs(c.cy - 360) <= 12 for c in found)


def test_cover_detector_hough_method():
    img = np.full((260, 260, 3), 60, np.uint8)
    cv2.circle(img, (130, 130), 58, (205, 205, 205), -1)
    cfg = CoverDetectorConfig(
        method="hough", min_diameter_px=80, max_diameter_px=170, hough_param2=25)
    res = CoverDetector(cfg).detect(img)
    assert res.count >= 1


def test_shape_finds_low_contrast_solid_cover():
    # a dark-grey D cover on lighter wood with soft edges — the real failure case
    # (edge-only detection missed it; the Otsu-region pass catches it).
    from bung_cover_robot.vision.detection import find_round_objects

    img = np.full((260, 340, 3), 165, np.uint8)                      # light background
    cv2.circle(img, (170, 130), 70, (95, 95, 95), -1)               # dark-grey cover
    cv2.rectangle(img, (216, 60), (260, 200), (165, 165, 165), -1)  # flat side (D)
    img = cv2.GaussianBlur(img, (7, 7), 0)                           # soft edges
    found = find_round_objects(img, 60, 200)
    assert found and any(
        abs(c.cx - 170) <= 20 and abs(c.cy - 130) <= 20 for c in found)
    assert found[0].contour is not None    # outline is available for the overlay


def test_hole_detector_finds_bright_centered_holes():
    # the real failure case: dark RING + BRIGHT center. Blob(dark) chokes; shape wins.
    img = np.full((200, 460, 3), 70, np.uint8)
    for x in (90, 200, 310):                                    # >= 2, collinear
        cv2.circle(img, (x, 100), 26, (15, 15, 15), 5)          # dark ring
        cv2.circle(img, (x, 100), 18, (240, 240, 240), -1)      # bright center
    cfg = HoleDetectorConfig(
        expected_count=3, roi=(50, 55, 320, 90),
        min_diameter_px=30, max_diameter_px=70, collinear_tol_px=8.0,
    )
    res = HoleDetector(cfg).detect(img)
    assert res.count == 3 and res.ok


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
