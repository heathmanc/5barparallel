"""Shared OpenCV detection primitives (Claude.md §12).

Round holes and round covers are found the same way: segment (dark or bright),
find contours, and keep the ones that are circular and the right size. The
hole/cover detectors add their own filtering and sanity checks on top.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

ROI = Tuple[int, int, int, int]  # x, y, w, h


@dataclass(frozen=True)
class Circle:
    cx: float
    cy: float
    radius: float
    area: float
    circularity: float

    @property
    def diameter(self) -> float:
        return 2.0 * self.radius

    @property
    def center(self) -> Tuple[float, float]:
        return (self.cx, self.cy)


def crop_roi(frame: np.ndarray, roi: Optional[ROI]) -> Tuple[np.ndarray, int, int]:
    """Return (subimage, offset_x, offset_y). None => whole frame."""
    if roi is None:
        return frame, 0, 0
    x, y, w, h = roi
    x = max(0, x)
    y = max(0, y)
    return frame[y : y + h, x : x + w], x, y


def _gray_blur(frame: np.ndarray, blur: int) -> np.ndarray:
    import cv2

    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if blur and blur >= 3:
        k = blur if blur % 2 == 1 else blur + 1
        gray = cv2.GaussianBlur(gray, (k, k), 0)
    return gray


def dark_mask(frame: np.ndarray, blur: int = 5, thresh: Optional[int] = None) -> np.ndarray:
    """Binary mask of the dark regions (for holes). Otsu unless ``thresh`` set."""
    import cv2

    gray = _gray_blur(frame, blur)
    if thresh is None:
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)
    return _open(mask)


def bright_mask(frame: np.ndarray, blur: int = 5, thresh: Optional[int] = None) -> np.ndarray:
    """Binary mask of the bright regions (for covers)."""
    import cv2

    gray = _gray_blur(frame, blur)
    if thresh is None:
        _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)
    return _open(mask)


def _open(mask: np.ndarray) -> np.ndarray:
    import cv2

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def find_circles(
    mask: np.ndarray,
    min_diameter: float,
    max_diameter: float,
    min_circularity: float,
) -> List[Circle]:
    """Circular contours in a binary mask that pass size + circularity filters."""
    import cv2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = math.pi * (min_diameter / 2.0) ** 2
    max_area = math.pi * (max_diameter / 2.0) ** 2
    out: List[Circle] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area * 0.5 or area > max_area * 1.5:
            continue
        perim = cv2.arcLength(c, True)
        if perim <= 0:
            continue
        circularity = 4.0 * math.pi * area / (perim * perim)
        if circularity < min_circularity:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        if not (min_diameter <= 2 * r <= max_diameter):
            continue
        out.append(Circle(float(x), float(y), float(r), float(area), float(circularity)))
    return out


def find_battery_roi(
    frame: np.ndarray, inset: int = 6, min_area_frac: float = 0.04
) -> Optional[ROI]:
    """Bounding box of the largest bright region (the battery top), inset a little.

    Lets hole detection confine itself to the battery without calibration, so
    loose covers on the tray can't be mistaken for holes. None if nothing large
    enough is found."""
    import cv2

    gray = _gray_blur(frame, 5)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area_frac * frame.shape[0] * frame.shape[1]:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return (x + inset, y + inset, max(1, w - 2 * inset), max(1, h - 2 * inset))


def find_blobs(
    frame: np.ndarray,
    dark: bool,
    min_diameter: float,
    max_diameter: float,
    min_circularity: float,
    blur: int = 5,
) -> List[Circle]:
    """Circular blobs via SimpleBlobDetector — robust to background nesting (e.g.
    holes *inside* the battery blob) and lighting. ``dark`` selects dark vs bright
    blobs."""
    import cv2

    gray = _gray_blur(frame, blur)
    p = cv2.SimpleBlobDetector_Params()
    p.filterByColor = True
    p.blobColor = 0 if dark else 255
    p.filterByArea = True
    p.minArea = math.pi * (min_diameter / 2.0) ** 2 * 0.6
    p.maxArea = math.pi * (max_diameter / 2.0) ** 2 * 1.6
    p.filterByCircularity = True
    p.minCircularity = min_circularity
    p.filterByConvexity = True
    p.minConvexity = 0.8
    p.filterByInertia = True
    p.minInertiaRatio = 0.4
    p.minThreshold = 10
    p.maxThreshold = 220
    p.thresholdStep = 10
    detector = cv2.SimpleBlobDetector_create(p)
    out: List[Circle] = []
    for k in detector.detect(gray):
        r = k.size / 2.0
        if min_diameter <= 2 * r <= max_diameter:
            out.append(Circle(float(k.pt[0]), float(k.pt[1]), float(r),
                              float(math.pi * r * r), 1.0))
    return out


def find_round_objects(
    frame: np.ndarray,
    min_diameter: float,
    max_diameter: float,
    blur: int = 5,
    min_circularity: float = 0.6,
    min_solidity: float = 0.9,
) -> List[Circle]:
    """Round-ish objects by their outline — color- and (partly) shape-agnostic.

    Segments the region (Otsu, tried BOTH polarities so it finds an object that
    is darker *or* brighter than its background), closes internal features (a
    bright center, small holes/vents, a flat spot), then keeps blobs that are the
    right size and **convex** (``solidity`` = area / convex-hull area). Solidity
    tolerates a D-shaped (flat-sided) cover — which is still convex — while
    rejecting wood grain and clutter, and the reported centre is the region
    centroid (the right vacuum-pickup point for a D), not a circle centre.

    ``min_circularity`` is relaxed (0.6) so a flat-sided disk still passes.
    """
    import cv2

    gray = _gray_blur(frame, blur)
    # Edge gradient is polarity-independent: a boundary shows up whether the object
    # is darker OR brighter than its background. Median-seeded Canny + a close to
    # bridge small gaps (and the flat side of a D) into a shut outline.
    med = float(np.median(gray))
    lo = int(max(0.0, 0.50 * med))
    hi = int(min(255.0, max(lo + 1.0, 1.30 * med)))
    edges = cv2.Canny(gray, lo, hi)
    ksz = max(3, (int(min_diameter / 4) | 1))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksz, ksz))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    height, width = gray.shape[:2]
    out: List[Circle] = []
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area <= 0.0:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        if x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1:
            continue                          # touches the border => background, not an object
        (_, _), r = cv2.minEnclosingCircle(cnt)
        if not (min_diameter <= 2.0 * r <= max_diameter):
            continue
        peri = cv2.arcLength(cnt, True)
        if peri <= 0.0:
            continue
        circ = 4.0 * math.pi * area / (peri * peri)
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        solidity = area / hull_area if hull_area > 0.0 else 0.0
        if circ < min_circularity or solidity < min_solidity:
            continue
        m = cv2.moments(cnt)
        if m["m00"] == 0.0:
            continue
        cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
        if any(math.hypot(cx - o.cx, cy - o.cy) < min_diameter * 0.5 for o in out):
            continue
        out.append(Circle(float(cx), float(cy), float(r), float(area), float(circ)))
    return out


def offset_circles(circles: Sequence[Circle], ox: int, oy: int) -> List[Circle]:
    if ox == 0 and oy == 0:
        return list(circles)
    return [Circle(c.cx + ox, c.cy + oy, c.radius, c.area, c.circularity) for c in circles]


def annotate(
    frame: np.ndarray,
    holes: Optional[Sequence[Circle]] = None,
    covers: Optional[Sequence["object"]] = None,
) -> np.ndarray:
    """Draw hole/cover overlays on a copy of the frame (BGR).

    ``covers`` items are CoverDetection (have .circle and .accepted); accepted =
    green, rejected = red.
    """
    import cv2

    out = frame.copy()
    if holes:
        for i, h in enumerate(holes):
            c = (int(h.cx), int(h.cy))
            cv2.circle(out, c, int(h.radius), (0, 220, 0), 2)
            cv2.drawMarker(out, c, (0, 220, 0), cv2.MARKER_CROSS, 8, 1)
            cv2.putText(out, str(i), (c[0] + int(h.radius) + 2, c[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)
    if covers:
        for cd in covers:
            circle = cd.circle
            ok = getattr(cd, "accepted", True)
            color = (0, 200, 0) if ok else (60, 60, 235)
            cv2.circle(out, (int(circle.cx), int(circle.cy)), int(circle.radius), color, 2)
    return out
