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
    contour: Optional[np.ndarray] = None   # detected outline (px), for overlays

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


def find_hough_circles(
    frame: np.ndarray,
    min_diameter: float,
    max_diameter: float,
    blur: int = 5,
    dp: float = 1.2,
    param1: float = 120.0,
    param2: float = 30.0,
    min_dist: Optional[float] = None,
) -> List[Circle]:
    """Circles by the Hough gradient transform — votes accumulate from a circular
    edge and ignore linear clutter (wood grain), so it's a strong fit for a clean
    round cover. ``param2`` is the accumulator threshold: lower finds more (and
    more false) circles. Best on a near-circular object; for a very flat-sided (D)
    shape prefer ``find_round_objects``."""
    import cv2

    gray = _gray_blur(frame, blur)
    md = min_dist if min_dist else max(min_diameter, 20.0)
    res = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp, md, param1=param1, param2=param2,
        minRadius=max(0, int(round(min_diameter / 2.0))),
        maxRadius=int(round(max_diameter / 2.0)))
    out: List[Circle] = []
    if res is not None:
        # res is strongest-first; drop concentric/overlapping duplicates (a rim +
        # inner ring on one cover) so a single object yields a single circle.
        for x, y, r in res[0]:
            if any(math.hypot(x - o.cx, y - o.cy) < 0.5 * o.radius for o in out):
                continue
            out.append(Circle(float(x), float(y), float(r),
                              float(math.pi * r * r), 1.0))
    return out


def _round_candidates(
    frame: np.ndarray,
    min_diameter: float,
    max_diameter: float,
    blur: int,
    min_circularity: float,
    min_solidity: float,
    canny_lo_frac: float,
    canny_hi_frac: float,
):
    """Yield ``(Circle, reason)`` for every SIZE-passing round candidate.

    Two complementary outline sources, merged: Canny edges (polarity-independent;
    catches ring/bright-centered shapes) and Otsu region segmentation in BOTH
    polarities (relative, so it catches a low-contrast solid an edge threshold
    would miss). ``reason`` is "ok", or why a right-sized blob was rejected
    (roundness/solidity) — the diagnostic view surfaces it.
    """
    import cv2

    gray = _gray_blur(frame, blur)
    height, width = gray.shape[:2]
    ksz = max(3, (int(min_diameter / 4) | 1))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksz, ksz))
    med = float(np.median(gray))
    lo = int(max(0.0, canny_lo_frac * med))
    hi = int(min(255.0, max(lo + 1.0, canny_hi_frac * med)))
    masks = [cv2.morphologyEx(cv2.Canny(gray, lo, hi), cv2.MORPH_CLOSE, kernel)]
    _, otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    masks.append(cv2.morphologyEx(otsu, cv2.MORPH_CLOSE, kernel))
    masks.append(cv2.morphologyEx(cv2.bitwise_not(otsu), cv2.MORPH_CLOSE, kernel))

    for mask in masks:
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area <= 0.0:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if (x <= 1 or y <= 1 or x + w >= width - 1 or y + h >= height - 1) and (
                    w >= 0.9 * width or h >= 0.9 * height):
                continue                          # crop-spanning background blob
            (_, _), r = cv2.minEnclosingCircle(cnt)
            if not (min_diameter <= 2.0 * r <= max_diameter):
                continue
            peri = cv2.arcLength(cnt, True)
            if peri <= 0.0:
                continue
            circ = 4.0 * math.pi * area / (peri * peri)
            hull_area = cv2.contourArea(cv2.convexHull(cnt))
            solidity = area / hull_area if hull_area > 0.0 else 0.0
            m = cv2.moments(cnt)
            if m["m00"] == 0.0:
                continue
            cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
            if circ < min_circularity:
                reason = f"round {circ:.2f}<{min_circularity:.2f}"
            elif solidity < min_solidity:
                reason = f"solid {solidity:.2f}<{min_solidity:.2f}"
            else:
                reason = "ok"
            yield Circle(float(cx), float(cy), float(r), float(area), float(circ), cnt), reason


def _near(c: Circle, others: Sequence[Circle], tol: float) -> bool:
    return any(math.hypot(c.cx - o.cx, c.cy - o.cy) < tol for o in others)


def find_round_objects(
    frame: np.ndarray,
    min_diameter: float,
    max_diameter: float,
    blur: int = 5,
    min_circularity: float = 0.6,
    min_solidity: float = 0.9,
    canny_lo_frac: float = 0.33,
    canny_hi_frac: float = 0.66,
) -> List[Circle]:
    """Round-ish objects by their outline — color- and (partly) shape-agnostic.

    Solidity (area / convex-hull area) tolerates a D-shaped (flat-sided) cover —
    still convex — while rejecting wood grain; the centre is the region centroid
    (the right vacuum-pickup point for a D). See ``_round_candidates``.
    """
    out: List[Circle] = []
    for c, reason in _round_candidates(
            frame, min_diameter, max_diameter, blur, min_circularity,
            min_solidity, canny_lo_frac, canny_hi_frac):
        if reason == "ok" and not _near(c, out, min_diameter * 0.5):
            out.append(c)
    return out


def analyze_round_objects(
    frame: np.ndarray,
    min_diameter: float,
    max_diameter: float,
    blur: int = 5,
    min_circularity: float = 0.6,
    min_solidity: float = 0.9,
    canny_lo_frac: float = 0.33,
    canny_hi_frac: float = 0.66,
) -> List[Tuple[Circle, str]]:
    """Diagnostic: every size-passing candidate with its keep/reject reason, so
    the operator can see *why* a right-sized blob was or wasn't taken."""
    cands = list(_round_candidates(
        frame, min_diameter, max_diameter, blur, min_circularity,
        min_solidity, canny_lo_frac, canny_hi_frac))
    tol = min_diameter * 0.5
    ok: List[Circle] = []
    for c, reason in cands:
        if reason == "ok" and not _near(c, ok, tol):
            ok.append(c)
    results: List[Tuple[Circle, str]] = [(c, "ok") for c in ok]
    shown: List[Circle] = list(ok)
    for c, reason in cands:
        if reason == "ok" or _near(c, shown, tol):
            continue
        shown.append(c)
        results.append((c, reason))
    return results


def _ensure_bgr(frame: np.ndarray) -> np.ndarray:
    """A 3-channel BGR copy — so colored overlays render on a mono/grayscale
    camera frame (drawing green on a 1-channel image would paint black)."""
    import cv2

    if frame.ndim == 2 or frame.shape[2] == 1:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame.copy()


def offset_circles(circles: Sequence[Circle], ox: int, oy: int) -> List[Circle]:
    if ox == 0 and oy == 0:
        return list(circles)
    out: List[Circle] = []
    for c in circles:
        cnt = None if c.contour is None else c.contour + [ox, oy]
        out.append(Circle(c.cx + ox, c.cy + oy, c.radius, c.area, c.circularity, cnt))
    return out


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

    out = _ensure_bgr(frame)
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
            color = (0, 255, 0) if ok else (60, 60, 235)   # bright green / red
            ctr = (int(circle.cx), int(circle.cy))
            if circle.contour is not None:                       # true detected outline
                cv2.drawContours(out, [circle.contour.astype(int)], -1, color, 4)
            else:
                cv2.circle(out, ctr, int(circle.radius), color, 4)
            cv2.drawMarker(out, ctr, color, cv2.MARKER_CROSS, 20, 2)  # pickup point
            reason = getattr(cd, "reason", "")
            if not ok and reason:                                # why it was rejected
                cv2.putText(out, reason, (ctr[0] - int(circle.radius), ctr[1] - int(circle.radius) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    return out


def annotate_candidates(
    frame: np.ndarray, candidates: Sequence[Tuple[Circle, str]]
) -> np.ndarray:
    """Diagnostic overlay: every size-passing candidate, green (kept) or orange
    (rejected), labelled with its diameter and the reason — so the operator can
    see which slider to move."""
    import cv2

    out = _ensure_bgr(frame)
    for circle, reason in candidates:
        ok = reason == "ok"
        color = (0, 255, 0) if ok else (0, 165, 255)     # BGR: green / orange
        ctr = (int(circle.cx), int(circle.cy))
        if circle.contour is not None:
            cv2.drawContours(out, [circle.contour.astype(int)], -1, color, 3)
        else:
            cv2.circle(out, ctr, int(circle.radius), color, 3)
        cv2.drawMarker(out, ctr, color, cv2.MARKER_CROSS, 18, 2)
        cv2.putText(out, f"{int(circle.diameter)}px {reason}",
                    (ctr[0] + int(circle.radius) + 4, ctr[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    return out
