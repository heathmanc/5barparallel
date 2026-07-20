"""Pure helpers behind the Drives-tab tuning assistant — grading a characterize
move's following error against the drive's fault window. No Qt, no I/O."""

from __future__ import annotations


def fe_degrees(counts: int, pulses_per_degree: float) -> float:
    """Following error in joint degrees from raw drive counts."""
    if pulses_per_degree <= 0:
        return 0.0
    return abs(counts) / pulses_per_degree


def fe_margin_pct(peak_counts: int, window_counts: int) -> float:
    """Peak following error as a percentage of the drive's fault window
    (``0x6065``). 100 % sits right at the Er.47/0x8611 trip; lower is safer.
    Returns ``inf`` when no window is known so the caller shows '—'."""
    if window_counts <= 0:
        return float("inf")
    return 100.0 * abs(peak_counts) / window_counts


def grade(peak_counts: int, window_counts: int) -> str:
    """One-word verdict for a characterize run's headroom under the fault window."""
    pct = fe_margin_pct(peak_counts, window_counts)
    if pct == float("inf"):
        return "unknown"
    if pct < 40.0:
        return "good"
    if pct < 70.0:
        return "ok"
    if pct < 100.0:
        return "marginal"
    return "TRIPPING"
