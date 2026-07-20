"""Kinematics + workspace tests.

These encode the verified design (Claude.md §3): the whole work zone — the six
holes, the cap pick point, and the +/-2 in cross-conveyor tolerance — must clear
every singularity/reach check. If a geometry change breaks these, the change is
almost certainly wrong (see Claude.md §3: "do not silently revert").
"""

import math
from pathlib import Path

import pytest

from bung_cover_robot.robot import (
    FiveBarConfig,
    FiveBarKinematics,
    KinematicsError,
    SingularityLimits,
    WorkspaceValidator,
)

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "robot_config.yaml"

# Work zone in the robot frame (Claude.md §4): the 13 in-wide part footprint
# centered on X = 0 (X spans -160..+160), nominal Y_robot = 240; the +/-2 in
# (50 mm) cross-conveyor tolerance moves Y over [190, 290].
Y_NOM = 240.0
TOL = 50.0  # ~2 inches
WORK_X = [-160.0, -105.0, -55.0, 0.0, 55.0, 105.0, 160.0]
WORK_Y = [Y_NOM - TOL, Y_NOM, Y_NOM + TOL]
WORK_ZONE = [(x, y) for y in WORK_Y for x in WORK_X]


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
def test_default_config_is_the_verified_design_point():
    cfg = FiveBarConfig()
    assert cfg.l1_mm == 200.0
    assert cfg.l2_mm == 230.0
    assert cfg.base_spacing_mm == 80.0
    assert cfg.left_elbow == "up"
    assert cfg.right_elbow == "down"
    assert cfg.joint_min_deg == -20.0
    assert cfg.joint_max_deg == 200.0
    assert cfg.max_reach_mm == 430.0
    assert cfg.left_base == (-40.0, 0.0)
    assert cfg.right_base == (40.0, 0.0)


def test_pulses_per_degree():
    # A6 17-bit encoder: 131072 counts/rev * 3:1 / 360 deg = 1092.2667
    assert FiveBarConfig().pulses_per_degree == pytest.approx(1092.2667, abs=1e-3)


def test_from_yaml_matches_defaults():
    cfg = FiveBarConfig.from_yaml(CONFIG_PATH)
    default = FiveBarConfig()
    assert cfg == default


def test_invalid_branch_rejected():
    with pytest.raises(ValueError):
        FiveBarConfig(left_elbow="sideways")


# --------------------------------------------------------------------------- #
# Forward / inverse round trip
# --------------------------------------------------------------------------- #
def test_forward_inverse_round_trip_over_work_zone():
    kin = FiveBarKinematics()
    for x, y in WORK_ZONE:
        jt = kin.inverse(x, y)
        fx, fy = kin.forward(jt.left_deg, jt.right_deg)
        assert fx == pytest.approx(x, abs=1e-6)
        assert fy == pytest.approx(y, abs=1e-6)


def test_inverse_populates_pulses_and_elbows():
    kin = FiveBarKinematics()
    jt = kin.inverse(0.0, Y_NOM)
    assert jt.left_pulses == round(jt.left_deg * kin.config.pulses_per_degree)
    assert jt.right_pulses == round(jt.right_deg * kin.config.pulses_per_degree)
    # Distal links are exactly L2 from the TCP.
    for elbow in (jt.left_elbow, jt.right_elbow):
        assert math.hypot(elbow[0] - jt.tcp[0], elbow[1] - jt.tcp[1]) == pytest.approx(
            kin.config.l2_mm, abs=1e-6
        )
    # Proximal links are exactly L1 from their base.
    assert _dist(jt.left_elbow, kin.config.left_base) == pytest.approx(
        kin.config.l1_mm, abs=1e-6
    )
    assert _dist(jt.right_elbow, kin.config.right_base) == pytest.approx(
        kin.config.l1_mm, abs=1e-6
    )


def test_assembly_branch_is_left_up_right_down():
    # "up" places the left elbow on the CCW (outward, -X) side; "down" places
    # the right elbow on the outward (+X) side. At the centered TCP the elbows
    # splay symmetrically outward.
    kin = FiveBarKinematics()
    jt = kin.inverse(0.0, Y_NOM)
    assert jt.left_elbow[0] < kin.config.left_base[0]  # left elbow further -X
    assert jt.right_elbow[0] > kin.config.right_base[0]  # right elbow further +X


# --------------------------------------------------------------------------- #
# Reachability envelope
# --------------------------------------------------------------------------- #
def test_is_reachable_true_in_zone_false_far_away():
    kin = FiveBarKinematics()
    assert kin.is_reachable(0.0, Y_NOM)
    assert not kin.is_reachable(0.0, 900.0)  # beyond 430 mm arms


def test_inverse_raises_outside_envelope():
    kin = FiveBarKinematics()
    with pytest.raises(KinematicsError):
        kin.inverse(0.0, 900.0)


# --------------------------------------------------------------------------- #
# Workspace guard — the design verification
# --------------------------------------------------------------------------- #
def test_entire_work_zone_passes_the_guard():
    validator = WorkspaceValidator()
    for x, y in WORK_ZONE:
        res = validator.validate(x, y)
        assert res.ok, f"({x}, {y}) rejected: {res.reason}"


def test_work_zone_meets_verified_thresholds():
    # Claude.md §3: worst case ~45 deg parallel margin, ~52 deg serial margin,
    # <= 82% reach. Assert the guaranteed thresholds hold everywhere in-zone.
    validator = WorkspaceValidator()
    limits = SingularityLimits()
    worst_parallel = 999.0
    worst_serial = 999.0
    worst_reach = 0.0
    for x, y in WORK_ZONE:
        res = validator.validate(x, y)
        m = res.metrics
        worst_parallel = min(worst_parallel, m["parallel_margin_deg"])
        worst_serial = min(worst_serial, m["serial_margin_deg"])
        worst_reach = max(worst_reach, m["reach_fraction"])
    assert worst_parallel >= limits.parallel_min_deg
    assert worst_serial >= limits.serial_min_deg
    assert worst_reach <= limits.reach_fraction_max
    # Sanity against the design-review figures.
    assert worst_parallel >= 40.0
    assert worst_reach <= 0.85


def test_cap_pick_point_is_valid():
    # Cap pick point: X_world = -50, Y_world = 0 -> robot frame (-175, 250).
    validator = WorkspaceValidator()
    assert validator.is_safe(-175.0, Y_NOM)


def test_guard_rejects_unreachable_target():
    validator = WorkspaceValidator()
    res = validator.validate(0.0, 900.0)
    assert not res.ok
    assert "unreachable" in res.reason


def test_guard_rejects_overextended_target():
    # Reachable geometrically but past the 85% stiffness cap.
    validator = WorkspaceValidator()
    res = validator.validate(0.0, 400.0)
    assert not res.ok
    assert "reach fraction" in res.reason


def test_validation_result_is_truthy():
    validator = WorkspaceValidator()
    assert bool(validator.validate(0.0, Y_NOM)) is True


def test_scan_returns_grid():
    validator = WorkspaceValidator()
    records = validator.scan(-50.0, 50.0, 230.0, 330.0, step=50.0)
    assert len(records) == 3 * 3  # x: -50,0,50  y: 230,280,330
    assert all("ok" in r for r in records)
    # Grid must stay within the requested bounds (no overshoot).
    assert max(r["x"] for r in records) == 50.0
    assert max(r["y"] for r in records) == 330.0


def _dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def test_validate_rejects_wrong_assembly_mirror_target():
    """inverse() returns valid-looking angles for a sub-shoulder-line target,
    but the mechanism assembles at the +Y mirror ~350 mm away. validate() must
    reject it via the forward-consistency guard (it is documented as the sole
    go/no-go before motion)."""
    from bung_cover_robot.robot.workspace import WorkspaceValidator
    from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics

    val = WorkspaceValidator(FiveBarKinematics())
    res = val.validate(0.0, -240.0)
    assert not res.ok and "wrong-assembly" in res.reason
    # a genuine +Y work-zone target still passes (guard doesn't over-reject)
    assert val.validate(0.0, 250.0).ok
    assert val.validate(60.0, 205.0).ok
