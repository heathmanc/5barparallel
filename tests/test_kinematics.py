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

# Work zone in the robot frame (Claude.md §4): every nominal target sits at
# Y_robot = 250; X_robot spans -175 (pick) to +175 (far hole). The +/-2 in
# (50.8 mm) cross-conveyor tolerance moves Y over [199.2, 300.8].
Y_NOM = 250.0
TOL = 50.8  # 2 inches
WORK_X = [-175.0, -125.0, -75.0, 0.0, 75.0, 125.0, 175.0]
WORK_Y = [Y_NOM - TOL, Y_NOM, Y_NOM + TOL]
WORK_ZONE = [(x, y) for y in WORK_Y for x in WORK_X]


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
def test_default_config_is_the_verified_design_point():
    cfg = FiveBarConfig()
    assert cfg.l1_mm == 220.0
    assert cfg.l2_mm == 230.0
    assert cfg.base_spacing_mm == 101.6
    assert cfg.left_elbow == "up"
    assert cfg.right_elbow == "down"
    assert cfg.joint_min_deg == -20.0
    assert cfg.joint_max_deg == 200.0
    assert cfg.max_reach_mm == 450.0
    assert cfg.left_base == (-50.8, 0.0)
    assert cfg.right_base == (50.8, 0.0)


def test_pulses_per_degree():
    # 3200 pulses/rev * 3:1 / 360 deg = 26.6667
    assert FiveBarConfig().pulses_per_degree == pytest.approx(26.66667, abs=1e-4)


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
    assert not kin.is_reachable(0.0, 900.0)  # beyond 450 mm arms


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
    # Claude.md §3: worst case ~31 deg parallel margin, ~53 deg serial margin,
    # <= 84% reach. Assert the guaranteed thresholds hold everywhere in-zone.
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
    assert worst_parallel >= 30.0
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
    res = validator.validate(0.0, 430.0)
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
