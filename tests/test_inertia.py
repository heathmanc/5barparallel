"""Load-inertia estimate for setting C00.06 without the (unsafe on a closed
5-bar) native inertia auto-tune. Covers the physics scaling and the guards."""

import math

import pytest

from bung_cover_robot.robot.inertia import (
    InertiaInputs,
    estimate_load_inertia,
)


def test_ratio_is_reflected_through_reduction_squared():
    # Same load, 3:1 vs 1:1 -> reflected inertia (and ratio) differ by N^2 = 9.
    base = InertiaInputs(reduction=1.0)
    geared = InertiaInputs(reduction=3.0)
    r1 = estimate_load_inertia(base)
    r9 = estimate_load_inertia(geared)
    assert r1.j_joint_kgm2 == pytest.approx(r9.j_joint_kgm2)          # joint side identical
    assert r1.j_reflected_kgm2 == pytest.approx(9.0 * r9.j_reflected_kgm2)
    assert r1.ratio_pct == pytest.approx(9.0 * r9.ratio_pct)


def test_ratio_inversely_scales_with_rotor_inertia():
    a = estimate_load_inertia(InertiaInputs(rotor_inertia_kgm2=1e-4))
    b = estimate_load_inertia(InertiaInputs(rotor_inertia_kgm2=2e-4))
    assert a.ratio_pct == pytest.approx(2.0 * b.ratio_pct)


def test_proximal_bar_term_matches_rod_about_end():
    # With no distal link and no payload, J_joint is exactly (1/3) m1 L1^2.
    inp = InertiaInputs(l2_mm=1e-6, payload_g=0.0)     # ~zero distal mass
    est = estimate_load_inertia(inp)
    l1 = inp.l1_mm / 1000.0
    expected_j1 = (1.0 / 3.0) * est.m1_kg * l1 * l1
    assert est.j_joint_kgm2 == pytest.approx(expected_j1, rel=1e-3)


def test_heavier_payload_raises_ratio():
    light = estimate_load_inertia(InertiaInputs(payload_g=0.0))
    heavy = estimate_load_inertia(InertiaInputs(payload_g=500.0))
    assert heavy.ratio_pct > light.ratio_pct


def test_c0006_is_rounded_ratio_and_plausible():
    est = estimate_load_inertia()          # verified-geometry defaults
    assert est.c0006 == round(est.ratio_pct)
    # sanity: a real aluminium 5-bar through 3:1 lands in a sane C00.06 band,
    # not 0 and not absurd (the drive accepts 0-12000%).
    assert 1 <= est.c0006 <= 12000


def test_bad_inputs_rejected():
    for bad in (dict(reduction=0.0), dict(rotor_inertia_kgm2=0.0), dict(l1_mm=0.0)):
        with pytest.raises(ValueError):
            InertiaInputs(**bad)
