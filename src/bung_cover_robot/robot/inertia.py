"""Geometry-derived load-inertia estimate for the A6 drive's load inertia ratio
(``C00.06`` / CoE ``0x2000:07``).

The drive's *native* inertia auto-tune (``F30.10``) measures load inertia by
free-spinning one axis several turns. That is unsafe on the assembled 5-bar: the
two shoulders are coupled through the linkage to a shared TCP, so no axis spins
freely — the routine would fight the mechanism, exceed the joint window, and
return a meaningless number. So we estimate the inertia from the *geometry* we
already own (link lengths + arm cross-section + a lumped payload) and reflect it
to the motor shaft through the belt reduction.

This is an ESTIMATE. The reflected inertia of a parallel 5-bar is pose-dependent
and the closed-chain coupling is approximated here (proximal link exact; distal
link + half the payload lumped at the elbow radius — a slight over-estimate).
Treat the result as a *starting* ``C00.06`` — far better than the drive's 100 %
default — and confirm empirically with the tuning assistant's following-error
measurement. Load inertia ratio only needs to be in the right ballpark (a servo
auto-tune tolerates roughly ±2×) for the gains and feedforward to behave.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class InertiaInputs:
    """Everything the estimate needs. Defaults are the verified geometry
    (``robot_config.yaml`` link lengths, ``cad/params.py`` ``ARM_H``) plus
    conservative material/payload guesses. ``rotor_inertia_kgm2`` is a MOTOR
    spec — override it with your A6M80-750 datasheet value; the ratio is only as
    trustworthy as that number."""

    l1_mm: float = 200.0            # proximal link (shoulder -> elbow)
    l2_mm: float = 230.0            # distal link (elbow -> TCP)
    arm_width_mm: float = 25.0      # arm cross-section width
    arm_height_mm: float = 30.0     # arm cross-section height (cad ARM_H)
    arm_density_kg_m3: float = 2700.0   # 6061 aluminium
    payload_g: float = 150.0        # bung cover + vacuum tool at the TCP
    reduction: float = 3.0          # belt ratio N (motor rev : joint rev)
    rotor_inertia_kgm2: float = 1.4e-4  # A6M80-750 rotor Jm — SET from datasheet

    def __post_init__(self) -> None:
        if self.reduction <= 0:
            raise ValueError("reduction must be positive")
        if self.rotor_inertia_kgm2 <= 0:
            raise ValueError("rotor_inertia_kgm2 must be positive")
        if self.l1_mm <= 0 or self.l2_mm <= 0:
            raise ValueError("link lengths must be positive")


def _bar_mass_kg(length_mm: float, w_mm: float, h_mm: float, density: float) -> float:
    """Mass of a rectangular bar, mm dimensions -> kg."""
    volume_m3 = (length_mm * w_mm * h_mm) * 1e-9      # mm^3 -> m^3
    return volume_m3 * density


@dataclass(frozen=True)
class InertiaEstimate:
    m1_kg: float
    m2_kg: float
    j_joint_kgm2: float          # load inertia at ONE shoulder joint
    j_reflected_kgm2: float      # ... reflected to the motor shaft (÷ N^2)
    ratio_pct: float             # j_reflected / rotor inertia * 100
    c0006: int                   # ratio rounded — the value to write to 0x2000:07


def estimate_load_inertia(inp: InertiaInputs | None = None) -> InertiaEstimate:
    """Load inertia reflected to one shoulder motor, as a % of rotor inertia.

    Model (per shoulder):
      * proximal link L1 — slender bar rotating about the shoulder pivot:
        ``J1 = (1/3) m1 L1^2`` (exact for a uniform bar about its end).
      * distal link L2 + half the TCP payload — lumped as a point mass at the
        elbow (radius L1): ``J2 = (m2 + payload/2) L1^2``. The other arm carries
        the payload's other half. This slightly over-estimates (ignores that the
        distal does not rotate rigidly with the proximal), which is the safe
        direction for a starting inertia ratio.
    Reflect to the motor through the belt: ``J_motor = J_joint / N^2``.
    """
    inp = inp or InertiaInputs()
    l1_m = inp.l1_mm / 1000.0
    m1 = _bar_mass_kg(inp.l1_mm, inp.arm_width_mm, inp.arm_height_mm, inp.arm_density_kg_m3)
    m2 = _bar_mass_kg(inp.l2_mm, inp.arm_width_mm, inp.arm_height_mm, inp.arm_density_kg_m3)
    payload_kg = inp.payload_g / 1000.0

    j1 = (1.0 / 3.0) * m1 * l1_m * l1_m
    j2 = (m2 + 0.5 * payload_kg) * l1_m * l1_m
    j_joint = j1 + j2
    j_reflected = j_joint / (inp.reduction ** 2)
    ratio = 100.0 * j_reflected / inp.rotor_inertia_kgm2
    return InertiaEstimate(
        m1_kg=m1, m2_kg=m2, j_joint_kgm2=j_joint,
        j_reflected_kgm2=j_reflected, ratio_pct=ratio,
        c0006=int(round(ratio)),
    )
