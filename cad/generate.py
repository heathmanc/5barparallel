"""Parametric CAD for the 5-bar arm hardware — regenerates every STEP + preview.

The geometry here is tied to the software's source of truth: link lengths, base
spacing, and the home pose come from ``FiveBarConfig`` / ``FiveBarKinematics``,
and the link-collision check sweeps the same ``WorkspaceValidator`` the runtime
uses, so the mechanical design and the motion software cannot silently drift.

Layout (this revision):

  * Motors sit BEHIND the shoulders (belts run rearward in -Y), splayed out by
    SPLAY just enough that the two 80-frame bodies clear each other — they are
    NOT exactly behind each shaft, which is fine. This collapses the base deck
    from 430 mm wide to a footprint that prints on a 300x300 bed.
  * Belt tension is a JACKSCREW SLIDER: each motor bolts to a carriage that
    slides on two rails along the belt axis; a jackscrew on the shoulder side
    pushes the carriage away to tension, then lock bolts clamp it. The lock
    bolts and rails sit outside the belt corridor (asserted).
  * Each 7005 angular-contact bearing is trapped by a printed cap on the top
    AND bottom face of its plate, bolted together on a Ø58 BCD — the outer race
    is captured both ways; shaft preload is by shim + locknut.
  * The elbow pins get a bottom head and a top retaining clip so no link (the
    flipped lower distal included) can walk off its pin.

Cross level assignment (distal of one side pairs with the proximal of the
other): plane A = proximal L + distal R; plane B = proximal R + distal L. The
two proximals are on different planes so they can never collide; the two distals
land on adjacent planes so they stack at the TCP.

Regenerate:  pip install cadquery   (only needed for CAD, not the runtime)
             python cad/generate.py
Outputs:     cad/step/*.step   docs/cad/*.png
"""

from __future__ import annotations

import math
from pathlib import Path

import cadquery as cq
import numpy as np
from cadquery import Assembly, Color

from bung_cover_robot.robot.fivebar_kinematics import FiveBarKinematics
from bung_cover_robot.robot.workspace import WorkspaceValidator

ROOT = Path(__file__).resolve().parents[1]
STEP = ROOT / "cad" / "step"
IMG = ROOT / "docs" / "cad"
STEP.mkdir(parents=True, exist_ok=True)
IMG.mkdir(parents=True, exist_ok=True)

KIN = FiveBarKinematics()
VAL = WorkspaceValidator(KIN)
CFG = KIN.config

# ---------------------------------------------------------------- layout (mm)
HX = CFG.base_spacing_mm / 2          # shoulder half-spacing (40)
PD20, PD60 = 20 * 5 / math.pi, 60 * 5 / math.pi   # HTD-5M pitch dias
BELT_LEN = 450.0                       # stock 450-5M-15 (90T)
_wrap = math.pi * (PD20 + PD60) / 2
_k = (PD60 - PD20) ** 2 / 4
C = ((BELT_LEN - _wrap) + math.sqrt((BELT_LEN - _wrap) ** 2 - 8 * _k)) / 4

# Motors BEHIND the shoulders (belts rearward). Two 80-frame bodies can't both
# sit exactly behind shafts 80 mm apart, so each is splayed out by SPLAY from
# the -Y axis just enough to clear — approximately-behind is acceptable.
SPLAY = math.radians(16.0)
MXx = HX + C * math.sin(SPLAY)         # motor axis x  (~73.3): splayed outward
MXy = -C * math.cos(SPLAY)             # motor axis y  (~-116): behind the shafts

ZL, ZR = 5.0, 25.0                     # belt plane bottoms (staggered so the two 60T pass)
PW = 15.0                              # pulley / belt face width
ARM_H = 30.0

# --- jackscrew-slider tensioner -------------------------------------------- #
MP_T = 8.0                             # carriage plate thickness
MPL_TOP, MPR_TOP = 1.0, 21.0           # carriage top faces (staggered with the belts)
TENSION = 6.0                          # jackscrew travel along the belt (+/-)
CAR_L, CAR_W = 96.0, 108.0             # carriage: length (along belt) x width
SLOT_V = 40.0                          # lock-slot lateral offset from the belt axis
RAIL_V = 52.0                          # guide-rail lateral offset from the belt axis

# --- bottom-up Z stack (derived, so the bearing caps can't silently collide) -
CAP_T = 5.0                            # bearing-cap thickness
BRG_W = 12.0                           # 7005 width (= deck thickness)
DECK_T = 12.0
DECK_Z0 = ZR + PW + 12.0               # deck bottom, clear above the high belt (52)
DECK_Z1 = DECK_Z0 + DECK_T             # 64
STANDOFF = 40.0                        # deck top -> top-plate bottom
TOPP_Z0 = DECK_Z1 + STANDOFF           # top-plate bottom (104)
TOPP_T = 10.0
TOPP_Z1 = TOPP_Z0 + TOPP_T             # 114
LOCKNUT_T = 8.0                        # shaft preload locknut, above the upper top cap
# stack above the upper bearing: plate -> top cap (CAP_T) -> locknut -> arm
UCAP_TOP = TOPP_Z1 + CAP_T             # top of the upper bearing's top cap (119)
LOCKNUT_Z = UCAP_TOP                   # locknut sits directly on the top cap
ARM_GAP = 4.0
PLANE_A = LOCKNUT_Z + LOCKNUT_T + ARM_GAP + ARM_H / 2   # lower arm plane center (146)
PLANE_B = PLANE_A + 35.0                                # upper arm plane center (181)
SHAFT_LEN = 205.0
A0, A1 = PLANE_A - ARM_H / 2, PLANE_A + ARM_H / 2   # 125..155
B0, B1 = PLANE_B - ARM_H / 2, PLANE_B + ARM_H / 2   # 160..190

# TCP joint: hollow spindle so a miniature air cylinder runs through the axis.
SPINDLE_OD, SPINDLE_ID = 20.0, 16.2
TCP_BRG_OD, TCP_BRG_W = 32.0, 7.0      # 6804-2RS 20x32x7, one per distal boss
CYL_BARREL_OD = 15.0
assert SPINDLE_ID >= CYL_BARREL_OD + 1.0, "cylinder must clear the spindle bore"


# ------------------------------------------------------------------- checks
def check_cross_pairs(grid: float = 8.0) -> float:
    """Min centerline distance of the same-plane cross pairs (proxL-distR and
    proxR-distL) over the ENTIRE validated workspace (XY only — unaffected by
    the Z stack)."""
    bL, bR = CFG.left_base, CFG.right_base

    def segd(p1, p2, p3, p4):
        p1, p2, p3, p4 = (np.asarray(p, float) for p in (p1, p2, p3, p4))
        cr = lambda a, b: a[0] * b[1] - a[1] * b[0]
        d1 = cr(p4 - p3, p1 - p3); d2 = cr(p4 - p3, p2 - p3)
        d3 = cr(p2 - p1, p3 - p1); d4 = cr(p2 - p1, p4 - p1)
        if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
            return 0.0
        def pt(p, a, b):
            ab = b - a
            t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0, 1)
            return float(np.linalg.norm(p - (a + t * ab)))
        return min(pt(p1, p3, p4), pt(p2, p3, p4), pt(p3, p1, p2), pt(p4, p1, p2))

    worst = 1e9
    for x in np.arange(-CFG.max_reach_mm, CFG.max_reach_mm + 0.1, grid):
        for y in np.arange(40.0, CFG.max_reach_mm + 0.1, grid):
            if not VAL.validate(float(x), float(y)).ok:
                continue
            jt = KIN.inverse(float(x), float(y))
            worst = min(worst,
                        segd(bL, jt.left_elbow, jt.right_elbow, (x, y)),
                        segd(bR, jt.right_elbow, jt.left_elbow, (x, y)))
    return worst


# belt half-width at distance d (mm) from the 20T (motor) end toward the 60T
_bh = lambda d: PD20 / 2 + (d / C) * ((PD60 - PD20) / 2) + 2.0

# geometric interference asserts (fail the build, never ship a colliding model)
assert (ZR - 1.5) - (ZL + PW + 1.5) >= 2.0, "staggered 60T pulleys must clear"
assert ZR + PW + 1.5 <= DECK_Z0 - 3.0, "high 60T must clear the deck underside"
assert 2 * MXx >= 84.0, "the two motor frames must clear each other behind the shafts"
assert B0 - A1 >= 4.0, "arm plane gap"
assert SLOT_V - _bh(CAR_L / 2) > 6.0, "carriage lock bolts must clear the belt path"
assert RAIL_V - _bh(CAR_L / 2) > 6.0, "guide rails must clear the belt path"
assert (DECK_Z0 - CAP_T - 2.0) - (ZR + PW) >= 3.0, "deck bottom bearing-cap must clear the belt"
assert A0 >= LOCKNUT_Z + LOCKNUT_T + 2.0, "arm boss must clear the shaft locknut"
assert SHAFT_LEN >= B1 + 4.0, "shaft must extend past the upper arm clamp"
_clear = check_cross_pairs()
assert _clear > 24.0, f"cross-pair clearance {_clear:.1f} mm too small"
print(f"layout OK: C={C:.1f} (belt 450-5M-15), motors back+splayed to ({MXx:.0f},{MXy:.0f}), "
      f"deck z{DECK_Z0:.0f}-{DECK_Z1:.0f}, arm planes {PLANE_A:.0f}/{PLANE_B:.0f}, "
      f"cross-pair clearance {_clear:.1f} mm")


# ------------------------------------------------------------- part builders
def i_pts(W, H, tf, tw):
    ww, hh, htw = W / 2, H / 2, tw / 2
    return [(-ww, hh), (ww, hh), (ww, hh - tf), (htw, hh - tf), (htw, -hh + tf),
            (ww, -hh + tf), (ww, -hh), (-ww, -hh), (-ww, -hh + tf),
            (-htw, -hh + tf), (-htw, hh - tf), (-ww, hh - tf)]


def zcyl(x, y, r, z0, z1):
    return cq.Workplane("XY").workplane(offset=z0).moveTo(x, y).circle(r).extrude(z1 - z0)


def dbore(arm, x, r, H, bore, flat, ss):
    rb = bore / 2
    cap = cq.Workplane("XY").box(bore * 2, bore * 2, 3 * H).translate((x, (rb - flat) + bore, 0))
    arm = arm.cut(zcyl(x, 0, rb, -H, 2 * H).cut(cap))
    hole = (cq.Workplane("XY").circle(ss).extrude(r + 4)
            .rotate((0, 0, 0), (1, 0, 0), -90).translate((x, -1, 0)))
    return arm.cut(hole)


def brg_pocket(arm, x, r, H, od=16.0, w=5.0, thru=4.0, faces="both"):
    """Bearing pocket(s) of OD ``od`` x width ``w`` with a ``thru`` radius
    clearance bore. ``faces``: 'both' (688 pair), 'top' or 'bottom'."""
    arm = arm.cut(zcyl(x, 0, thru, -H, 2 * H))
    if faces in ("both", "top"):
        arm = arm.cut(zcyl(x, 0, od / 2, H / 2 - w, H / 2 + 2))
    if faces in ("both", "bottom"):
        arm = arm.cut(zcyl(x, 0, od / 2, -H / 2 - 2, -H / 2 + w))
    return arm


def pin_clamp(arm, x, r, H):
    arm = arm.cut(zcyl(x, 0, 4.0, -H, 2 * H))
    arm = arm.cut(cq.Workplane("XY").box(r + 6, 1.4, H + 2)
                  .translate((x - (r + 6) / 2 + 1.0, 0, 0)))
    for dz in (-H / 4, H / 4):
        bolt = (cq.Workplane("XY").circle(1.7).extrude(2 * r + 6)
                .rotate((0, 0, 0), (1, 0, 0), -90)
                .translate((x - r * 0.55, -(r + 3), dz)))
        arm = arm.cut(bolt)
    return arm


def proximal():
    """L1 link: O25 D-bore shoulder (O42 boss) -> 688 bearing pockets at elbow."""
    b = cq.Workplane("YZ").polyline(i_pts(22, 30, 3.5, 4.0)).close().extrude(CFG.l1_mm)
    b = b.union(cq.Workplane("XY").workplane(offset=-15).circle(21).extrude(30))
    b = b.union(cq.Workplane("XY").workplane(offset=-15).moveTo(CFG.l1_mm, 0).circle(12).extrude(30))
    return brg_pocket(dbore(b, 0, 21, 30, 25.0, 2.0, 2.2), CFG.l1_mm, 12, 30)


def distal():
    """L2 link: O8 pin clamp at elbow -> 6804 bearing at TCP (single pocket,
    outer face). The plane-A distal is this same part flipped about its long
    axis, so its TCP pocket faces down (bearings land in the OUTER faces)."""
    b = cq.Workplane("YZ").polyline(i_pts(20, 30, 3.0, 3.5)).close().extrude(CFG.l2_mm)
    b = b.union(cq.Workplane("XY").workplane(offset=-15).circle(12).extrude(30))
    b = b.union(cq.Workplane("XY").workplane(offset=-15).moveTo(CFG.l2_mm, 0).circle(20).extrude(30))
    return brg_pocket(pin_clamp(b, 0, 12, 30), CFG.l2_mm, 20, 30,
                      od=TCP_BRG_OD, w=TCP_BRG_W, thru=SPINDLE_OD / 2 + 0.5, faces="top")


def shoulder_shaft():
    """O25 x 195. One long D-flat serves both sides (left arm clamps at plane A,
    right at plane B). The flat runs from below plane A through the top so a
    D-bore arm can slide on from the top."""
    s = cq.Workplane("XY").circle(12.5).extrude(SHAFT_LEN)
    flat_z0, flat_len = A0 - 4, SHAFT_LEN - (A0 - 4)
    return s.cut(cq.Workplane("XY").box(40, 40, flat_len * 2)
                 .translate((0, 10.5 + 20, flat_z0 + flat_len)))


def ring(od, idd, w):
    return (cq.Workplane("XY").circle(od / 2).extrude(w)
            .cut(cq.Workplane("XY").circle(idd / 2).extrude(w + 1).translate((0, 0, -0.5))))


def bearing_cap():
    """Printed retainer that traps a 7005 outer race. Bolts on a Ø58 BCD; a
    Ø46.6 register spigot centers it in the Ø47 bore; the spigot's outer ring
    (Ø41-46.6) presses ONLY the outer race, relieved inboard to clear the
    rotating inner race; Ø27 shaft clearance. Built spigot-up; flip for the top
    cap so top+bottom bolt together through the plate and capture the race both
    ways."""
    cap = zcyl(0, 0, 31, 0, CAP_T)                        # OD62 disc
    cap = cap.union(zcyl(0, 0, 23.3, CAP_T, CAP_T + 2))   # Ø46.6 register spigot
    cap = cap.cut(zcyl(0, 0, 13.5, -1, CAP_T + 3))        # Ø27 shaft clearance
    cap = cap.cut(zcyl(0, 0, 20.5, CAP_T + 0.5, CAP_T + 3))  # relieve inboard of Ø41 (clear inner race)
    for a in (45, 135, 225, 315):
        bx, by = 29 * math.cos(math.radians(a)), 29 * math.sin(math.radians(a))
        cap = cap.cut(zcyl(bx, by, 2.2, -1, CAP_T + 3))   # 4x M4 on Ø58 BCD
    return cap


def pulley(T, bore, z):
    pd = T * 5 / math.pi
    p = cq.Workplane("XY").circle(pd / 2 - 0.7).extrude(PW)
    for dz in (-1.5, PW):
        p = p.union(cq.Workplane("XY").circle(pd / 2 + 3.5).extrude(1.5).translate((0, 0, dz)))
    return p.cut(cq.Workplane("XY").circle(bore / 2).extrude(PW + 4).translate((0, 0, -2))).translate((0, 0, z))


def belt(z):
    """Belt band, local frame: 60T at origin, 20T at (-C, 0)."""
    ang = math.degrees(math.asin((PD60 - PD20) / 2 / C))
    def hull(rb, rs):
        pts = [(rb * math.cos(math.radians(a)), rb * math.sin(math.radians(a)))
               for a in np.linspace(-(90 + ang), 90 + ang, 40)]
        pts += [(-C + rs * math.cos(math.radians(a)), rs * math.sin(math.radians(a)))
                for a in np.linspace(90 + ang, 270 - ang, 30)]
        return cq.Workplane("XY").polyline(pts).close().extrude(PW - 1)
    return hull(PD60 / 2 + 1.6, PD20 / 2 + 1.6).cut(hull(PD60 / 2 - 2.2, PD20 / 2 - 2.2)).translate((0, 0, z + 0.5))


def motor(mp_top):
    """A6M80-750 stand-in: 80 sq flange x10, O70x3 pilot, O19x38 shaft, 80 sq
    body. Flange top sits under the carriage (top face at mp_top)."""
    dz = mp_top - MP_T - 5
    m = cq.Workplane("XY").box(80, 80, 10).translate((0, 0, -5))
    m = m.union(cq.Workplane("XY").circle(35).extrude(3))
    m = m.union(cq.Workplane("XY").circle(9.5).extrude(38))
    m = m.union(cq.Workplane("XY").box(80, 80, 112).translate((0, 0, -10 - 56)))
    return m.translate((0, 0, dz))


def motor_carriage(mp_top):
    """The motor bolts to this; it slides on the rails along the belt axis (x_l)
    to tension. Top face at mp_top. Pilot bore + Ø90-BCD clearance for the
    motor; two tension lock-slots (along x_l) at +/-SLOT_V, well outside the
    belt corridor."""
    t = MP_T
    p = cq.Workplane("XY").box(CAR_L, CAR_W, t).translate((0, 0, mp_top - t / 2))
    p = p.cut(zcyl(0, 0, 35.5, mp_top - t - 1, mp_top + 1))          # Ø71 pilot
    for a in (45, 135, 225, 315):
        bx, by = 45 * math.cos(math.radians(a)), 45 * math.sin(math.radians(a))
        p = p.cut(zcyl(bx, by, 3.4, mp_top - t - 1, mp_top + 1))     # 4x Ø6.8 flange
    for sy in (SLOT_V, -SLOT_V):
        slot = (cq.Workplane("XY").slot2D(2 * TENSION + 5.5, 5.5, 0).extrude(t + 2)
                .translate((0, sy, mp_top - t - 1)))
        p = p.cut(slot)
    return p


def carriage_rails(mp_top):
    """Two rails (along the belt axis) from the deck underside down to the
    carriage — they take belt-tension shear so the lock bolts only need to
    clamp. Outside the belt corridor at +/-RAIL_V."""
    h = DECK_Z0 - mp_top
    rail = cq.Workplane("XY").box(CAR_L + 2 * TENSION + 16, 6, h)
    w = rail.translate((0, RAIL_V, mp_top + h / 2))
    w = w.union(rail.translate((0, -RAIL_V, mp_top + h / 2)))
    return w


def tension_block(mp_top):
    """Fixed block on the SHOULDER side of the carriage. A jackscrew threads
    through it (Ø5 clearance / tap M6) and its tip pushes the carriage away from
    the shoulder to tension; belt tension keeps the screw in compression."""
    x0 = -(CAR_L / 2 + TENSION + 12)
    blk = (cq.Workplane("XY").box(12, 2 * RAIL_V, DECK_Z0 - (mp_top - MP_T))
           .translate((x0, 0, (DECK_Z0 + mp_top - MP_T) / 2)))
    screw = (cq.Workplane("YZ").circle(3.0).extrude(30)
             .translate((x0 - 15, 0, mp_top - MP_T / 2)))
    return blk.cut(screw)


# ------------------------------------------------------------- single parts
def export(wp, name):
    s = wp.val()
    assert s.isValid(), name
    cq.exporters.export(wp, str(STEP / f"{name}.step"))
    print(f"{name}: vol {s.Volume()/1000:6.1f} cm3  Al {s.Volume()*2.7/1000:4.0f} g  "
          f"PA12 {s.Volume()*1.01/1000:3.0f} g")
    return wp


prox = export(proximal(), "proximal_arm")
dist = export(distal(), "distal_arm")
shaft = export(shoulder_shaft(), "shoulder_shaft")
cap = export(bearing_cap(), "bearing_cap")

# ------------------------------------------------------------- full assembly
jt = KIN.inverse(0.0, 250.0)
eL, eR = jt.left_elbow, jt.right_elbow
dLa = math.degrees(math.atan2(250 - eL[1], 0 - eL[0]))
dRa = math.degrees(math.atan2(250 - eR[1], 0 - eR[0]))


def xf(px, py, adeg, ox, oy):
    a = math.radians(adeg)
    return (ox + px * math.cos(a) - py * math.sin(a),
            oy + px * math.sin(a) + py * math.cos(a))


P: list = []

# --- bottom deck: fits 300x300, only deliberate holes ---------------------- #
deck_pts = [(-72, 55), (72, 55), (125, -60), (125, -200), (-125, -200), (-125, -60)]
BCD = [(29 * math.cos(math.radians(a)), 29 * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
STANDOFF_PTS = [(60, 32), (60, -32), (-60, 32), (-60, -32)]
# carriage rail-bolt landing points on the deck underside (per motor)
rail_pts = []
for sgn in (-1, 1):
    adir = math.degrees(math.atan2(-math.cos(SPLAY), sgn * math.sin(SPLAY)))
    mx = sgn * MXx
    for px in (-CAR_L / 2 + 6, CAR_L / 2 - 6):
        for py in (RAIL_V, -RAIL_V):
            rail_pts.append(xf(px, py, adir, mx, MXy))

deck = (cq.Workplane("XY").polyline(deck_pts).close().extrude(DECK_T)
        .edges("|Z").fillet(8))
deck = (deck.faces(">Z").workplane().pushPoints([(HX, 0), (-HX, 0)]).hole(47)          # shoulder bearings
        .faces(">Z").workplane().pushPoints([(sx + bx, by) for sx in (HX, -HX) for bx, by in BCD]).hole(4.4)  # cap bolts
        .faces(">Z").workplane().pushPoints(STANDOFF_PTS).hole(5.2)                     # standoff bolts
        .faces(">Z").workplane().pushPoints(rail_pts).hole(4.4))                        # carriage rail bolts
deck = deck.translate((0, 0, DECK_Z0))
P.append((deck, (0.35, 0.5, 0.65), "bottom_deck"))

# --- top plate (small, over the shoulders) --------------------------------- #
plate_t = (cq.Workplane("XY").box(150, 92, TOPP_T).edges("|Z").fillet(8)
           .faces(">Z").workplane().pushPoints([(HX, 0), (-HX, 0)]).hole(47)
           .faces(">Z").workplane().pushPoints([(sx + bx, by) for sx in (HX, -HX) for bx, by in BCD]).hole(4.4)
           .faces(">Z").workplane().pushPoints(STANDOFF_PTS).hole(5.2))
P.append((plate_t.translate((0, 0, TOPP_Z0)), (0.35, 0.5, 0.65), "top_plate"))

standoffs = cq.Workplane("XY")
for px, py in STANDOFF_PTS:
    standoffs = standoffs.union(ring(12, 5.2, STANDOFF).translate((px, py, DECK_Z1)))
P.append((standoffs, (0.6, 0.6, 0.62), "plate_standoffs_40mm"))

# --- shoulders: shaft, bearings + caps, pulley, belt, tensioner, motor ----- #
for sgn, tag, zp, mp_top in ((-1, "L", ZL, MPL_TOP), (1, "R", ZR, MPR_TOP)):
    sx = sgn * HX
    dirx, diry = sgn * math.sin(SPLAY), -math.cos(SPLAY)
    mx, my = sx + C * dirx, C * diry
    adir = math.degrees(math.atan2(diry, dirx))               # carriage/motor group azimuth
    rot_belt = math.degrees(math.atan2(-diry, -dirx))         # maps belt local (-C,0) -> motor dir

    arm_ang = jt.left_deg if sgn < 0 else jt.right_deg
    P.append((shaft.rotate((0, 0, 0), (0, 0, 1), arm_ang).translate((sx, 0, 0)),
              (0.55, 0.55, 0.58), f"shaft_{tag}"))

    # angular-contact bearings + their capture caps (deck = lower, top plate = upper)
    P.append((ring(47, 25, BRG_W).translate((sx, 0, DECK_Z0)), (0.85, 0.68, 0.2), f"brg7005_lo_{tag}"))
    P.append((ring(47, 25, BRG_W).translate((sx, 0, TOPP_Z0 + TOPP_T / 2 - BRG_W / 2)),
              (0.85, 0.68, 0.2), f"brg7005_up_{tag}"))
    # lower: bottom cap under the deck (spigot up), top cap on the deck (spigot down)
    P.append((cap.translate((sx, 0, DECK_Z0 - CAP_T - 2)), (0.5, 0.5, 0.52), f"brgcap_lo_bot_{tag}"))
    P.append((cap.rotate((0, 0, 0), (1, 0, 0), 180).translate((sx, 0, DECK_Z1 + CAP_T)),
              (0.5, 0.5, 0.52), f"brgcap_lo_top_{tag}"))
    # upper: caps on both faces of the top plate
    P.append((cap.translate((sx, 0, TOPP_Z0 - CAP_T - 2)), (0.5, 0.5, 0.52), f"brgcap_up_bot_{tag}"))
    P.append((cap.rotate((0, 0, 0), (1, 0, 0), 180).translate((sx, 0, TOPP_Z1 + CAP_T)),
              (0.5, 0.5, 0.52), f"brgcap_up_top_{tag}"))
    # shaft preload locknut, on top of the upper top cap (clears the arm above)
    P.append((ring(38, 25, LOCKNUT_T).translate((sx, 0, LOCKNUT_Z)), (0.5, 0.5, 0.52), f"locknut_{tag}"))

    # drive
    P.append((pulley(60, 25, zp).translate((sx, 0, 0)), (0.30, 0.32, 0.36), f"pulley60T_{tag}"))
    P.append((pulley(20, 19, zp).translate((mx, my, 0)), (0.30, 0.32, 0.36), f"pulley20T_{tag}"))
    P.append((belt(zp).rotate((0, 0, 0), (0, 0, 1), rot_belt).translate((sx, 0, 0)),
              (0.12, 0.12, 0.14), f"belt_450_5M_{tag}"))

    # jackscrew-slider tensioner + motor (built in the belt frame, rotated to azimuth)
    place = lambda part: part.rotate((0, 0, 0), (0, 0, 1), adir).translate((mx, my, 0))
    P.append((place(motor_carriage(mp_top)), (0.72, 0.6, 0.42), f"motor_carriage_{tag}"))
    P.append((place(carriage_rails(mp_top)), (0.6, 0.6, 0.62), f"carriage_rails_{tag}"))
    P.append((place(tension_block(mp_top)), (0.6, 0.6, 0.62), f"tension_block_{tag}"))
    P.append((place(motor(mp_top)), (0.42, 0.44, 0.5), f"motor_A6M80_{tag}"))

# --- arms — CROSS level assignment ----------------------------------------- #
P.append((prox.rotate((0, 0, 0), (0, 0, 1), jt.left_deg).translate((-HX, 0, PLANE_A)),
          (0.80, 0.82, 0.84), "proximal_L_planeA"))
P.append((prox.rotate((0, 0, 0), (0, 0, 1), jt.right_deg).translate((HX, 0, PLANE_B)),
          (0.80, 0.82, 0.84), "proximal_R_planeB"))
P.append((dist.rotate((0, 0, 0), (0, 0, 1), dLa).translate((eL[0], eL[1], PLANE_B)),
          (0.68, 0.72, 0.76), "distal_L_planeB"))
P.append((dist.rotate((0, 0, 0), (1, 0, 0), 180).rotate((0, 0, 0), (0, 0, 1), dRa)
          .translate((eR[0], eR[1], PLANE_A)), (0.68, 0.72, 0.76), "distal_R_planeA"))

# --- elbow pins with retention (bottom head + top clip so no link walks off)- #
for (ex, ey), tag in ((eL, "L"), (eR, "R")):
    pin = (zcyl(ex, ey, 4, A0 - 3, B1 + 4)                    # Ø8 pin spanning both planes
           .union(zcyl(ex, ey, 7, A0 - 6, A0 - 3)))          # Ø14 head at the bottom
    P.append((pin, (0.5, 0.5, 0.52), f"elbow_pin_{tag}_75mm"))
    P.append((ring(14, 8.2, 3).translate((ex, ey, B1 + 1)),  # printed top retaining clip
              (0.75, 0.45, 0.30), f"elbow_pin_clip_{tag}"))

# --- TCP: hollow spindle + through-axis mini air cylinder ------------------- #
P.append((ring(TCP_BRG_OD, SPINDLE_OD, TCP_BRG_W).translate((0, 250, A0)),
          (0.85, 0.68, 0.2), "brg6804_tcp_lower"))
P.append((ring(TCP_BRG_OD, SPINDLE_OD, TCP_BRG_W).translate((0, 250, B1 - TCP_BRG_W)),
          (0.85, 0.68, 0.2), "brg6804_tcp_upper"))
spindle = (zcyl(0, 250, SPINDLE_OD / 2, A0 - 4, B1 + 6)
           .union(zcyl(0, 250, 12, A0 - 4.5, A0))            # bottom retaining flange
           .cut(zcyl(0, 250, SPINDLE_ID / 2, A0 - 12, B1 + 14)))
P.append((spindle, (0.55, 0.55, 0.58), "tcp_spindle_O20xO16"))
P.append((ring(28, SPINDLE_OD + 0.2, 8).translate((0, 250, B1 + 0.5)),
          (0.5, 0.5, 0.52), "tcp_collar"))
cyl = (zcyl(0, 250, CYL_BARREL_OD / 2, A1, B1 + 14)
       .union(zcyl(0, 250, 4, A1 - 6, A1))
       .union(zcyl(0, 250, 2, A0 - 40, A1 - 6)))
P.append((cyl, (0.42, 0.44, 0.5), "air_cyl_ISO6432_O10"))
P.append((cq.Workplane("XY").workplane(offset=A0 - 52).moveTo(0, 250).circle(9)
          .workplane(offset=10).moveTo(0, 250).circle(3).loft(),
          (0.75, 0.45, 0.30), "vacuum_cup"))

assy = Assembly()
for s, c, n in P:
    assy.add(s, color=Color(*c), name=n)
assy.export(str(STEP / "dual_base_full.step"))
print(f"dual_base_full.step: {len(P)} named parts")

# ----------------------------------------------------------------- previews
import cv2  # noqa: E402  (project dependency)

BG = (250, 248, 245)
F = cv2.FONT_HERSHEY_SIMPLEX
tris = []
for s, c, n in P:
    col = (int(c[2] * 255), int(c[1] * 255), int(c[0] * 255))
    v, ts = s.val().tessellate(0.4)
    V = np.array([[p.x, p.y, p.z] for p in v])
    for a, b, cc in ts:
        tris.append((V[a], V[b], V[cc], col))


def paint(fname, proj, depth, size, sc, label, notes, Lm, cam, keep=None):
    Lm = Lm / np.linalg.norm(Lm); cam = cam / np.linalg.norm(cam)
    use = [t for t in tris if keep is None or keep(t)]
    allv = np.array([t[j] for t in use for j in range(3)])
    R = np.array([proj(p) for p in allv])
    mx, my = (R[:, 0].min() + R[:, 0].max()) / 2, (R[:, 1].min() + R[:, 1].max()) / 2
    img = np.full((size[1], size[0], 3), BG, np.uint8)
    cx, cy = size[0] // 2, size[1] // 2 + 16
    def PX(p):
        rx, ry = proj(p)
        return (int(cx + (rx - mx) * sc), int(cy + (ry - my) * sc))
    for a, b, cc, col in sorted(use, key=lambda t: depth(t[0]) + depth(t[1]) + depth(t[2])):
        n = np.cross(b - a, cc - a); nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        if np.dot(n, cam) < 0:
            n = -n
        br = 0.42 + 0.58 * max(0.0, float(np.dot(n, Lm)))
        cv2.fillPoly(img, [np.array([PX(a), PX(b), PX(cc)], np.int32)],
                     tuple(int(min(255, ch * br)) for ch in col))
    cv2.putText(img, label, (30, 40), F, 0.6, (60, 55, 50), 2, cv2.LINE_AA)
    for i, s_ in enumerate(notes):
        cv2.putText(img, s_, (30, size[1] - 14 - 22 * (len(notes) - 1 - i)),
                    F, 0.42, (105, 100, 95), 1, cv2.LINE_AA)
    cv2.imwrite(str(IMG / fname), img)


c26, s26 = math.cos(math.radians(26)), math.sin(math.radians(26))
iso = lambda p: ((p[0] - p[1]) * c26, (p[0] + p[1]) * s26 - p[2])
paint("base_iso.png", iso, lambda p: p[0] + p[1] + p[2], (1560, 980), 1.55,
      "DUAL-SHOULDER BASE - motors to the rear, jackscrew-slider tensioners",
      [f"motors splayed behind the shafts at ({MXx:.0f},{MXy:.0f}); deck fits 300x300; 7005s captured by bolted top+bottom caps",
       f"cross-pair clearance >{_clear:.0f}mm everywhere the validator allows; belts 450-5M-15 C={C:.1f}"],
      np.array([-0.3, -0.5, 0.81]), np.array([0.75, 0.7, 1.2]))
paint("base_front.png", lambda p: (p[0], -p[2]), lambda p: -p[1], (1560, 820), 1.7,
      "FRONT ELEVATION - belts staggered below, crossed arm planes above",
      [f"left belt z{ZL:.0f}-{ZL+PW:.0f} / right belt z{ZR:.0f}-{ZR+PW:.0f} | plane A z{A0:.0f}-{A1:.0f}, plane B z{B0:.0f}-{B1:.0f}",
       "7005 shoulder bearings trapped by top+bottom caps bolted through each plate on a Ø58 BCD"],
      np.array([-0.25, -0.75, 0.55]), np.array([0.15, -1.0, 0.25]))
near_tcp = lambda t: sum(np.linalg.norm(t[j][:2] - np.array([0.0, 250.0])) < 240 for j in range(3)) == 3
paint("tcp_closeup.png", iso, lambda p: p[0] + p[1] + p[2], (1240, 780), 2.4,
      "TCP - through-axis air cylinder in a hollow spindle",
      ["O20/O16 spindle rides 2x 6804 in the stacked bosses' outer faces; bottom flange + top collar",
       "capture both distals; elbow pins get a bottom head + top clip so no link can walk off"],
      np.array([-0.3, -0.5, 0.81]), np.array([0.75, 0.7, 1.2]), keep=near_tcp)
print("previews written to docs/cad/")
