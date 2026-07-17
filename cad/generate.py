"""Parametric CAD for the 5-bar arm hardware — regenerates every STEP + preview.

The geometry here is tied to the software's source of truth: link lengths, base
spacing, and the home pose come from ``FiveBarConfig`` / ``FiveBarKinematics``,
and the link-collision check sweeps the same ``WorkspaceValidator`` the runtime
uses, so the mechanical design and the motion software cannot silently drift.

Level assignment (the "cross" arrangement — distal of one side pairs with the
proximal of the other):

    plane A (z111-141): proximal L  +  distal R
    plane B (z146-176): proximal R  +  distal L

  * the two proximals are on different planes -> they can never collide;
  * the two distals land on adjacent planes -> they stack naturally at the TCP
    (full-height bosses, shared pin, one 5 mm spacer);
  * each distal shares a plane only with the OPPOSITE proximal, and the sweep
    below proves those cross pairs keep >80 mm centerline clearance everywhere
    the workspace validator allows the TCP to be;
  * proximals are identical parts (the right one clamps higher on the shaft's
    long D-flat); distals are identical parts.

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
XM = HX + C                            # motor axis x (160.8)
ZL, ZR = 5.0, 25.0                     # 60T body bottom z: left low / right high
PW = 15.0                              # pulley/belt face width
PLANE_A, PLANE_B = 126.0, 161.0        # arm plane centers: A=111-141, B=146-176
ARM_H = 30.0
MP_T = 8.0                             # motor plate thickness
MPL_TOP, MPR_TOP = 1.0, 21.0           # motor plate top faces
WALL_T, WALL_Y = 10.0, 50.0            # mount shear walls: thickness, centerline +/-y
WALL_L, WALL_R = 45.0 - MPL_TOP, 45.0 - MPR_TOP   # wall heights 44 / 24
TENSION = 4.0                          # belt tension adjustment +/- (motor slides on plate)
# TCP joint: hollow spindle so a miniature air cylinder runs through the axis.
# Default cylinder: ISO 6432 O10 bore (O15 barrel) — CHANGE CYL_BARREL_OD to
# your actual cylinder and keep SPINDLE_ID ~1 mm larger.
SPINDLE_OD, SPINDLE_ID = 20.0, 16.2    # hollow TCP spindle (rides in 6804s)
TCP_BRG_OD, TCP_BRG_W = 32.0, 7.0      # 6804-2RS 20x32x7, one per distal boss
CYL_BARREL_OD = 15.0                   # ISO 6432 O10-bore mini cylinder barrel
assert SPINDLE_ID >= CYL_BARREL_OD + 1.0, "cylinder must clear the spindle bore"

# ------------------------------------------------------------------- checks
def check_cross_pairs(grid: float = 8.0) -> float:
    """Min centerline distance of the same-plane cross pairs (proxL-distR and
    proxR-distL) over the ENTIRE validated workspace. Beams are 20-22 wide, so
    anything above ~24 mm is safe; the design point gives >80 mm."""
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


# geometric interference asserts (fail the build, never ship a colliding model)
assert (ZR - 1.5) - (ZL + PW + 1.5) >= 2.0, "staggered 60T pulleys must clear"
assert ZR + PW + 1.5 <= 45.0 - 3.0, "high 60T must clear bottom plate"
assert MPR_TOP - MP_T + 38 < 57.0, "right motor shaft tip inside plate bore"
assert PLANE_B - ARM_H / 2 - (PLANE_A + ARM_H / 2) >= 5.0, "arm plane gap"
_bh = lambda d: PD20 / 2 + (d / C) * ((PD60 - PD20) / 2) + 2.0  # belt half-width
assert (WALL_Y - WALL_T / 2) - _bh(55.0) > 10.0, "mount walls clear the belt run"
assert (WALL_Y - WALL_T / 2) - 40.0 >= 5.0, "mount walls clear the motor body"
_clear = check_cross_pairs()
assert _clear > 24.0, f"cross-pair clearance {_clear:.1f} mm too small"
print(f"layout OK: C={C:.1f} (belt 450-5M-15), wall mounts L={WALL_L:.0f}/R={WALL_R:.0f}, "
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
    clearance bore. ``faces``: 'both' (688 pair), 'top' or 'bottom' (single
    bearing pocketed from that face — used at the TCP so the two distals'
    bearings sit in the OUTER faces for maximum span on the spindle)."""
    arm = arm.cut(zcyl(x, 0, thru, -H, 2 * H))
    if faces in ("both", "top"):
        arm = arm.cut(zcyl(x, 0, od / 2, H / 2 - w, H / 2 + 2))
    if faces in ("both", "bottom"):
        arm = arm.cut(zcyl(x, 0, od / 2, -H / 2 - 2, -H / 2 + w))
    return arm


def pin_clamp(arm, x, r, H):
    arm = arm.cut(zcyl(x, 0, 4.0, -H, 2 * H))
    # The slit must run from OUTSIDE the boss all the way into the bore, or the
    # clamp can't compress: span x-(r+5) .. x+1 (through the bore center), and
    # taller than the boss so the cut is unambiguously through top and bottom.
    arm = arm.cut(cq.Workplane("XY").box(r + 6, 1.4, H + 2)
                  .translate((x - (r + 6) / 2 + 1.0, 0, 0)))
    # Two M3 pinch bolts, stacked at +/-H/4, both crossing the slit so the
    # clamp compresses evenly along the pin instead of cocking.
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
    outer face). Full height everywhere — the cross level assignment needs no
    lap joint. The TCP boss is O40 around a hollow O20 spindle so a miniature
    air cylinder can run THROUGH the TCP axis (cup on-axis = immune to the
    platform's free spin). The plane-A distal is this same part flipped about
    its long axis, which puts its pocket on the bottom face — bearings land in
    the OUTER faces for maximum span."""
    b = cq.Workplane("YZ").polyline(i_pts(20, 30, 3.0, 3.5)).close().extrude(CFG.l2_mm)
    b = b.union(cq.Workplane("XY").workplane(offset=-15).circle(12).extrude(30))
    b = b.union(cq.Workplane("XY").workplane(offset=-15).moveTo(CFG.l2_mm, 0).circle(20).extrude(30))
    return brg_pocket(pin_clamp(b, 0, 12, 30), CFG.l2_mm, 20, 30,
                      od=TCP_BRG_OD, w=TCP_BRG_W, thru=SPINDLE_OD / 2 + 0.5, faces="top")


def shoulder_shaft():
    """O25 x 180. One long D-flat so the SAME part serves both sides: the left
    arm clamps at plane A (111-141), the right at plane B (146-176). The flat
    runs THROUGH the top end (z109 -> past 180) — any full-round section above
    the flat would make it impossible to slide the D-bore arm on from the top."""
    s = cq.Workplane("XY").circle(12.5).extrude(180)
    return s.cut(cq.Workplane("XY").box(40, 40, 76).translate((0, 10.5 + 20, 109 + 38)))


def ring(od, idd, w):
    return (cq.Workplane("XY").circle(od / 2).extrude(w)
            .cut(cq.Workplane("XY").circle(idd / 2).extrude(w + 1).translate((0, 0, -0.5))))


def pulley(T, bore, z):
    pd = T * 5 / math.pi
    p = cq.Workplane("XY").circle(pd / 2 - 0.7).extrude(PW)
    for dz in (-1.5, PW):
        p = p.union(cq.Workplane("XY").circle(pd / 2 + 3.5).extrude(1.5).translate((0, 0, dz)))
    return p.cut(cq.Workplane("XY").circle(bore / 2).extrude(PW + 4).translate((0, 0, -2))).translate((0, 0, z))


def belt(z):
    """Belt band, local frame: shoulder pulley at origin, motor at (-C, 0)."""
    ang = math.degrees(math.asin((PD60 - PD20) / 2 / C))
    def hull(rb, rs):
        pts = [(rb * math.cos(math.radians(a)), rb * math.sin(math.radians(a)))
               for a in np.linspace(-(90 + ang), 90 + ang, 40)]
        pts += [(-C + rs * math.cos(math.radians(a)), rs * math.sin(math.radians(a)))
                for a in np.linspace(90 + ang, 270 - ang, 30)]
        return cq.Workplane("XY").polyline(pts).close().extrude(PW - 1)
    return hull(PD60 / 2 + 1.6, PD20 / 2 + 1.6).cut(hull(PD60 / 2 - 2.2, PD20 / 2 - 2.2)).translate((0, 0, z + 0.5))


def motor():
    """A6M80-750 stand-in: 80 sq flange x10, O70x3 pilot, O19x35 shaft, 80 sq body.
    Standard 80-frame pattern — VERIFY against the boxed datasheet."""
    m = cq.Workplane("XY").box(80, 80, 10).translate((0, 0, -5))
    m = m.union(cq.Workplane("XY").circle(35).extrude(3))
    m = m.union(cq.Workplane("XY").circle(9.5).extrude(38))
    m = m.union(cq.Workplane("XY").box(80, 80, 112).translate((0, 0, -10 - 56)))
    return m.union(cq.Workplane("XY").box(14, 46, 30).translate((-47, 0, -100)))


def motor_plate():
    """Belt tension adjusts HERE: the pilot bore and the four flange holes are
    slots (+/-TENSION along the belt direction), so the motor slides on a rigid,
    FIXED plate — the mount itself never moves. The slot width still registers
    the motor in Y, preserving belt tracking alignment."""
    p = (cq.Workplane("XY").box(110, 110, MP_T)
         .faces(">Z").workplane().slot2D(70.4 + 2 * TENSION, 70.4, 0).cutThruAll())
    bcd = [(45 * math.cos(math.radians(a)), 45 * math.sin(math.radians(a)))
           for a in (45, 135, 225, 315)]
    p = (p.faces(">Z").workplane().pushPoints(bcd)
         .slot2D(6.6 + 2 * TENSION, 6.6, 0).cutThruAll())
    edge = [(dx, sy * WALL_Y) for dx in (-40, 0, 40) for sy in (-1, 1)]
    return p.faces(">Z").workplane().pushPoints(edge).hole(4.5)


def mount_walls(h):
    """Two shear walls per motor, aligned WITH the belt pull (their strong axis)
    — replaces the four O10 posts, ~1000x stiffer in the tension direction."""
    w = cq.Workplane("XY").box(110, WALL_T, h).translate((0, WALL_Y, h / 2))
    return w.union(cq.Workplane("XY").box(110, WALL_T, h).translate((0, -WALL_Y, h / 2)))


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

# ------------------------------------------------------------- full assembly
jt = KIN.inverse(0.0, 250.0)
eL, eR = jt.left_elbow, jt.right_elbow
dLa = math.degrees(math.atan2(250 - eL[1], 0 - eL[0]))
dRa = math.degrees(math.atan2(250 - eR[1], 0 - eR[0]))

P: list = []
plate_b = (cq.Workplane("XY").box(430, 110, 12)
           .faces(">Z").workplane().pushPoints([(HX, 0), (-HX, 0)]).hole(47)
           .faces(">Z").workplane().pushPoints([(XM, 0), (-XM, 0)]).hole(26)
           .faces(">Z").workplane().pushPoints(
               [(sx * XM + dx, sy * WALL_Y) for sx in (-1, 1) for dx in (-40, 0, 40) for sy in (-1, 1)])
           .hole(4.5)
           .faces(">Z").workplane().pushPoints([(x, y) for x in (-205, 205) for y in (-47, 47)]).hole(8)
           ).translate((0, 0, 51))
P.append((plate_b, (0.35, 0.5, 0.65), "bottom_plate"))
plate_t = (cq.Workplane("XY").box(210, 96, 10)
           .faces(">Z").workplane().pushPoints([(HX, 0), (-HX, 0)]).hole(47))
P.append((plate_t.translate((0, 0, 105)), (0.35, 0.5, 0.65), "top_plate"))
P.append((cq.Workplane("XY").pushPoints([(x, y) for x in (-92, 0, 92) for y in (-40, 40)])
          .circle(4.5).extrude(43).translate((0, 0, 57)), (0.6, 0.6, 0.62), "plate_standoffs_43mm"))

for sgn, tag, zp, mp_top, wall_h in ((-1, "L", ZL, MPL_TOP, WALL_L), (1, "R", ZR, MPR_TOP, WALL_R)):
    x, xm = sgn * HX, sgn * XM
    # Rotate the shaft with its arm's home angle so the D-flat actually mates
    # with the arm's D-bore at the home pose (the flat's azimuth on the shaft is
    # set at assembly; the model shows the assembled home state).
    arm_ang = jt.left_deg if sgn < 0 else jt.right_deg
    P.append((shaft.rotate((0, 0, 0), (0, 0, 1), arm_ang).translate((x, 0, 0)),
              (0.55, 0.55, 0.58), f"shaft_{tag}"))
    P.append((ring(47, 25, 12).translate((x, 0, 45)), (0.85, 0.68, 0.2), f"brg7005_lo_{tag}"))
    P.append((ring(47, 25, 12).translate((x, 0, 98)), (0.85, 0.68, 0.2), f"brg7005_up_{tag}"))
    P.append((pulley(60, 25, zp).translate((x, 0, 0)), (0.30, 0.32, 0.36), f"pulley60T_{tag}"))
    P.append((pulley(20, 19, zp).translate((xm, 0, 0)), (0.30, 0.32, 0.36), f"pulley20T_{tag}"))
    bl = belt(zp)
    if sgn > 0:
        bl = bl.mirror("YZ")
    P.append((bl.translate((x, 0, 0)), (0.12, 0.12, 0.14), f"belt_450_5M_{tag}"))
    P.append((motor_plate().translate((xm, 0, mp_top - MP_T / 2)), (0.72, 0.6, 0.42), f"motor_plate_{tag}"))
    P.append((mount_walls(wall_h).translate((xm, 0, mp_top)),
              (0.6, 0.6, 0.62), f"motor_mount_walls_{tag}_{wall_h:.0f}mm"))
    P.append((motor().translate((xm, 0, mp_top - MP_T)), (0.42, 0.44, 0.5), f"motor_A6M80_{tag}"))

# arms — CROSS level assignment: proxL+distR on plane A, proxR+distL on plane B
P.append((prox.rotate((0, 0, 0), (0, 0, 1), jt.left_deg).translate((-HX, 0, PLANE_A)),
          (0.80, 0.82, 0.84), "proximal_L_planeA"))
P.append((prox.rotate((0, 0, 0), (0, 0, 1), jt.right_deg).translate((HX, 0, PLANE_B)),
          (0.80, 0.82, 0.84), "proximal_R_planeB"))
P.append((dist.rotate((0, 0, 0), (0, 0, 1), dLa).translate((eL[0], eL[1], PLANE_B)),
          (0.68, 0.72, 0.76), "distal_L_planeB"))
# Same part flipped about its long axis -> TCP bearing pocket faces DOWN, so
# the two bearings sit in the outer faces of the stacked bosses (max span).
P.append((dist.rotate((0, 0, 0), (1, 0, 0), 180).rotate((0, 0, 0), (0, 0, 1), dRa)
          .translate((eR[0], eR[1], PLANE_A)), (0.68, 0.72, 0.76), "distal_R_planeA"))
P.append((zcyl(eL[0], eL[1], 4, 111, 176), (0.5, 0.5, 0.52), "elbow_pin_L_65mm"))
P.append((zcyl(eR[0], eR[1], 4, 111, 176), (0.5, 0.5, 0.52), "elbow_pin_R_65mm"))
# --- TCP: hollow spindle + through-axis mini air cylinder -------------------
P.append((ring(TCP_BRG_OD, SPINDLE_OD, TCP_BRG_W).translate((0, 250, 111)),
          (0.85, 0.68, 0.2), "brg6804_tcp_lower"))
P.append((ring(TCP_BRG_OD, SPINDLE_OD, TCP_BRG_W).translate((0, 250, 176 - TCP_BRG_W)),
          (0.85, 0.68, 0.2), "brg6804_tcp_upper"))
spindle = (zcyl(0, 250, SPINDLE_OD / 2, 107, 182)
           .union(zcyl(0, 250, 12, 106.5, 111))            # bottom retaining flange
           .cut(zcyl(0, 250, SPINDLE_ID / 2, 100, 190)))
P.append((spindle, (0.55, 0.55, 0.58), "tcp_spindle_O20xO16"))
P.append((ring(28, SPINDLE_OD + 0.2, 8).translate((0, 250, 176.5)),
          (0.5, 0.5, 0.52), "tcp_collar"))
cyl = (zcyl(0, 250, CYL_BARREL_OD / 2, 128, 190)           # barrel (ports up top)
       .union(zcyl(0, 250, 4, 122, 128))                   # nose
       .union(zcyl(0, 250, 2, 70, 122)))                   # rod, extended
P.append((cyl, (0.42, 0.44, 0.5), "air_cyl_ISO6432_O10"))
P.append((cq.Workplane("XY").workplane(offset=58).moveTo(0, 250).circle(9)
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
      "DUAL-SHOULDER BASE - cross level assignment, home pose",
      ["plane A: proximal L + distal R | plane B: proximal R + distal L -> distals stack at TCP, no lap joint",
       f"cross-pair clearance >{_clear:.0f}mm everywhere the validator allows; belts 450-5M-15 C={C:.1f}, wall mounts 44/24"],
      np.array([-0.3, -0.5, 0.81]), np.array([0.75, 0.7, 1.2]))
paint("base_front.png", lambda p: (p[0], -p[2]), lambda p: -p[1], (1560, 760), 1.9,
      "FRONT ELEVATION - staggered belt planes below, crossed arm planes above",
      ["left belt z5-20 (44mm walls) / right belt z25-40 (24mm walls) | plane A z111-141, plane B z146-176",
       "identical arms both sides: right proximal simply clamps higher on the shaft's long D-flat"],
      np.array([-0.25, -0.75, 0.55]), np.array([0.15, -1.0, 0.25]))
near_tcp = lambda t: sum(np.linalg.norm(t[j][:2] - np.array([0.0, 250.0])) < 240 for j in range(3)) == 3
paint("tcp_closeup.png", iso, lambda p: p[0] + p[1] + p[2], (1240, 780), 2.4,
      "TCP - through-axis air cylinder in a hollow spindle",
      ["O20/O16 spindle rides 2x 6804 in the stacked bosses' outer faces; mini cylinder drops through,",
       "cup on the joint axis (immune to platform spin); barrel + port stay accessible above the collar"],
      np.array([-0.3, -0.5, 0.81]), np.array([0.75, 0.7, 1.2]), keep=near_tcp)
print("previews written to docs/cad/")
