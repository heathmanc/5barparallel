"""Single source of truth for the base-hardware layout (pure math, no CAD deps).

Every dimension shared by the solid model (generate.py) and the machine-shop
drawing set (drawings.py) lives HERE, so the model and the paper drawings can
never drift apart. ``check_layout()`` re-derives every clearance — bolt edge
distances, belt-corridor offsets, deck containment, centreline gaps — and
raises AssertionError before either output can be produced from a colliding
layout.

Coordinate frames
-----------------
world  : robot XY. Shoulder shafts at (+-HX, 0), motors behind at (+-MXx, MXy).
         Z up through the deck stack.
local  : per-motor 'belt frame'. a = along the belt axis, positive from the
         motor TOWARD its shoulder; b = lateral, positive OUTBOARD (away from
         the machine centreline).  Right side: world = M + a*U + b*V; the left
         side mirrors x.

Tensioner architecture (this revision)
--------------------------------------
Each motor bolts (4x M6, from below through the flange) to a sliding CARRIAGE
plate.  The carriage rides on a fixed CRADLE: a window frame directly under
the carriage whose two full-length SHEAR FINS rise to the deck underside and
bolt to it (3x M5 per fin through a top flange).  The fins guide the carriage
edges and carry the belt-tension shear into the deck.  Four M5 LOCK BOLTS pass
down through +-6 mm slots in the carriage into the cradle frame right below —
the slots clamp onto real tapped material, not open air.  A JACKSCREW (M6)
through an integral block on the cradle's shoulder-side border pushes the
carriage away from the shoulder to tension; belt pull keeps it in compression.
Everything stays inside the deck outline (asserted) and the whole stack still
prints on a 300x300 bed.
"""

from __future__ import annotations

import math

# --------------------------------------------------------------------- drive
BELT_PITCH = 5.0                       # HTD-5M
T_MOT, T_DRV = 24, 72                  # 3:1 (24T keeps hub wall over the keyway)
PD_MOT = T_MOT * BELT_PITCH / math.pi  # 38.197
PD_DRV = T_DRV * BELT_PITCH / math.pi  # 114.592
BELT_LEN = 450.0                       # stock 450-5M-15 (90T)
_wrap = math.pi * (PD_MOT + PD_DRV) / 2
_k = (PD_DRV - PD_MOT) ** 2 / 4
C = ((BELT_LEN - _wrap) + math.sqrt((BELT_LEN - _wrap) ** 2 - 8 * _k)) / 4

# Motor-shaft pulley bore + keyway (DIN 6885: O19 shaft -> 6 mm key, hub keyseat
# 2.8 mm above the bore). Wall left between keyseat and pulley body OD:
MOTOR_BORE = 19.0
KEY_HUB_DEPTH = 2.8
PULLEY_BODY_R = PD_MOT / 2 - 0.7       # solid body radius under the teeth
PULLEY_KEY_WALL = PULLEY_BODY_R - (MOTOR_BORE / 2 + KEY_HUB_DEPTH)

# ------------------------------------------------------------------- layout
HX = 40.0                              # shoulder half-spacing (base_spacing/2)
SPLAY = math.radians(16.0)             # motor splay out from -Y
SIN_S, COS_S = math.sin(SPLAY), math.cos(SPLAY)
MXx = HX + C * SIN_S                   # motor axis x (right side)
MXy = -C * COS_S                       # motor axis y (behind the shoulders)
# right-side local basis (left mirrors world x)
U = (-SIN_S, COS_S)                    # a: motor -> shoulder
V = (COS_S, SIN_S)                     # b: outboard

ZL, ZR = 5.0, 25.0                     # belt plane bottoms (staggered)
PW = 15.0                              # pulley / belt face width
ARM_H = 30.0

# ------------------------------------------------------- carriage (sliding)
MP_T = 8.0                             # carriage plate thickness
MPL_TOP, MPR_TOP = 1.0, 21.0           # carriage top faces (belt stagger)
TENSION = 6.0                          # jackscrew travel along the belt (+-)
CAR_L, CAR_W = 96.0, 104.0             # carriage: length (a) x width (b)
PILOT_D = 71.0                         # motor pilot bore (O70 spigot + 1)
MOTOR_BCD = 90.0                       # motor flange bolts, M6 at 45 deg
LOCK_A = 30.0                          # lock-bolt line, +- along a
LOCK_B = 46.0                          # lock-bolt line, +- along b (1.5 mm
                                       # insert wall to the window edge)
SLOT_L = 2 * TENSION + 6.5             # 18.5 slot: full +-6 travel with margin
SLOT_W = 5.5                           # M5 clearance slot width
# Tool-access slots THROUGH THE DECK, projected over each lock bolt so the
# heads can be reached with an allen key from the top (they sit ~45 mm below
# the deck; there is no side access). Long axis = the travel direction.
ACC_W = 12.0                           # clears an M5 head (8.5) + 4 mm key wobble
ACC_L = 2 * TENSION + ACC_W            # 24: follows the bolt over +-6 travel

# --------------------------------------------------------- cradle (fixed)
FR_A = 66.0                            # frame half-length (a)
FR_B = 62.0                            # frame half-width (b)
FR_T = 8.0                             # frame plate thickness
WIN_A = 48.0                           # window half-length (motor + travel)
WIN_B = 41.0                           # window half-width (80 flange + 1)
# Fins are SOLID full-depth walls (no web-and-flange section): from the guide
# face at FIN_BI out to the frame edge at FIN_BO, frame bottom to the deck.
# Simpler print, stiffer, and the deck bolts get full-depth heat-sets.
FIN_BI = 52.5                          # fin wall inner face (guides carriage)
FIN_BO = 62.0                          # fin wall outer face = frame edge (9.5 solid)
FIN_A_OUT = 62.0                       # outboard fin wall ends (a)  [both +-]
FIN_A_INN = 19.0                       # inboard fin wall forward end (a) —
                                       # limited by the centreline clip at the
                                       # full 62 width (asserted)
FLG_B = 62.0                           # fin outer edge (= FIN_BO; kept for the
                                       # deck-containment corner points)
FIN_BOLT_B = 57.25                     # fin->deck M5 line (mid-wall)
FIN_BOLTS_OUT = (-52.0, 0.0, 50.0)     # a-positions, outboard fin
FIN_BOLTS_INN = (-52.0, -16.0, 12.0)   # a-positions, inboard fin
BLK_A0, BLK_A1 = 56.0, 66.0            # jack block along a (front border)
BLK_B = 15.0                           # jack block half-width
BLK_H = 8.0                            # jack block above frame top
JACK_Z_OFF = 3.0                       # screw axis above frame top
CL_MARGIN = 1.5                        # keep-out each side of centreline
# Second (deeper) corner cut at the inboard-front corner: the RIGHT frame's
# z-band (5..13) is coplanar with the LEFT 72T pulley flange + belt, and the
# straight centreline clip alone left that corner 0.44 mm INSIDE the opposite
# flange sweep. The cut runs from (CUT_A, on the clip line) to (FR_A, CUT_B),
# keeping every frame point clear of the opposite flange circle + belt strand
# (both asserted below).
CUT_A = 60.0
CUT_B = -43.0

# ------------------------------------------------- z stack (bottom-up, mm)
CAP_T = 5.0                            # bearing-cap thickness
BRG_W = 12.0                           # 7005 width (= deck thickness)
DECK_T = 12.0
DECK_Z0 = ZR + PW + 12.0               # deck bottom (52), clear of high belt
DECK_Z1 = DECK_Z0 + DECK_T             # 64
STANDOFF = 40.0
TOPP_Z0 = DECK_Z1 + STANDOFF           # 104
TOPP_T = 12.0                          # = 7005 width: race sits FLUSH with both
                                       # faces of each plate, caps seat flat
TOPP_Z1 = TOPP_Z0 + TOPP_T             # 116
LOCKNUT_T = 8.0
UCAP_TOP = TOPP_Z1 + CAP_T             # 119
LOCKNUT_Z = UCAP_TOP
ARM_GAP = 4.0
PLANE_A = LOCKNUT_Z + LOCKNUT_T + ARM_GAP + ARM_H / 2
PLANE_B = PLANE_A + 35.0
SHAFT_LEN = 205.0
A0, A1 = PLANE_A - ARM_H / 2, PLANE_A + ARM_H / 2
B0, B1 = PLANE_B - ARM_H / 2, PLANE_B + ARM_H / 2

# ------------------------------------------------------------ bearing caps
# The 7005 race (12 wide) sits FLUSH in its 12-thick plate; each cap is a flat
# disc that bolts down onto the plate face, with a 0.3 mm PROUD annular pad
# that lands on the outer race only — a defined clamp crush, instead of the
# old 2 mm register spigot that held the whole disc off the plate and clamped
# through cap flex.
CAP_OD = 72.0                          # was 62: bolt holes broke out of the OD
CAP_BCD = 58.0                         # 4x M4 through cap+plate+cap
CAP_BOLT_D = 4.5
CAP_PAD_OD = 46.5                      # pad presses the outer race only ...
CAP_PAD_ID = 41.0                      # ... clear of the inner race + cage
CAP_PAD_H = 0.3                        # proud of the disc face (clamp crush)
CAP_SHAFT_CLR = 27.0

# ------------------------------------------------------------------- plates
DECK_PTS = [(-72.0, 62.0), (72.0, 62.0), (147.0, -60.0), (147.0, -178.0),
            (-147.0, -178.0), (-147.0, -60.0)]
BED = 300.0                            # print bed (square)
STANDOFF_PTS = [(64.0, 36.0), (64.0, -36.0), (-64.0, 36.0), (-64.0, -36.0)]
TOPP_W, TOPP_H = 156.0, 92.0           # wide enough that the O72 caps at
                                       # +-40 stay fully on the plate

# TCP joint (unchanged)
SPINDLE_OD, SPINDLE_ID = 20.0, 16.2
TCP_BRG_OD, TCP_BRG_W = 32.0, 7.0
CYL_BARREL_OD = 15.0


# --------------------------------------------------------------- helpers
def belt_half(a: float) -> float:
    """Belt-corridor half-width at ``a`` mm from the motor (+2 mm margin)."""
    return PD_MOT / 2 + (a / C) * ((PD_DRV - PD_MOT) / 2) + 2.0


def clip_b(a: float) -> float:
    """Inboard limit: most-negative allowed b at ``a`` so the part stays
    CL_MARGIN clear of the machine centreline (right side)."""
    return -((MXx - CL_MARGIN) - SIN_S * a) / COS_S


def local_to_world(sgn: int, a: float, b: float):
    """(a, b) in the belt frame of side ``sgn`` (+1 right / -1 left) -> world.
    b is positive OUTBOARD on both sides."""
    x = MXx - SIN_S * a + COS_S * b
    y = MXy + COS_S * a + SIN_S * b
    return (sgn * x, y)


def frame_poly_local():
    """Cradle frame outline in (a, b). Two inboard-front cuts:
    1. the centreline clip (from (a1, -FR_B) along clip_b to (CUT_A, ...)),
       keeping the two cradles CL_MARGIN clear of the machine centreline;
    2. the corner cut to (FR_A, CUT_B), keeping the corner clear of the
       OPPOSITE side's 72T pulley flange and belt strand (they share this
       z-band; asserted in check_layout)."""
    a1 = (MXx - CL_MARGIN - COS_S * FR_B) / SIN_S
    return [(-FR_A, -FR_B), (a1, -FR_B), (CUT_A, clip_b(CUT_A)),
            (FR_A, CUT_B), (FR_A, FR_B), (-FR_A, FR_B)]


def deck_access_slots():
    """Tool-access slots through the deck, one over each carriage lock bolt:
    (id, world_x, world_y, angle_deg) with the long axis along the travel
    direction. ACC_L x ACC_W, THRU."""
    rows = []
    n = 1
    for sgn in (1, -1):
        ang = math.degrees(math.atan2(COS_S, -sgn * SIN_S))
        for sa in (LOCK_A, -LOCK_A):
            for sb in (LOCK_B, -LOCK_B):
                x, y = local_to_world(sgn, sa, sb)
                rows.append((f"A{n}", x, y, ang))
                n += 1
    return rows


def deck_hole_table():
    """Every deliberate hole in the deck: (id, x, y, dia, note)."""
    rows = []
    n = 1
    for sx in (HX, -HX):
        rows.append((f"B{n}", sx, 0.0, 47.0, "7005 bearing bore, thru")); n += 1
    n = 1
    for sx in (HX, -HX):
        for ang in (45, 135, 225, 315):
            bx = sx + CAP_BCD / 2 * math.cos(math.radians(ang))
            by = CAP_BCD / 2 * math.sin(math.radians(ang))
            rows.append((f"C{n}", bx, by, CAP_BOLT_D, "M4 cap bolt, thru")); n += 1
    n = 1
    for px, py in STANDOFF_PTS:
        rows.append((f"S{n}", px, py, 5.2, "M5 standoff bolt, thru")); n += 1
    n = 1
    for sgn in (1, -1):
        for a in FIN_BOLTS_OUT:
            x, y = local_to_world(sgn, a, FIN_BOLT_B)
            rows.append((f"F{n}", x, y, 5.2, "M5 fin bolt, thru")); n += 1
        for a in FIN_BOLTS_INN:
            x, y = local_to_world(sgn, a, -FIN_BOLT_B)
            rows.append((f"F{n}", x, y, 5.2, "M5 fin bolt, thru")); n += 1
    return rows


# ------------------------------------------------------------------ checks
def _dist_pt_seg(p, s0, s1):
    px, py = p; x0, y0 = s0; x1, y1 = s1
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    cx, cy = x0 + t * dx, y0 + t * dy
    return math.hypot(px - cx, py - cy)


def _inside_poly(p, poly):
    x, y = p
    inside = False
    for i in range(len(poly)):
        x0, y0 = poly[i]; x1, y1 = poly[(i + 1) % len(poly)]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def _deck_margin(p):
    d = min(_dist_pt_seg(p, DECK_PTS[i], DECK_PTS[(i + 1) % len(DECK_PTS)])
            for i in range(len(DECK_PTS)))
    return d if _inside_poly(p, DECK_PTS) else -d


def check_layout(verbose: bool = False) -> list:
    """Re-derive every clearance; raise AssertionError on any violation."""
    notes = []

    def ok(cond, margin, what):
        notes.append(f"{'ok ' if cond else 'FAIL'} {what}: {margin:+.2f} mm")
        assert cond, f"{what}: margin {margin:+.2f} mm"

    # -- drive / pulleys ------------------------------------------------
    ok(PULLEY_KEY_WALL >= 3.5, PULLEY_KEY_WALL - 3.5,
       "motor-pulley hub wall over keyway (>=3.5)")
    m = (ZR - 1.5) - (ZL + PW + 1.5)
    ok(m >= 2.0, m - 2.0, "staggered driven pulleys pass in z")
    m = (DECK_Z0 - 3.0) - (ZR + PW + 1.5)
    ok(m >= 0.0, m, "high driven pulley clears deck underside")
    m = (DECK_Z0 - CAP_T) - (ZR + PW + 1.5)
    ok(m >= 3.0, m - 3.0, "deck bottom cap clears the pulley flange")
    rot_half = 40.0 * (COS_S + SIN_S)          # 80-frame rotated by SPLAY
    m = 2 * (MXx - rot_half)
    ok(m >= 6.0, m - 6.0, "motor bodies clear each other at centreline")

    # -- bearing caps -----------------------------------------------------
    edge = CAP_OD / 2 - (CAP_BCD / 2 + CAP_BOLT_D / 2)
    ok(edge >= 3.0, edge - 3.0, "cap bolt hole edge distance (>=3)")
    ok(CAP_OD / 2 - CAP_BCD / 2 >= 1.5 * CAP_BOLT_D,
       CAP_OD / 2 - CAP_BCD / 2 - 1.5 * CAP_BOLT_D, "cap bolt 1.5d edge rule")
    m = 2 * HX - CAP_OD
    ok(m >= 4.0, m - 4.0, "adjacent bearing caps clear each other")
    m = CAP_BCD / 2 - CAP_BOLT_D / 2 - 23.5
    ok(m >= 2.5, m - 2.5, "cap bolt hole to O47 plate bore wall")
    so = math.hypot(STANDOFF_PTS[0][0] - HX, STANDOFF_PTS[0][1])
    m = so - CAP_OD / 2 - 6.0
    ok(m >= 1.0, m - 1.0, "cap clears standoff boss")
    m = TOPP_W / 2 - (HX + CAP_OD / 2)
    ok(m >= 1.0, m - 1.0, "top plate fully covers the O72 caps")
    m = CAP_BCD / 2 - CAP_BOLT_D / 2 - CAP_PAD_OD / 2
    ok(m >= 2.0, m - 2.0, "cap bolt hole to pressing pad")
    # races must sit flush so the flat caps seat on the plate faces
    ok(abs(DECK_T - BRG_W) < 0.01, 0.0, "deck thickness = 7005 race width")
    ok(abs(TOPP_T - BRG_W) < 0.01, 0.0, "top plate thickness = 7005 race width")
    m = 47.0 - CAP_PAD_OD
    ok(m >= 0.3, m - 0.3, "pressing pad stays inside the bore edge")

    # -- carriage ---------------------------------------------------------
    m = CAR_W / 2 - (LOCK_B + SLOT_W / 2)
    ok(m >= 3.0, m - 3.0, "carriage slot to carriage edge")
    m = LOCK_B - 3.5 - WIN_B
    ok(m >= 1.4, m - 1.4, "cradle lock-bolt boss to window edge")
    m = (LOCK_B - 5.0) - belt_half(LOCK_A + SLOT_L / 2)
    ok(m >= 3.0, m - 3.0, "lock bolt head clear of belt corridor")
    m = FIN_BI - (LOCK_B + 5.0)
    ok(m >= 1.45, m - 1.45, "lock bolt head clear of fin wall")
    m = SLOT_L / 2 - (TENSION + 2.5)
    ok(m >= 0.5, m - 0.5, "slot end clear of the bolt shank at full travel")
    m = WIN_B - 40.0
    ok(m >= 1.0, m - 1.0, "window clears motor flange laterally")
    m = WIN_A - (40.0 + TENSION)
    ok(m >= 1.5, m - 1.5, "window clears motor flange at travel extremes")
    m = FIN_BI - CAR_W / 2
    ok(0.3 <= m <= 1.0, m - 0.3, "carriage-to-fin guide clearance (0.3..1.0)")
    m = math.hypot(MOTOR_BCD / 2 * COS_S * 0 + MOTOR_BCD / 2 / math.sqrt(2), 0)
    m = CAR_W / 2 - (MOTOR_BCD / 2 / math.sqrt(2) + 4.0)
    ok(m >= 8.0, m - 8.0, "motor bolt boss inside carriage")
    for zp, mp_top, tag in ((ZL, MPL_TOP, "L"), (ZR, MPR_TOP, "R")):
        m = (zp - 1.5) - mp_top
        ok(m >= 2.0, m - 2.0, f"pulley lower flange clears carriage top ({tag})")

    # -- cradle / fins ------------------------------------------------------
    m = FIN_BI - belt_half(FIN_A_OUT)
    ok(m >= 3.0, m - 3.0, "outboard fin wall clear of belt corridor")
    m = FIN_BI - belt_half(FIN_A_INN)
    ok(m >= 3.0, m - 3.0, "inboard fin wall clear of belt corridor")
    # fin wall nearest corner vs the O113 driven pulley + flange
    fx, fy = local_to_world(1, FIN_A_OUT, FIN_BI)
    m = math.hypot(fx - HX, fy) - (PD_DRV / 2 + 3.5)
    ok(m >= 2.0, m - 2.0, "outboard fin end clears 72T pulley flange")
    for zp, mp_top, tag in ((ZL, MPL_TOP, "L"), (ZR, MPR_TOP, "R")):
        frame_top = mp_top - MP_T
        # datum is the pulley FLANGE bottom (zp - 1.5), which reaches 1.5 mm
        # below the belt band and sweeps over the block/nut footprint
        m = (zp - 1.5) - (frame_top + BLK_H)
        ok(m >= 2.0, m - 2.0, f"jack block top under pulley flange ({tag})")
        nut_top = frame_top + JACK_Z_OFF + 5.55       # M6 jam nut across corners
        m = (zp - 1.5) - nut_top
        ok(m >= 1.4, m - 1.4, f"jackscrew jam nut under pulley flange ({tag})")
    # inboard fin / flange forward ends respect the centreline clip
    m = -FIN_BO - clip_b(FIN_A_INN)
    ok(m >= 0.5, m - 0.5, "inboard fin end inside centreline clip")

    # -- centreline gaps -----------------------------------------------------
    # nearest carriage corner to x=0 is the front-inboard one at full travel
    car_x, _ = local_to_world(1, CAR_L / 2 + TENSION, -CAR_W / 2)
    ok(2 * car_x >= 3.0, 2 * car_x - 3.0, "left/right carriages gap at centreline")
    fp = frame_poly_local()
    fmin = min(local_to_world(1, a, b)[0] for a, b in fp)
    ok(fmin >= CL_MARGIN - 0.1, fmin - CL_MARGIN, "cradle frame at centreline clip")

    # -- CROSS-SIDE: the right frame (z 5..13 at MPR) shares the z-band of the
    # LEFT 72T pulley flange (zp-1.5 up) and the LEFT belt. Every frame point
    # (vertices + edge midpoints) must clear the opposite flange circle AND
    # the opposite belt's inboard strand. This is the one place the geometry
    # crosses sides — the straight centreline clip alone does NOT cover it.
    fpts = list(fp) + [((fp[i][0] + fp[(i + 1) % len(fp)][0]) / 2,
                        (fp[i][1] + fp[(i + 1) % len(fp)][1]) / 2)
                       for i in range(len(fp))]
    wpts = [local_to_world(1, a, b) for a, b in fpts]
    flange_r = PD_DRV / 2 + 3.5
    m = min(math.hypot(px + HX, py) for px, py in wpts) - flange_r
    ok(m >= 1.5, m - 1.5, "frame corner clears the OPPOSITE 72T flange sweep")
    # opposite belt inboard strand = outer tangent between the two belt-back
    # circles of the LEFT drive; clearance is the signed distance to that line
    c1, r1 = (-HX, 0.0), PD_DRV / 2 + 1.6
    c2, r2 = (-MXx, MXy), PD_MOT / 2 + 1.6
    dx, dy = c1[0] - c2[0], c1[1] - c2[1]
    D = math.hypot(dx, dy)
    alpha = math.acos(-(r1 - r2) / D)
    phi = math.atan2(dy, dx)
    n = max(((math.cos(phi + s * alpha), math.sin(phi + s * alpha))
             for s in (1, -1)), key=lambda v: v[0])   # inboard-pointing normal
    cline = n[0] * c1[0] + n[1] * c1[1] + r1
    m = min(n[0] * px + n[1] * py - cline for px, py in wpts)
    ok(m >= 1.5, m - 1.5, "frame corner clears the OPPOSITE belt strand")

    # -- deck containment (every underside part, 1.5 mm inside the outline) --
    pts = {}
    for a, b in fp:
        pts[f"frame({a:.0f},{b:.0f})"] = (a, b)
    pts["flangeO-rear"] = (-FR_A, FLG_B); pts["flangeO-frnt"] = (FIN_A_OUT, FLG_B)
    pts["flangeI-rear"] = (-FR_A, -FLG_B); pts["flangeI-frnt"] = (FIN_A_INN, -FLG_B)
    for cx in (-1, 1):
        for cy in (-1, 1):
            pts[f"carriage({cx},{cy})"] = (cx * (CAR_L / 2 + TENSION), cy * CAR_W / 2)
            pts[f"motor({cx},{cy})"] = (cx * 46.0, cy * 40.0)
    pts["block-out"] = (BLK_A1, BLK_B); pts["block-in"] = (BLK_A1, -BLK_B)
    worst, worst_k = 1e9, ""
    for k, (a, b) in pts.items():
        for sgn in (1, -1):
            d = _deck_margin(local_to_world(sgn, a, b))
            if d < worst:
                worst, worst_k = d, f"{k} side {sgn}"
    ok(worst >= 1.5, worst - 1.5, f"deck contains all hardware (worst: {worst_k})")
    # tool-access slots: inside the outline, clear of every circular deck hole
    holes = deck_hole_table()
    hl, hw = ACC_L / 2, ACC_W / 2
    worst_edge, worst_hole, worst_pair = 1e9, 1e9, ""
    for hid, sx_, sy_, ang in deck_access_slots():
        th = math.radians(ang)
        ux, uy = math.cos(th), math.sin(th)
        vx, vy = -math.sin(th), math.cos(th)
        for i in (-1, 1):
            for j in (-1, 1):
                worst_edge = min(worst_edge, _deck_margin(
                    (sx_ + i * hl * ux + j * hw * vx,
                     sy_ + i * hl * uy + j * hw * vy)))
        for hid2, hx_, hy_, d, _n in holes:
            px, py = hx_ - sx_, hy_ - sy_
            t = max(-(hl - hw), min(hl - hw, px * ux + py * uy))
            dd = math.hypot(px - t * ux, py - t * uy) - hw - d / 2
            if dd < worst_hole:
                worst_hole, worst_pair = dd, f"{hid}-{hid2}"
    ok(worst_edge >= 1.5, worst_edge - 1.5, "deck access slots inside the outline")
    ok(worst_hole >= 2.0, worst_hole - 2.0,
       f"deck access slots clear of deck holes (worst: {worst_pair})")

    xs = [p[0] for p in DECK_PTS]; ys = [p[1] for p in DECK_PTS]
    m = BED - (max(xs) - min(xs))
    ok(m >= 0, m, "deck width fits 300 bed")
    m = BED - (max(ys) - min(ys))
    ok(m >= 0, m, "deck depth fits 300 bed")

    # -- arm stack (unchanged geometry, still guarded) -----------------------
    ok(B0 - A1 >= 4.0, B0 - A1 - 4.0, "arm plane gap")
    m = A0 - (LOCKNUT_Z + LOCKNUT_T)
    ok(m >= 2.0, m - 2.0, "arm boss clears shaft locknut")
    ok(SHAFT_LEN >= B1 + 4.0, SHAFT_LEN - B1 - 4.0, "shaft past upper arm clamp")

    if verbose:
        for s in notes:
            print(" ", s)
    return notes


if __name__ == "__main__":
    print(f"24T/72T HTD-5M  C={C:.2f}  motor=({MXx:.2f},{MXy:.2f})  "
          f"keyway wall {PULLEY_KEY_WALL:.2f}")
    check_layout(verbose=True)
    print("layout check: all clear")
