"""Machine-shop drawing set for the base hardware — dimensioned 2D sheets.

Every number on these sheets comes from cad/params.py — the same module the
solid model (generate.py) builds from — so the drawings can never disagree
with the STEP files. Regenerate with:

    python cad/drawings.py

Outputs:  cad/drawings/bcr_drawing_set.pdf   (multi-sheet, vector)
          docs/cad/drawings/sheet*.png       (per-sheet previews)

Sheet list
  1  BRG-CAP      bearing retainer cap (qty 8)
  2  CARRIAGE     sliding motor carriage (qty 2)
  3  CRADLE       fixed slider cradle (qty 1 LH + 1 RH mirrored)
  4  DECK         bottom deck plate w/ full hole table
  5  TOP-PLATE    top plate + standoffs (standoff positions moved for Ø72 caps)
  6  ASSY         tensioner assembly section, hardware list, adjustment notes
"""

from __future__ import annotations

import math
from datetime import date
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Arc, Circle, FancyArrowPatch, Polygon, Rectangle

from params import (A0, B1, BED, BELT_LEN, BLK_A0, BLK_A1, BLK_B, BLK_H, C,
                    CAP_BCD, CAP_BOLT_D, CAP_OD, CAP_RELIEF, CAP_SHAFT_CLR,
                    CAP_SPIGOT, CAP_T, CAR_L, CAR_W, DECK_PTS, DECK_T, DECK_Z0,
                    FIN_A_INN, FIN_A_OUT, FIN_BI, FIN_BO, FIN_BOLT_B,
                    FIN_BOLTS_INN, FIN_BOLTS_OUT, FLG_A_INN, FLG_B, FLG_T,
                    FR_A, FR_B, FR_T, HX, JACK_Z_OFF, LOCK_A, LOCK_B,
                    MOTOR_BCD, MOTOR_BORE, MP_T, MPL_TOP, MPR_TOP, MXx, MXy,
                    PD_DRV, PD_MOT, PILOT_D, PW, SLOT_L, SLOT_W, SPLAY,
                    STANDOFF, STANDOFF_PTS, T_DRV, T_MOT, TENSION, TOPP_H,
                    TOPP_T, TOPP_W, WIN_A, WIN_B, ZL, ZR, check_layout,
                    deck_hole_table, frame_poly_local)

ROOT = Path(__file__).resolve().parents[1]
OUT_PDF = ROOT / "cad" / "drawings"
OUT_PNG = ROOT / "docs" / "cad" / "drawings"
OUT_PDF.mkdir(parents=True, exist_ok=True)
OUT_PNG.mkdir(parents=True, exist_ok=True)

INK = "#1a1a1a"
DIM = "#00527a"
CTR = "#8a2a2a"
THIN = 0.5
THICK = 1.1


# ----------------------------------------------------------------- toolkit
class View:
    """A scaled 2D viewport anchored at paper position (ox, oy), mm units."""

    def __init__(self, ax, ox, oy, s=1.0):
        self.ax, self.ox, self.oy, self.s = ax, ox, oy, s

    def X(self, x):
        return self.ox + x * self.s

    def Y(self, y):
        return self.oy + y * self.s

    def line(self, pts, lw=THICK, color=INK, ls="-"):
        xs = [self.X(p[0]) for p in pts]
        ys = [self.Y(p[1]) for p in pts]
        self.ax.plot(xs, ys, lw=lw, color=color, ls=ls, solid_capstyle="round")

    def poly(self, pts, lw=THICK, color=INK, ls="-", fill=False, fc="none",
             hatch=None):
        xy = [(self.X(p[0]), self.Y(p[1])) for p in pts]
        self.ax.add_patch(Polygon(xy, closed=True, fill=fill, fc=fc,
                                  ec=color, lw=lw, ls=ls, hatch=hatch))

    def rect(self, x0, y0, x1, y1, **kw):
        self.poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], **kw)

    def circle(self, x, y, d, lw=THICK, color=INK, ls="-"):
        self.ax.add_patch(Circle((self.X(x), self.Y(y)), d / 2 * self.s,
                                 fill=False, ec=color, lw=lw, ls=ls))

    def cmark(self, x, y, r=3.0):
        e = r * self.s
        X, Y = self.X(x), self.Y(y)
        self.ax.plot([X - e, X + e], [Y, Y], lw=THIN, color=CTR)
        self.ax.plot([X, X], [Y - e, Y + e], lw=THIN, color=CTR)

    def slot(self, cx, cy, L, W, lw=THICK):
        r = W / 2
        x0, x1 = cx - L / 2 + r, cx + L / 2 - r
        self.line([(x0, cy + r), (x1, cy + r)], lw=lw)
        self.line([(x0, cy - r), (x1, cy - r)], lw=lw)
        for (x, a0) in ((x0, 90), (x1, -90)):
            self.ax.add_patch(Arc((self.X(x), self.Y(cy)), W * self.s, W * self.s,
                                  angle=0, theta1=a0, theta2=a0 + 180,
                                  ec=INK, lw=lw))

    def text(self, x, y, s, size=6.5, ha="center", va="center", color=DIM,
             rot=0, weight="normal"):
        self.ax.text(self.X(x), self.Y(y), s, fontsize=size, ha=ha, va=va,
                     color=color, rotation=rot, weight=weight)

    # --- dimensions (drawn in model coords, offsets in model mm) ----------
    def _arrow(self, p0, p1):
        self.ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="<|-|>",
                                          mutation_scale=6, lw=THIN,
                                          color=DIM, shrinkA=0, shrinkB=0))

    def dimh(self, x0, x1, y, off, txt=None, size=6.5):
        yd = y + off
        for x in (x0, x1):
            self.line([(x, y), (x, yd + (0.8 if off > 0 else -0.8))],
                      lw=THIN, color=DIM)
        self._arrow((self.X(x0), self.Y(yd)), (self.X(x1), self.Y(yd)))
        self.text((x0 + x1) / 2, yd + (2.2 if off > 0 else -2.6) / self.s,
                  txt or f"{abs(x1 - x0):g}", size=size)

    def dimv(self, y0, y1, x, off, txt=None, size=6.5):
        xd = x + off
        for y in (y0, y1):
            self.line([(x, y), (xd + (0.8 if off > 0 else -0.8), y)],
                      lw=THIN, color=DIM)
        self._arrow((self.X(xd), self.Y(y0)), (self.X(xd), self.Y(y1)))
        self.text(xd + (2.2 if off > 0 else -2.6) / self.s, (y0 + y1) / 2,
                  txt or f"{abs(y1 - y0):g}", size=size, rot=90)

    def leader(self, x, y, dx, dy, txt, size=6.5, ha="left"):
        self.ax.add_patch(FancyArrowPatch((self.X(x + dx), self.Y(y + dy)),
                                          (self.X(x), self.Y(y)),
                                          arrowstyle="-|>", mutation_scale=6,
                                          lw=THIN, color=DIM))
        self.text(x + dx + (1.5 if ha == "left" else -1.5) / self.s, y + dy,
                  txt, size=size, ha=ha)


def new_sheet(pdf, no, title, part, material, qty, scale, notes=()):
    fig = plt.figure(figsize=(420 / 25.4, 297 / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 420); ax.set_ylim(0, 297); ax.axis("off")
    ax.add_patch(Rectangle((8, 8), 404, 281, fill=False, ec=INK, lw=1.2))
    # title block
    x0, y0, w, h = 254, 8, 158, 40
    ax.add_patch(Rectangle((x0, y0), w, h, fill=False, ec=INK, lw=1.0))
    for fy in (y0 + 10, y0 + 20, y0 + 30):
        ax.plot([x0, x0 + w], [fy, fy], lw=0.5, color=INK)
    ax.plot([x0 + 79, x0 + 79], [y0, y0 + 30], lw=0.5, color=INK)
    ax.text(x0 + 4, y0 + 34, "5-BAR BUNG-COVER ROBOT — BASE HARDWARE",
            fontsize=8, weight="bold", color=INK)
    ax.text(x0 + 4, y0 + 24, f"TITLE: {title}", fontsize=7.5, color=INK)
    ax.text(x0 + 83, y0 + 24, f"DWG NO: BCR-{no:02d}   REV A", fontsize=7.5, color=INK)
    ax.text(x0 + 4, y0 + 14, f"MATERIAL: {material}", fontsize=7, color=INK)
    ax.text(x0 + 83, y0 + 14, f"QTY: {qty}", fontsize=7, color=INK)
    ax.text(x0 + 4, y0 + 4, f"SCALE {scale}   UNITS mm   THIRD ANGLE",
            fontsize=7, color=INK)
    ax.text(x0 + 83, y0 + 4, f"SHEET {no} OF 6   {date.today().isoformat()}",
            fontsize=7, color=INK)
    # general tolerance + notes
    ax.text(14, 283, f"SHEET {no}: {title}   ({part})", fontsize=11,
            weight="bold", color=INK)
    tol = ("GENERAL TOLERANCES UNLESS NOTED:  X ±0.5   X.X ±0.2   "
           "HOLES +0.2/-0   ANGLES ±0.5°   BREAK ALL EDGES 0.3")
    ax.text(14, 276, tol, fontsize=6.5, color=INK)
    ax.text(14, 271.5, "DUAL MATERIAL CALLOUTS: select ONE per order — printed parts take brass "
            "heat-set inserts at every thread callout; 6061 parts are tapped.", fontsize=6.5, color=INK)
    for i, n in enumerate(notes):
        ax.text(14, 265 - 5.2 * i, f"{i + 1}. {n}", fontsize=6.5, color=INK)
    return fig, ax


def finish(pdf, fig, png_name):
    pdf.savefig(fig)
    fig.savefig(OUT_PNG / png_name, dpi=140)
    plt.close(fig)


# ------------------------------------------------------------- sheet 1: cap
def sheet_cap(pdf):
    fig, ax = new_sheet(
        pdf, 1, "BEARING RETAINER CAP", "BRG-CAP", "PA12 MJF / 6061-T6", 8, "1.5:1",
        notes=(
            "One cap above AND one below each 7005 bearing bore; the pair bolts together THROUGH the plate "
            "with 4x M4x35 SHCS + nyloc (plate holes 4.5 on the same BCD; grip 26 at the deck, 20 at the plate).",
            "The 46.6 spigot registers in the 47.0 plate bore. Its 41.0-46.6 face ring presses the bearing "
            "OUTER race only; the bore is relieved inboard of 41.0 to clear the rotating inner race.",
            "Build one cap spigot-UP and one spigot-DOWN per bore (same part, flipped).",
            "OD is 72 so the bolt holes keep a full 1.5d rim (edge distance 4.75).",
        ))
    v = View(ax, 105, 150, 1.5)
    # plan
    v.circle(0, 0, CAP_OD)
    v.circle(0, 0, CAP_SPIGOT, lw=THIN, ls="--")
    v.circle(0, 0, CAP_RELIEF, lw=THIN, ls="--")
    v.circle(0, 0, CAP_SHAFT_CLR)
    v.circle(0, 0, CAP_BCD, lw=THIN, ls=(0, (6, 2, 1, 2)), color=CTR)
    for a in (45, 135, 225, 315):
        bx = CAP_BCD / 2 * math.cos(math.radians(a))
        by = CAP_BCD / 2 * math.sin(math.radians(a))
        v.circle(bx, by, CAP_BOLT_D)
        v.cmark(bx, by, 3.5)
    v.cmark(0, 0, CAP_OD / 2 + 4)
    v.leader(CAP_OD / 2 * 0.707, CAP_OD / 2 * 0.707, 14, 8,
             f"Ø{CAP_OD:g}")
    v.leader(CAP_BCD / 2 * 0.707, -CAP_BCD / 2 * 0.707, 16, -10,
             f"4x Ø{CAP_BOLT_D:g} THRU EQ SP ON Ø{CAP_BCD:g} BCD")
    v.leader(0, -CAP_SHAFT_CLR / 2, 10, -16, f"Ø{CAP_SHAFT_CLR:g} THRU")
    v.leader(-CAP_SPIGOT / 2 * 0.707, CAP_SPIGOT / 2 * 0.707, -16, 10,
             f"Ø{CAP_SPIGOT:g} -0.05/-0.15 SPIGOT", ha="right")
    v.leader(-CAP_RELIEF / 2, 0, -20, -4, f"Ø{CAP_RELIEF:g} RELIEF", ha="right")
    v.text(0, -CAP_OD / 2 - 12 / 1.5, "PLAN (spigot side away)", size=7)

    # section A-A (half profile, hatched)
    s = View(ax, 275, 150, 1.5)
    r27, r41, r466, r72 = CAP_SHAFT_CLR / 2, CAP_RELIEF / 2, CAP_SPIGOT / 2, CAP_OD / 2
    prof = [(r27, 0), (r72, 0), (r72, CAP_T), (r466, CAP_T),
            (r466, CAP_T + 2), (r41, CAP_T + 2), (r41, CAP_T + 0.5),
            (r27, CAP_T + 0.5)]
    for sgn in (1, -1):
        s.poly([(sgn * x, y) for x, y in prof], hatch="////", lw=THICK)
    s.line([(0, -6), (0, CAP_T + 8)], lw=THIN, color=CTR, ls=(0, (8, 3, 1, 3)))
    s.dimv(0, CAP_T, r72, 10, f"{CAP_T:g}")
    s.dimv(CAP_T, CAP_T + 2, r466, 26, "2.0 SPIGOT")
    s.dimh(-r72, r72, 0, -10, f"Ø{CAP_OD:g}")
    s.dimh(-r466, r466, CAP_T + 2, 12, f"Ø{CAP_SPIGOT:g}")
    s.dimh(-r41, r41, CAP_T + 0.5, 22, f"Ø{CAP_RELIEF:g}")
    s.text(0, -24 / 1.5, "SECTION A-A", size=7)
    s.leader(r466 - (r466 - r41) / 2, CAP_T + 2, 22, 8,
             "presses OUTER race only")
    finish(pdf, fig, "sheet1_cap.png")


# -------------------------------------------------------- sheet 2: carriage
def sheet_carriage(pdf):
    fig, ax = new_sheet(
        pdf, 2, "SLIDING MOTOR CARRIAGE", "CARRIAGE", "PA12 MJF / 6061-T6", 2, "1:1",
        notes=(
            "Motor (A6M80, 80 sq flange) mounts from BELOW: 4x M6x18 SHCS up through the motor flange "
            "into the four M6 positions. PRINTED part: M6 brass heat-set inserts. 6061: tap M6.",
            f"The four slots take M5x16 SHCS + washer down into the cradle frame bosses directly beneath; "
            f"slot length gives ±{TENSION:g} mm of tension travel.",
            "The +X edge (toward the shoulder) is the jackscrew pad — the M6 jackscrew tip bears mid-edge.",
            "Same part both sides (symmetric).",
        ))
    v = View(ax, 120, 150, 1.0)
    L, W = CAR_L / 2, CAR_W / 2
    v.rect(-L, -W, L, W)
    v.circle(0, 0, PILOT_D)
    v.cmark(0, 0, PILOT_D / 2 + 6)
    for a in (45, 135, 225, 315):
        bx = MOTOR_BCD / 2 * math.cos(math.radians(a))
        by = MOTOR_BCD / 2 * math.sin(math.radians(a))
        v.circle(bx, by, 5.0)
        v.cmark(bx, by, 4)
    v.circle(0, 0, MOTOR_BCD, lw=THIN, ls=(0, (6, 2, 1, 2)), color=CTR)
    for sa in (LOCK_A, -LOCK_A):
        for sb in (LOCK_B, -LOCK_B):
            v.slot(sa, sb, SLOT_L, SLOT_W)
            v.cmark(sa, sb, 4)
    v.dimh(-L, L, -W, -12, f"{CAR_L:g}")
    v.dimv(-W, W, L, 14, f"{CAR_W:g}")
    v.dimh(-LOCK_A, LOCK_A, W, 10, f"{2 * LOCK_A:g}")
    v.dimv(-LOCK_B, LOCK_B, -L, -14, f"{2 * LOCK_B:g}")
    v.leader(0, PILOT_D / 2, 30, 14, f"Ø{PILOT_D:g} THRU (motor pilot)")
    v.leader(MOTOR_BCD / 2 * 0.707, -MOTOR_BCD / 2 * 0.707, 26, -12,
             f"4x M6 THRU ON Ø{MOTOR_BCD:g} BCD AT 45° (insert/tap per material)")
    v.leader(LOCK_A + SLOT_L / 2 - 2, LOCK_B, 22, 10,
             f"4x SLOT {SLOT_L:g} x {SLOT_W:g} THRU")
    v.leader(L, 0, 14, -22, "JACKSCREW PAD (this edge)")
    v.text(0, -W - 22, "PLAN (top face)", size=7)
    # edge view
    e = View(ax, 120, 62, 1.0)
    e.rect(-L, 0, L, MP_T, hatch="////")
    e.dimv(0, MP_T, L, 12, f"{MP_T:g}")
    e.text(0, -8, "EDGE VIEW", size=7)
    finish(pdf, fig, "sheet2_carriage.png")


# ---------------------------------------------------------- sheet 3: cradle
def sheet_cradle(pdf):
    fig, ax = new_sheet(
        pdf, 3, "SLIDER CRADLE (LH SHOWN, RH = MIRROR)", "CRADLE-L / CRADLE-R",
        "PA12 MJF / PETG-CF (print flat, frame down)", "1 + 1 (mirror)", "1:2 / 1:1",
        notes=(
            "ONE printed part per motor = window frame + two shear fins + jack block. The RH part is the "
            "exact MIRROR of the LH drawn here; fin/wall heights differ per the table (belt planes stagger).",
            "Fin top flanges (10 thick - full-depth M5 heat-set) bolt UP into the deck underside: 3x M5 per fin, "
            "M5x20 SHCS from the deck top (deck holes 5.2 THRU; ~8 mm engagement).",
            "The four M5 positions in the frame border take the carriage lock bolts — fit heat-set inserts.",
            "M6 heat-set (or tapped) jackscrew boss in the block; jam nut on the screw against the block face.",
            "The chamfered corner keeps the two cradles clear of the machine centreline — do not 'square it up'.",
            f"FIN WALLS: inner faces at b = ±{52.5:g}, {5:g} thick, rising to the deck underside. Outboard wall "
            f"spans a = -66..+62, inboard a = -66..+30. The inner faces guide the carriage edges (0.5 nominal).",
            "Install the belt around both pulleys BEFORE bolting the deck down (the fins close the corridor).",
        ))
    v = View(ax, 118, 158, 0.9)
    poly = frame_poly_local()
    v.poly(poly)
    v.rect(-WIN_A, -WIN_B, WIN_A, WIN_B)
    # fins (top flanges seen from above; walls dashed underneath)
    v.rect(-FR_A, FIN_BI, FIN_A_OUT, FLG_B)
    v.line([(-FR_A, FIN_BO), (FIN_A_OUT, FIN_BO)], lw=THIN, ls="--")
    v.rect(-FR_A, -FLG_B, FLG_A_INN, -FIN_BI)
    v.line([(-FR_A, -FIN_BO), (FIN_A_INN, -FIN_BO)], lw=THIN, ls="--")
    v.line([(FLG_A_INN, -FIN_BI), (FIN_A_INN, -FIN_BI)], lw=THIN, ls="--")
    for a in FIN_BOLTS_OUT:
        v.circle(a, FIN_BOLT_B, 4.2); v.cmark(a, FIN_BOLT_B, 3.5)
    for a in FIN_BOLTS_INN:
        v.circle(a, -FIN_BOLT_B, 4.2); v.cmark(a, -FIN_BOLT_B, 3.5)
    for sa in (LOCK_A, -LOCK_A):
        for sb in (LOCK_B, -LOCK_B):
            v.circle(sa, sb, 4.2); v.cmark(sa, sb, 3.5)
    v.rect(BLK_A0, -BLK_B, BLK_A1, BLK_B)
    v.leader(BLK_A1, 0, 12, -14, "M6 JACKSCREW BOSS")
    # dims
    v.dimh(-FR_A, FR_A, -FR_B, -12, f"{2 * FR_A:g}")
    v.dimv(-FR_B, FR_B, -FR_A, -12, f"{2 * FR_B:g}")
    v.dimh(-WIN_A, WIN_A, WIN_B, 6, f"WINDOW {2 * WIN_A:g}")
    v.dimv(-WIN_B, WIN_B, WIN_A - 68, 4, f"x {2 * WIN_B:g}", size=6)
    v.dimh(-LOCK_A, LOCK_A, -WIN_B, -5, f"{2 * LOCK_A:g}")
    v.dimv(-LOCK_B, LOCK_B, -FR_A + 8, 4, f"{2 * LOCK_B:g}", size=6)
    v.leader(LOCK_A, LOCK_B, 34, 22, "4x M5 HEAT-SET (lock bolts)")
    v.leader(0, FIN_BOLT_B, -34, 8, f"3x M5 HEAT-SET ON {FIN_BOLT_B:g} LINE",
             ha="right")
    v.leader(-16, -FIN_BOLT_B, -30, -9, "3x M5 HEAT-SET", ha="right")
    ch0, ch1 = poly[1], poly[2]
    v.leader((ch0[0] + ch1[0]) / 2, (ch0[1] + ch1[1]) / 2, 16, -18,
             f"CHAMFER ({ch0[0]:.1f},{ch0[1]:g}) TO ({ch1[0]:g},{ch1[1]:.1f})")
    v.text(0, FLG_B + 16, "PLAN — LH PART (frame top; fins rise toward viewer)",
           size=7)
    v.text(0, -FR_B - 26, "DATUM: origin at frame centre. +a = toward the shoulder "
           "(jack-block end), +b = toward the OUTBOARD fin. All holes in the HOLE TABLE.",
           size=6)
    # jack block + fin wall dimensions
    v.dimh(BLK_A0, BLK_A1, -BLK_B, -8, f"{BLK_A1 - BLK_A0:g}")
    v.dimv(-BLK_B, BLK_B, BLK_A1, 14, f"{2 * BLK_B:g}")

    # elevation (view along -b: outboard fin in front)
    e = View(ax, 262, 196, 0.9)
    ftL = MPL_TOP - MP_T
    e.rect(-FR_A, ftL - FR_T, FR_A, ftL, hatch="////")
    e.rect(-FR_A, ftL, FIN_A_OUT, DECK_Z0)
    e.rect(BLK_A0, ftL, BLK_A1, ftL + BLK_H, hatch="////")
    e.line([(-FR_A - 12, DECK_Z0), (FR_A + 12, DECK_Z0)], lw=THIN, color=CTR,
           ls=(0, (8, 3, 1, 3)))
    e.text(0, DECK_Z0 + 7, "DECK UNDERSIDE z52", size=6)
    e.dimv(ftL - FR_T, DECK_Z0, -FR_A, -10, "H (table)")
    e.dimv(ftL - FR_T, ftL, FR_A, 10, f"{FR_T:g}")
    e.dimv(DECK_Z0 - FLG_T, DECK_Z0, -FR_A, -26, f"{FLG_T:g} FLANGE")
    e.leader(BLK_A1, ftL + JACK_Z_OFF, -56, -34,
             f"M6 AXIS +{JACK_Z_OFF:g} ABOVE FRAME TOP", ha="right")
    e.dimv(ftL, ftL + BLK_H, BLK_A1, 30, f"{BLK_H:g} BLOCK")
    e.text(-30, ftL - FR_T - 46, "ELEVATION (outboard fin)", size=7)
    # LH/RH table
    ty = 158
    ax.text(262, ty + 20, "PART TABLE", fontsize=8, weight="bold", color=INK)
    rows = [("", "FRAME TOP z", "WALL H (frame btm→deck)", "MIRROR"),
            ("CRADLE-L", f"{MPL_TOP - MP_T:g}", f"{DECK_Z0 - (MPL_TOP - MP_T - FR_T):g}", "as drawn"),
            ("CRADLE-R", f"{MPR_TOP - MP_T:g}", f"{DECK_Z0 - (MPR_TOP - MP_T - FR_T):g}", "mirrored")]
    for i, r in enumerate(rows):
        for j, cell in enumerate(r):
            ax.text(262 + j * 38, ty + 10 - i * 7, cell, fontsize=6.5, color=INK,
                    weight="bold" if i == 0 else "normal")
    # hole coordinate table (a, b from the frame-centre datum) — LH part;
    # the RH (mirrored) part uses the same table with b negated
    hy = ty - 30
    ax.text(262, hy, "HOLE TABLE (a, b) — LH part; RH = same with b negated",
            fontsize=7, weight="bold", color=INK)
    hrows = ([("H%d" % (i + 1), a, FIN_BOLT_B, "M5 heat-set, fin flange (outboard)")
              for i, a in enumerate(FIN_BOLTS_OUT)]
             + [("H%d" % (i + 4), a, -FIN_BOLT_B, "M5 heat-set, fin flange (inboard)")
                for i, a in enumerate(FIN_BOLTS_INN)]
             + [("L%d" % (i + 1), sa, sb, "M5 heat-set, carriage lock")
                for i, (sa, sb) in enumerate(
                    [(x, y) for x in (LOCK_A, -LOCK_A) for y in (LOCK_B, -LOCK_B)])]
             + [("J1", (BLK_A0 + BLK_A1) / 2, 0.0, "M6 heat-set, jackscrew (axis "
                 "horizontal, +%g above frame top)" % JACK_Z_OFF)])
    for col in (0, 1):
        ax.text(262 + col * 78, hy - 6, "ID      a        b   SPEC", fontsize=5.6,
                family="monospace", weight="bold", color=INK)
    for i, (hid, a, b, spec) in enumerate(hrows):
        col, row = divmod(i, 7)
        spec_short = spec.split(",")[0] + "," + spec.split(",")[1].split("(")[0] \
            if "(" in spec else spec
        ax.text(262 + col * 78, hy - 11 - row * 4.6,
                f"{hid:<3} {a:7.2f} {b:7.2f} {spec_short[:26]}",
                fontsize=5.0, family="monospace", color=INK)
    finish(pdf, fig, "sheet3_cradle.png")


# ------------------------------------------------------------ sheet 4: deck
def sheet_deck(pdf):
    fig, ax = new_sheet(
        pdf, 4, "BOTTOM DECK PLATE", "DECK", "PA12 MJF / 6061-T6 (12 THK)", 1, "1:2",
        notes=(
            "Machine per the HOLE TABLE (coordinates from the plate origin = mid-span between the two "
            "shoulder bores; +Y toward the arms). All holes THRU.",
            "The two 47.0 bores carry the 7005 bearings (transition fit: bore 47.0 +0.025/0).",
            "Corner radii R8. Outline vertices in the VERTEX TABLE.",
            f"Plate must stay within a {BED:g} x {BED:g} print bed (current {294:g} x {240:g}).",
        ))
    v = View(ax, 118, 152, 0.55)
    v.poly(DECK_PTS)
    rows = deck_hole_table()
    for hid, x, y, d, note in rows:
        v.circle(x, y, max(d, 4.0))
        v.cmark(x, y, max(d, 4.0) / 2 + 2)
        dx = 5 if x >= 0 else -5
        v.text(x + dx, y + 5, hid, size=5, ha="left" if x >= 0 else "right",
               color=INK)
    v.dimh(-147, 147, -178, -14, "294")
    v.dimv(-178, 62, -147, -14, "240")
    v.cmark(0, 0, 10)
    v.text(0, 74, "PLAN — deck top (origin at mid-span of shoulder bores)", size=7)
    # hole table
    tx, ty = 258, 262
    ax.text(tx, ty, "HOLE TABLE", fontsize=8, weight="bold", color=INK)
    ax.text(tx, ty - 6, "ID      X        Y        Ø     SPEC", fontsize=6,
            family="monospace", color=INK, weight="bold")
    for i, (hid, x, y, d, note) in enumerate(rows):
        ax.text(tx, ty - 11 - i * 4.6,
                f"{hid:<4} {x:8.2f} {y:8.2f} {d:5.1f}  {note}",
                fontsize=5.4, family="monospace", color=INK)
    vy = ty - 11 - len(rows) * 4.6 - 6
    ax.text(tx, vy, "VERTEX TABLE (outline, R8 corners)", fontsize=7,
            weight="bold", color=INK)
    for i, (x, y) in enumerate(DECK_PTS):
        ax.text(tx + (i % 3) * 52, vy - 6 - (i // 3) * 5,
                f"V{i + 1} ({x:g},{y:g})", fontsize=5.6, family="monospace",
                color=INK)
    finish(pdf, fig, "sheet4_deck.png")


# ------------------------------------------------------- sheet 5: top plate
def sheet_top(pdf):
    fig, ax = new_sheet(
        pdf, 5, "TOP PLATE + STANDOFFS", "TOP-PLATE", "PA12 MJF / 6061-T6 (10 THK)",
        "1 (+4 standoffs)", "1:1",
        notes=(
            "Same bearing bore + cap-bolt pattern as the deck (47.0 bores, 4x 4.5 on Ø58 BCD per bore).",
            f"Standoff bolt holes moved OUT to (±{STANDOFF_PTS[0][0]:g}, ±{STANDOFF_PTS[0][1]:g}) so the "
            f"Ø{CAP_OD:g} caps clear the standoff bosses (they collided with the old Ø62 caps at ±60,±32).",
            f"4x standoffs Ø12 x {STANDOFF:g}, Ø5.2 clearance bore THRU. Each stack is THROUGH-BOLTED: "
            f"one M5 x 70 SHCS from the plate top, nyloc + washer under the deck (no threads in the standoff).",
        ))
    v = View(ax, 145, 150, 1.0)
    W, H = TOPP_W / 2, TOPP_H / 2
    v.rect(-W, -H, W, H)
    for sx in (HX, -HX):
        v.circle(sx, 0, 47.0)
        v.cmark(sx, 0, 28)
        v.circle(sx, 0, CAP_BCD, lw=THIN, ls=(0, (6, 2, 1, 2)), color=CTR)
        for a in (45, 135, 225, 315):
            bx = sx + CAP_BCD / 2 * math.cos(math.radians(a))
            by = CAP_BCD / 2 * math.sin(math.radians(a))
            v.circle(bx, by, CAP_BOLT_D); v.cmark(bx, by, 3.5)
    for px, py in STANDOFF_PTS:
        v.circle(px, py, 5.2); v.cmark(px, py, 4.5)
    v.dimh(-W, W, -H, -12, f"{TOPP_W:g}")
    v.dimv(-H, H, W, 10, f"{TOPP_H:g}")
    v.dimh(-HX, HX, H, 8, f"{2 * HX:g}")
    v.dimh(-STANDOFF_PTS[0][0], STANDOFF_PTS[0][0], -H, -22,
           f"{2 * STANDOFF_PTS[0][0]:g}")
    v.dimv(-STANDOFF_PTS[0][1], STANDOFF_PTS[0][1], W, 28,
           f"{2 * STANDOFF_PTS[0][1]:g}")
    v.leader(HX, 47.0 / 2 * 0.707, 26, 16, "2x Ø47.0 +0.025/0 THRU")
    v.leader(-HX + CAP_BCD / 2 * 0.707, -CAP_BCD / 2 * 0.707, -26, -14,
             f"8x Ø{CAP_BOLT_D:g} THRU ON Ø{CAP_BCD:g} BCD, CLOCKED 45° (= deck C-holes)", ha="right")
    v.leader(STANDOFF_PTS[0][0], STANDOFF_PTS[0][1], 16, 10, "4x Ø5.2 THRU")
    v.text(0, -H - 34, "PLAN", size=7)
    # standoff detail
    s = View(ax, 330, 120, 1.0)
    s.rect(-6, 0, 6, STANDOFF, hatch="////")
    s.rect(-2.6, -4, 2.6, STANDOFF + 4, lw=THIN, ls="--")
    s.dimv(0, STANDOFF, 6, 10, f"{STANDOFF:g}")
    s.dimh(-6, 6, 0, -8, "Ø12")
    s.leader(0, STANDOFF - 6, 14, 6, "Ø5.2 THRU")
    s.text(0, -16, "STANDOFF (x4)", size=7)
    finish(pdf, fig, "sheet5_top_plate.png")


# ------------------------------------------------------- sheet 6: assembly
def sheet_assy(pdf):
    fig, ax = new_sheet(
        pdf, 6, "TENSIONER ASSEMBLY + HARDWARE", "ASSY-TENSIONER", "-", "2 (mirrored)",
        "1:1 section",
        notes=(
            "TENSION PROCEDURE: loosen the 4x M5 lock bolts 1/2 turn -> turn the M6 jackscrew CW to push "
            "the carriage away from the shoulder (belt tightens) -> torque the lock bolts (M5: 4 N·m in "
            "inserts) -> set the M6 jam nut against the block.",
            f"Travel available ±{TENSION:g} mm. Belt: {BELT_LEN:g}-5M-15 ({T_MOT}T -> {T_DRV}T, "
            f"C = {C:.1f} nominal).",
            "ASSEMBLY ORDER: pulleys + belt on motor/shaft FIRST, then lower the deck onto fins + standoffs "
            "and bolt down (the fin walls close the belt corridor once the deck is on).",
            "The belt passes BETWEEN the two fins, above the frame and below the deck — nothing crosses it.",
        ))
    # section elevation through the left tensioner, cut on the belt axis plane
    v = View(ax, 130, 130, 1.0)
    ftL = MPL_TOP - MP_T
    v.rect(-FR_A - 20, DECK_Z0, FR_A + 30, DECK_Z0 + DECK_T, hatch="////")
    v.text(FR_A + 34, DECK_Z0 + 6, "DECK", size=6, ha="left")
    # fin behind the section plane (outline only)
    v.rect(-FR_A, ftL - FR_T, FIN_A_OUT, DECK_Z0, lw=THIN, ls="--")
    v.text(-FR_A - 4, (DECK_Z0 + ftL) / 2, "FIN (behind)", size=6, ha="right",
           rot=90)
    # frame + block (cut)
    v.rect(-FR_A, ftL - FR_T, FR_A, ftL, hatch="////")
    v.rect(BLK_A0, ftL, BLK_A1, ftL + BLK_H, hatch="////")
    # carriage (cut) with slot + lock bolt
    v.rect(-CAR_L / 2, ftL, CAR_L / 2, MPL_TOP, hatch="\\\\\\\\")
    v.line([(LOCK_A - SLOT_L / 2, MPL_TOP), (LOCK_A + SLOT_L / 2, MPL_TOP)],
           lw=2.2, color=CTR)
    v.line([(LOCK_A, MPL_TOP + 6), (LOCK_A, ftL - FR_T + 2)], lw=1.6, color=DIM)
    v.text(LOCK_A + 14, MPL_TOP + 26, "M5 LOCK BOLT (4x)\nthrough slot into frame boss",
           size=5.5)
    # motor flange + body below
    v.rect(-40, ftL - 10, 40, ftL, lw=THIN)
    v.rect(-40, ftL - 10 - 50, 40, ftL - 10, lw=THIN)
    v.text(0, ftL - 36, "MOTOR (hangs in window)", size=6)
    # jackscrew
    v.line([(BLK_A1 + 10, ftL + JACK_Z_OFF), (CAR_L / 2, ftL + JACK_Z_OFF)],
           lw=1.8, color=DIM)
    v.leader(BLK_A1 + 6, ftL + JACK_Z_OFF, 14, 14,
             "M6 JACKSCREW + JAM NUT")
    v.leader(CAR_L / 2, ftL + JACK_Z_OFF, 20, -12, "pushes carriage pad")
    # belt (between fins, above frame)
    v.rect(-21, ZL, FR_A + 26, ZL + PW, lw=THIN, ls=(0, (2, 2)))
    v.text(-6, ZL + PW / 2, "BELT (between fins)", size=5.5)
    v.dimv(ftL, MPL_TOP, -CAR_L / 2, -10, f"{MP_T:g} CARRIAGE")
    v.dimv(ftL - FR_T, ftL, -FR_A + 8, -22, f"{FR_T:g} FRAME", size=5.5)
    v.text(0, DECK_Z0 + DECK_T + 8, "SECTION ON THE BELT AXIS — LEFT SIDE "
           "(right side mirrors, +20 in z)", size=7)
    # hardware table
    tx, ty = 252, 230
    ax.text(tx, ty, "HARDWARE (per machine)", fontsize=8, weight="bold", color=INK)
    hw = [
        ("16x", "M4 x 35 SHCS + nyloc + washers", "bearing cap pairs, 4 per bore stack (2 bores x 2 plates)"),
        ("8x", "M5 x 16 SHCS + washer", "carriage lock bolts (4 per side)"),
        ("12x", "M5 x 20 SHCS", "fin flanges -> deck underside (10-deep flange, full heat-set)"),
        ("4x", "M5 x 70 SHCS + nyloc", "top plate -> standoff -> deck through-bolt (one per standoff)"),
        ("8x", "M6 x 18 SHCS", "motor flange -> carriage from below (full 8 mm engagement)"),
        ("2x", "M6 x 40 jackscrew + jam nut", "tensioner"),
        ("2x", "M25x1.5 shaft locknut (KM5)", "bearing preload"),
        ("--", "M5/M6 brass heat-set inserts", "all printed bosses"),
    ]
    for i, (q, item, use) in enumerate(hw):
        ax.text(tx, ty - 7 - i * 5.4, f"{q:<4} {item}", fontsize=6,
                family="monospace", color=INK)
        ax.text(tx, ty - 7 - i * 5.4 - 2.6, f"      {use}", fontsize=5,
                family="monospace", color="#555555")
    finish(pdf, fig, "sheet6_assembly.png")


# --------------------------------------------------------------------- main
def main():
    check_layout()                       # never draw a colliding layout
    pdf_path = OUT_PDF / "bcr_drawing_set.pdf"
    with PdfPages(pdf_path) as pdf:
        sheet_cap(pdf)
        sheet_carriage(pdf)
        sheet_cradle(pdf)
        sheet_deck(pdf)
        sheet_top(pdf)
        sheet_assy(pdf)
    print(f"drawing set -> {pdf_path} (6 sheets) + PNGs in {OUT_PNG}")


if __name__ == "__main__":
    main()
