#!/usr/bin/env python3
"""Render the ClearLink motion ladder diagrams to SVG.

Regenerates the three diagrams that depend on the ClearLink Step-Direction
interface, kept in sync with the Structured Text in docs/plc_program.md,
docs/plc_homing.md, and docs/plc_ladder.md (which follow Teknic's shipped
CompactLogix examples SD_Position_Move / SD_Homing):

    docs/plc_axismove_ladder.svg       AOI_AxisMove
    docs/plc_homing_axis_ladder.svg    AOI_HomeAxis
    docs/plc_homing_coord_ladder.svg   R30_Homing coordinator

Run:  python scripts/render_plc_ladders.py
It is a self-contained emitter (no third-party deps) so the SVGs can be rebuilt
whenever the logic changes. The visual style matches the other docs/*_ladder.svg.
"""

from __future__ import annotations

import html
from pathlib import Path
from typing import List, Tuple

# --- canvas / geometry ------------------------------------------------------
W = 1240
L_RAIL = 60
R_RAIL = 1180
FONT = "DejaVu Sans, sans-serif"

ROW_DY = 56          # vertical spacing between stacked outputs on one rung
BRANCH_DY = 30       # vertical spacing between OR-parallel contacts
CHAR = 6.0           # ~px per char at font-size 10/11


def _t(x, y, s, size=11, anchor="start", weight="normal", style="normal", fill="black"):
    extra = ""
    if weight != "normal":
        extra += f' font-weight="{weight}"'
    if style != "normal":
        extra += f' font-style="{style}"'
    if fill != "black":
        extra += f' fill="{fill}"'
    a = f' text-anchor="{anchor}"' if anchor != "start" else ""
    return (f'<text x="{x}" y="{y}" font-family="{FONT}" font-size="{size}"{a}{extra}>'
            f'{html.escape(s)}</text>')


def _line(x1, y1, x2, y2, w=1.0, color="black"):
    sw = f' stroke-width="{w}"' if w != 1.0 else ""
    return f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}"{sw}/>'


def _rect(x, y, w, h, fill="none"):
    return f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="black"/>'


def _txtw(s: str) -> float:
    return len(s) * CHAR


# --- series elements (return exit_x) ----------------------------------------
def contact(out: List[str], x0: float, y: float, label: str, closed: bool) -> float:
    """XIC (closed=False -> ] [) / XIO (closed=True -> ]/[). Occupies a slot that
    fully contains its label so adjacent contacts never collide."""
    adv = max(46.0, _txtw(label) + 16)
    cx = x0 + adv / 2
    lb, rb = cx - 15, cx + 15
    out.append(_line(x0, y, lb, y))
    out.append(_line(lb, y - 11, lb, y + 11, 2))
    out.append(_line(rb, y - 11, rb, y + 11, 2))
    if closed:
        out.append(_line(lb, y + 11, rb, y - 11, 1.5))
    out.append(_line(rb, y, x0 + adv, y))          # trailing wire to slot end
    out.append(_t(cx, y - 14, label, anchor="middle"))
    return x0 + adv


def ons(out: List[str], x0: float, y: float, inst: str) -> float:
    lb = x0 + 10
    out.append(_line(x0, y, lb, y))
    out.append(_rect(lb, y - 11, 40, 22))
    out.append(_t(lb + 20, y + 4, "ONS", size=10, anchor="middle"))
    out.append(_t(lb + 20, y - 14, inst, size=9, anchor="middle"))
    return lb + 40


def cmp_box(out: List[str], x0: float, y: float, op: str, expr: str) -> float:
    w = max(101.0, _txtw(expr) + 16)
    lb = x0 + 6
    out.append(_line(x0, y, lb, y))
    out.append(_rect(lb, y - 20, w, 40, "#fff"))
    out.append(_t(lb + w / 2, y - 4, op, anchor="middle", weight="bold"))
    out.append(_t(lb + w / 2, y + 14, expr, size=10, anchor="middle"))
    return lb + w


def or_group(out: List[str], x0: float, y: float, contacts: List[Tuple[str, bool]]) -> float:
    """Parallel (OR) contacts. Returns exit_x; rejoins on the right."""
    widths = [max(46.0, _txtw(lb) + 16) for lb, _ in contacts]
    adv = max(widths)
    lb, rb = x0 + 8, x0 + 8 + adv
    # left + right vertical bus of the branch
    ys = [y + i * BRANCH_DY for i in range(len(contacts))]
    out.append(_line(lb, ys[0], lb, ys[-1]))
    out.append(_line(rb, ys[0], rb, ys[-1]))
    for (label, closed), yy in zip(contacts, ys):
        cx = (lb + rb) / 2
        a, b = cx - 15, cx + 15
        out.append(_line(lb, yy, a, yy))
        out.append(_line(b, yy, rb, yy))
        out.append(_line(a, yy - 11, a, yy + 11, 2))
        out.append(_line(b, yy - 11, b, yy + 11, 2))
        if closed:
            out.append(_line(a, yy + 11, b, yy - 11, 1.5))
        out.append(_t(cx, yy - 14, label, anchor="middle"))
    out.append(_line(x0, y, lb, y))
    return rb


# --- output elements (drawn right-aligned; connect to right rail) -----------
def mov(out: List[str], jx: float, y: float, src: str, dest: str) -> None:
    w = max(150.0, _txtw("Source " + src) + 16, _txtw("Dest " + dest) + 16)
    bx = R_RAIL - w
    out.append(_line(jx, y, bx, y))
    out.append(_rect(bx, y - 23, w, 46, "#eef"))
    out.append(_t(bx + w / 2, y - 9, "MOV", anchor="middle", weight="bold"))
    out.append(_t(bx + 8, y + 6, "Source " + src, size=10))
    out.append(_t(bx + 8, y + 20, "Dest " + dest, size=10))
    out.append(_line(R_RAIL, y, R_RAIL, y))


def cpt(out: List[str], jx: float, y: float, expr: str) -> None:
    w = max(150.0, _txtw(expr) + 16)
    bx = R_RAIL - w
    out.append(_line(jx, y, bx, y))
    out.append(_rect(bx, y - 17, w, 34, "#eef"))
    out.append(_t(bx + w / 2, y - 3, "CPT", anchor="middle", weight="bold"))
    out.append(_t(bx + 8, y + 12, expr, size=10))


def coil(out: List[str], jx: float, y: float, label: str, kind: str) -> None:
    cx = R_RAIL - 32
    a, b = cx - 16, cx + 16
    out.append(_line(jx, y, a, y))
    out.append(f'<path d="M {a} {y-11} A 14 14 0 0 0 {a} {y+11}" fill="none" stroke="black" stroke-width="2"/>')
    out.append(f'<path d="M {b} {y-11} A 14 14 0 0 1 {b} {y+11}" fill="none" stroke="black" stroke-width="2"/>')
    # anchor the (often long) tag label to end at the rail so it never overflows
    out.append(_t(R_RAIL, y - 14, label, anchor="end"))
    if kind in ("L", "U"):
        out.append(_t(cx, y + 4, kind, anchor="middle"))
    out.append(_line(b, y, R_RAIL, y))


def aoi_box(out: List[str], jx: float, y: float, title: str, lines: List[str]) -> None:
    w = 470.0
    h = 20 + 16 * len(lines)
    bx = R_RAIL - w
    out.append(_line(jx, y, bx, y))
    out.append(f'<rect x="{bx}" y="{y-h/2}" width="{w}" height="{h}" fill="#efe" stroke="black"/>')
    out.append(_t(bx + w / 2, y - h / 2 + 15, title, anchor="middle", weight="bold"))
    for i, ln in enumerate(lines):
        out.append(_t(bx + 8, y - h / 2 + 33 + i * 16, ln, size=10))
    out.append(_line(R_RAIL, y, R_RAIL, y))


# --- rung model -------------------------------------------------------------
class Rung:
    def __init__(self, comment: str, series, outputs, out_rows=None):
        self.comment = comment
        self.series = series      # list of series element callables
        self.outputs = outputs    # list of output element callables
        self.out_rows = out_rows  # None -> stacked; else explicit row index per output


def render(title: str, rungs: List[Rung], path: Path) -> None:
    body: List[str] = []
    y0 = 40
    for i, r in enumerate(rungs):
        wire_y = y0 + 63
        n_out = max(1, len(r.outputs))
        # series
        jx = L_RAIL
        for elem in r.series:
            jx = elem(body, jx, wire_y)
        if not r.series:
            jx = L_RAIL
        # rung header (number + comment)
        body.append(_t(10, wire_y + 4, str(i), size=12, fill="#555"))
        body.append(_t(L_RAIL, wire_y - 37, r.comment, style="italic", fill="#367"))
        # outputs (stacked)
        rows = [wire_y + k * ROW_DY for k in range(n_out)]
        if len(r.outputs) > 1:
            body.append(_line(jx, wire_y, jx, rows[-1]))
        for k, oelem in enumerate(r.outputs):
            oelem(body, jx, rows[k])
        if not r.outputs:  # series with no output -> wire to rail
            body.append(_line(jx, wire_y, R_RAIL, wire_y))
        # rung must be tall enough for both stacked outputs and any OR branch
        series_depth = max([getattr(e, "depth", 1) for e in r.series], default=1)
        last_y = max(rows[-1], wire_y + (series_depth - 1) * BRANCH_DY)
        sep_y = last_y + 37
        body.append(_line(L_RAIL, sep_y, R_RAIL, sep_y, color="#ddd"))
        y0 = sep_y
    height = y0 + 18
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{height}" '
           f'viewBox="0 0 {W} {height}"><rect width="{W}" height="{height}" fill="white"/>']
    svg.append(_t(W / 2, 26, title, size=18, anchor="middle", weight="bold"))
    svg.append(_line(L_RAIL, 40, L_RAIL, height - 10, 2.5))
    svg.append(_line(R_RAIL, 40, R_RAIL, height - 10, 2.5))
    svg.extend(body)
    svg.append("</svg>")
    path.write_text("".join(svg))
    print(f"wrote {path}  ({height}px, {len(rungs)} rungs)")


# --- convenience element builders ------------------------------------------
def XIC(label):  # noqa: N802
    return lambda o, x, y: contact(o, x, y, label, False)


def XIO(label):  # noqa: N802
    return lambda o, x, y: contact(o, x, y, label, True)


def ONS(inst):  # noqa: N802
    return lambda o, x, y: ons(o, x, y, inst)


def EQU(expr):  # noqa: N802
    return lambda o, x, y: cmp_box(o, x, y, "EQU", expr)


def OR(items):  # noqa: N802
    fn = lambda o, x, y: or_group(o, x, y, items)  # noqa: E731
    fn.depth = len(items)
    return fn


def MOV(src, dest):  # noqa: N802
    return lambda o, x, y: mov(o, x, y, src, dest)


def CPT(expr):  # noqa: N802
    return lambda o, x, y: cpt(o, x, y, expr)


def OTE(label):  # noqa: N802
    return lambda o, x, y: coil(o, x, y, label, "")


def OTL(label):  # noqa: N802
    return lambda o, x, y: coil(o, x, y, label, "L")


def OTU(label):  # noqa: N802
    return lambda o, x, y: coil(o, x, y, label, "U")


def AOI(title, lines):  # noqa: N802
    return lambda o, x, y: aoi_box(o, x, y, title, lines)


# --------------------------------------------------------------------------- #
# 1. AOI_AxisMove  (mirrors SD_Position_Move)
# --------------------------------------------------------------------------- #
AXISMOVE = [
    Rung("Energize this axis (Enable output held while the AOI runs)",
         [], [OTE("Ax.Enable  (Motor0_Output_Reg_Enable)")]),
    Rung("Convert the target angle to steps (always)",
         [], [CPT("TargetSteps := ROUND(TargetDeg * STEPS_PER_DEG)")]),
    Rung("Rising edge of Execute: load target, limits, absolute, and Load Position Data",
         [ONS("Exec_ons"), XIO("Fault")],
         [MOV("TargetSteps", "Motor0_Move_Dist"),
          MOV("MOVE_VEL", "Motor0_Vel_Limit"),
          MOV("MOVE_ACC", "Motor0_Accel_Lim"),
          OTL("Motor0_Output_Reg_Abs_Flag"),
          OTL("Motor0_Output_Reg_Load_Posn_Data")]),
    Rung("ClearLink acknowledges the load -> drop Load Position Data (handshake)",
         [XIC("Motor0_Status_Load_Posn_Move_Ack")],
         [OTU("Motor0_Output_Reg_Load_Posn_Data")]),
    Rung("Move done = At Target Position (set HLFB Inversion for the EM806; else use Steps Active)",
         [XIC("Motor0_Status_At_Target_Posn"), XIO("Motor0_Output_Reg_Load_Posn_Data")],
         [OTE("InPosition")]),
    Rung("Motor fault, ClearLink shutdown, or EM806 alarm -> Fault",
         [OR([("Motor0_Status_Motor_In_Fault", False),
              ("Motor0_Status_Shutdowns_Pres", False),
              ("Ax.ALM (EM806 ALM via DIP)", False)])],
         [OTE("Fault")]),
]

# --------------------------------------------------------------------------- #
# 2. AOI_HomeAxis  (mirrors SD_Homing; ClearLink runs the homing move)
# --------------------------------------------------------------------------- #
HOMEAXIS = [
    Rung("Idle -> start on HomeReq rising edge (not already faulted)",
         [ONS("HReq_ons"), EQU("Step = 0"), XIO("Fault")],
         [MOV("10", "Step")]),
    Rung("State 10 ENABLING: hold the Enable output",
         [EQU("Step = 10")], [OTE("Motor0_Output_Reg_Enable")]),
    Rung("State 10: advance once HLFB is asserted (EM806 needs HLFB Inversion, see notes)",
         [EQU("Step = 10"), XIC("Motor0_Status_HLFB_ON")], [MOV("20", "Step")]),
    Rung("State 20 CLEAR MOTOR FAULTS (ensures Ready To Home)",
         [EQU("Step = 20")], [OTE("Motor0_Output_Reg_Clear_Fault")]),
    Rung("State 20: on Clear-Fault ack, drop the request and advance",
         [EQU("Step = 20"), XIC("Motor0_Status_Clear_Motor_Fault_Ack")],
         [OTU("Motor0_Output_Reg_Clear_Fault"), MOV("30", "Step")]),
    Rung("State 30 CLEAR ALERTS (clears any latched shutdown)",
         [EQU("Step = 30")], [OTE("Motor0_Output_Reg_Clear_Alerts")]),
    Rung("State 30: when no shutdowns remain, drop the request and advance",
         [EQU("Step = 30"), XIO("Motor0_Status_Shutdowns_Pres")],
         [OTU("Motor0_Output_Reg_Clear_Alerts"), MOV("40", "Step")]),
    Rung("State 40: wait for Ready To Home",
         [EQU("Step = 40"), XIC("Motor0_Status_Ready_To_Home")], [MOV("50", "Step")]),
    Rung("State 50 BEGIN HOMING MOVE: home flag + velocity move toward the prox",
         [EQU("Step = 50")],
         [OTL("Motor0_Output_Reg_Home_Flag"),
          MOV("HomeVel (signed)", "Motor0_Jog_Vel"),
          MOV("HomeAccel", "Motor0_Accel_Lim"),
          OTL("Motor0_Output_Reg_Load_Vel_Data"),
          MOV("55", "Step")]),
    Rung("State 55: on Load-Velocity ack, drop the load bit and homing flag",
         [EQU("Step = 55"), XIC("Motor0_Status_Load_Vel_Move_Ack")],
         [OTU("Motor0_Output_Reg_Load_Vel_Data"),
          OTU("Motor0_Output_Reg_Home_Flag"), MOV("60", "Step")]),
    Rung("State 60 HOMING: ClearLink zeroes at the prox -> wait Has Homed",
         [EQU("Step = 60"), XIC("Motor0_Status_Has_Homed")],
         [OTL("Done"), MOV("70", "Step")]),
    Rung("Homing timeout (timer runs in states 10..60) -> fault",
         [XIC("HomeTmr.DN")], [OTL("Fault"), MOV("900", "Step")]),
    Rung("EM806 alarm or ClearLink shutdown at any time -> fault",
         [OR([("Ax.ALM (EM806 ALM via DIP)", False),
              ("Motor0_Status_Shutdowns_Pres", False)])],
         [OTL("Fault"), MOV("900", "Step")]),
    Rung("State 900 FAULT: clear move bits, hold for coordinator reset",
         [EQU("Step = 900")],
         [OTU("Motor0_Output_Reg_Home_Flag"),
          OTU("Motor0_Output_Reg_Load_Vel_Data")]),
]

# --------------------------------------------------------------------------- #
# 3. R30_Homing coordinator (sequential, offset-aware publish)
# --------------------------------------------------------------------------- #
COORD = [
    Rung("Start homing on HomeRequest edge (enabled, not faulted)",
         [ONS("HR_ons"), EQU("HomeStep = 0"),
          XIC("Status.Enabled"), XIO("Status.Faulted")],
         [MOV("10", "HomeStep"), OTU("Status.Homed"), OTL("Ax0_HomeReq")]),
    Rung("Axis 0 (left) homed -> start Axis 1 (right)",
         [EQU("HomeStep = 10"), XIC("Ax0_HomeDone")],
         [OTU("Ax0_HomeReq"), OTL("Ax1_HomeReq"), MOV("20", "HomeStep")]),
    Rung("Axis 1 (right) homed",
         [EQU("HomeStep = 20"), XIC("Ax1_HomeDone")],
         [OTU("Ax1_HomeReq"), MOV("30", "HomeStep")]),
    Rung("Publish angles with the home offset, enable soft limits, return to idle",
         [EQU("HomeStep = 30")],
         [CPT("Status.ActualLeftDeg := (Ax0.CmdPosition + HOME_OFFSET_L) / STEPS_PER_DEG"),
          CPT("Status.ActualRightDeg := (Ax1.CmdPosition + HOME_OFFSET_R) / STEPS_PER_DEG"),
          OTE("SoftLimitsEnable"), OTL("Status.Homed"), MOV("0", "HomeStep")]),
    Rung("Run the Axis 0 homing AOI every scan (self-idles when not requested)",
         [],
         [AOI("AOI_HomeAxis  HomeAxis0", [
             "HomeReq := Ax0_HomeReq   HomeVel := HOME_VEL   HomeAccel := HOME_ACC",
             "TimeoutPreset := HOME_TIMEOUT   Ax := Ax0",
             "Done -> Ax0_HomeDone   Fault -> Ax0_HomeFault"])]),
    Rung("Run the Axis 1 homing AOI every scan (HomeVel sign = approach direction)",
         [],
         [AOI("AOI_HomeAxis  HomeAxis1", [
             "HomeReq := Ax1_HomeReq   HomeVel := -HOME_VEL   HomeAccel := HOME_ACC",
             "TimeoutPreset := HOME_TIMEOUT   Ax := Ax1",
             "Done -> Ax1_HomeDone   Fault -> Ax1_HomeFault"])]),
    Rung("Either axis homing fault -> homing fault (FaultCode 4)",
         [OR([("Ax0_HomeFault", False), ("Ax1_HomeFault", False)])],
         [OTL("Status.Faulted"), MOV("4", "Status.FaultCode"),
          MOV("900", "HomeStep")]),
]


def main() -> None:
    docs = Path(__file__).resolve().parents[1] / "docs"
    render("AOI_AxisMove — move one shoulder to an absolute angle (ClearLink Step-Dir)",
           AXISMOVE, docs / "plc_axismove_ladder.svg")
    render("AOI_HomeAxis — home one shoulder via the ClearLink homing move",
           HOMEAXIS, docs / "plc_homing_axis_ladder.svg")
    render("R30_Homing — homing coordinator (both shoulders, sequential)",
           COORD, docs / "plc_homing_coord_ladder.svg")


if __name__ == "__main__":
    main()
