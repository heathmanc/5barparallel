#!/usr/bin/env python3
"""Emit importable Studio 5000 files for the ClearLink motion side of the robot.

Three kinds of file, imported **in this order**:

    docs/l5x/1_VisionRobot_Cmd.L5X      \\
    docs/l5x/2_VisionRobot_Target.L5X    |  the VisionRobot UDT, one data type
    docs/l5x/3_VisionRobot_Manual.L5X    |  per file, imported in numeric order
    docs/l5x/4_VisionRobot_Status.L5X    |  (leaves 1-4 first, then the parent).
    docs/l5x/5_VisionRobot.L5X          /   Assets/Data Types -> Import  (FIRST)

    docs/l5x/RobotTags.csv              the glue tags + tuning values + the
                                       VisionRobot tag.  Controller Tags ->
                                       Import (SECOND — needs the UDT to exist)

    docs/l5x/R_MoveMotor0.L5X   absolute move, Motor 0  \\  right-click a Program
    docs/l5x/R_MoveMotor1.L5X   absolute move, Motor 1   |  -> Import Routine…
    docs/l5x/R_HomeMotor0.L5X   ClearLink homing, Motor 0|  (LAST, after the
    docs/l5x/R_HomeMotor1.L5X   ClearLink homing, Motor 1|  UDT + tags exist)
    docs/l5x/R30_Homing.L5X     2-axis homing coordinator/

Why one UDT per file: Studio 5000 does not reliably *create* the nested
(dependency) types from a single combined export — the parent then references
types that don't exist yet and the import fails with "member … data type was
missing". Importing each type as its own Target, leaves before the parent, is
the documented, reliable order.

Why the tags are CSV: Logix's bulk tag import is the Rockwell tag CSV
(`TYPE,SCOPE,NAME,DESCRIPTION,DATATYPE,SPECIFIER,ATTRIBUTES`), not L5X. Note that
CSV import creates tag *definitions* only — **it does not set values**, so the
tuning values (STEPS_PER_DEG, MOVE_VEL, HOME_VEL_0/1, …) are entered once by hand
after import (you tune most of them at commissioning anyway). See docs/plc_setup.md.

The routines carry Rockwell **neutral rung text** (the form inside Teknic's own
`.L5K` examples). They are plain **Routines**, not Add-On Instructions: Teknic
ships no motion AOI, and a real AOI can't touch the `ClearLink:O1/:I1` module
tags directly — so these implement the design's `AOI_AxisMove`/`AOI_HomeAxis`
(docs/plc_program.md §5) as one routine per motor.

Run:  python scripts/render_plc_l5x.py

NOTE: generated without Studio 5000 — schema-conformant but not import-verified.
Import the UDT files (1..5) first, then the CSV, then the routines. Report any
error and I'll fix the generator.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple
from xml.sax.saxutils import escape

Rung = Tuple[str, str]  # (comment, neutral text)

# Controller name for the L5X Controller context. (The tag CSV keeps the SCOPE
# column EMPTY for controller tags — that's what a real Logix export does.)
CONTROLLER = "RobotController"
PROGRAM = "Robot"
EXPORT_OPTS = (
    "References NoRawData L5KData DecoratedData Context Dependencies "
    "ForceProtectedEncoding AllProjDocTrans"
)


def _content_open(target_name: str, target_type: str, sub: str = "") -> str:
    """The <RSLogix5000Content …> root open tag for a partial export."""
    extra = f' TargetSubType="{sub}"' if sub else ""
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
        f'<RSLogix5000Content SchemaRevision="1.0" SoftwareRevision="30.00" '
        f'TargetName="{escape(target_name)}" TargetType="{target_type}"{extra} '
        f'ContainsContext="true" ExportOptions="{EXPORT_OPTS}">'
    )


# --------------------------------------------------------------------------- #
# Routines (neutral rung text)
# --------------------------------------------------------------------------- #
def _rung(i: int, comment: str, text: str) -> str:
    return (
        f'<Rung Number="{i}" Type="N">\n'
        f'<Comment><![CDATA[{comment}]]></Comment>\n'
        f'<Text><![CDATA[{text}]]></Text>\n'
        f'</Rung>'
    )


def routine_l5x(name: str, rungs: List[Rung], program: str = PROGRAM) -> str:
    body = "\n".join(_rung(i, c, t) for i, (c, t) in enumerate(rungs))
    return f'''{_content_open(name, "Routine", sub="RLL")}
<Controller Use="Context" Name="{CONTROLLER}">
<Programs Use="Context">
<Program Use="Context" Name="{escape(program)}">
<Routines Use="Context">
<Routine Use="Target" Name="{escape(name)}" Type="RLL">
<RLLContent>
{body}
</RLLContent>
</Routine>
</Routines>
</Program>
</Programs>
</Controller>
</RSLogix5000Content>
'''


def move_rungs(m: int) -> List[Rung]:
    """Absolute-move routine for Motor `m` (mirrors Teknic SD_Position_Move)."""
    o = f"ClearLink:O1.Motor{m}_"
    i = f"ClearLink:I1.Motor{m}_"
    other = 1 - m
    return [
        (f"R_MoveMotor{m}: Motor {m} absolute move. Tags/constants come from "
         f"RobotTags.csv (Move{m}_Execute/ons/Fault/InPosition, Move{m}_Steps, "
         f"Move{m}_Target_Deg, EM806_{m}_ALM, STEPS_PER_DEG, MOVE_VEL, MOVE_ACC) "
         f"and the ClearLink module (ClearLink:O1/:I1). Import the VisionRobot UDT "
         f".L5X files + RobotTags.csv first. Twin routine: R_MoveMotor{other}.",
         f"OTE({o}Output_Reg_Enable);"),
        ("Convert the target angle to steps.",
         f"CPT(Move{m}_Steps,TRUNC(Move{m}_Target_Deg * STEPS_PER_DEG));"),
        ("Rising edge of Execute: load Move Distance / limits, set Absolute, and "
         "latch Load Position Data.",
         f"XIC(Move{m}_Execute)ONS(Move{m}_ons)XIO(Move{m}_Fault)"
         f"MOV(Move{m}_Steps,{o}Move_Dist)MOV(MOVE_VEL,{o}Vel_Limit)"
         f"MOV(MOVE_ACC,{o}Accel_Lim)OTL({o}Output_Reg_Abs_Flag)"
         f"OTL({o}Output_Reg_Load_Posn_Data);"),
        ("ClearLink acknowledges the load -> drop Load Position Data.",
         f"XIC({i}Status_Load_Posn_Move_Ack)OTU({o}Output_Reg_Load_Posn_Data);"),
        ("Move done = At Target Position (needs HLFB Inversion for the EM806).",
         f"XIC({i}Status_At_Target_Posn)XIO({o}Output_Reg_Load_Posn_Data)"
         f"OTE(Move{m}_InPosition);"),
        ("Motor fault, ClearLink shutdown, or EM806 alarm -> Fault.",
         f"[XIC({i}Status_Motor_In_Fault),XIC({i}Status_Shutdowns_Pres),"
         f"XIC(EM806_{m}_ALM)]OTE(Move{m}_Fault);"),
    ]


def home_rungs(m: int) -> List[Rung]:
    """ClearLink homing routine for Motor `m` (mirrors Teknic SD_Homing)."""
    o = f"ClearLink:O1.Motor{m}_"
    i = f"ClearLink:I1.Motor{m}_"
    return [
        (f"R_HomeMotor{m}: Motor {m} homing. First set ClearLink:C.Motor{m}Config "
         f"(Home_Sensor = the prox input, Config Register Homing Enable bit0=1, "
         f"HLFB Inversion bit3=1). Tags come from RobotTags.csv (Home{m}_State, "
         f"Home{m}_Req/ons, Ax{m}_HomeDone/HomeFault, Home{m}_Tmr, EM806_{m}_ALM, "
         f"HOME_VEL_{m} signed toward the prox, HOME_ACC). R30_Homing JSRs "
         f"R_HomeMotor0/R_HomeMotor1.",
         f"XIC(Home{m}_Req)ONS(Home{m}_ons)EQU(Home{m}_State,0)XIO(Ax{m}_HomeFault)"
         f"MOV(10,Home{m}_State);"),
        ("State 10 ENABLING: hold the Enable output.",
         f"EQU(Home{m}_State,10)OTE({o}Output_Reg_Enable);"),
        ("State 10: advance once HLFB is asserted.",
         f"EQU(Home{m}_State,10)XIC({i}Status_HLFB_ON)MOV(20,Home{m}_State);"),
        ("State 20 CLEAR MOTOR FAULTS.",
         f"EQU(Home{m}_State,20)OTE({o}Output_Reg_Clear_Fault);"),
        ("State 20: on Clear-Fault ack, drop the request and advance.",
         f"EQU(Home{m}_State,20)XIC({i}Status_Clear_Motor_Fault_Ack)"
         f"OTU({o}Output_Reg_Clear_Fault)MOV(30,Home{m}_State);"),
        ("State 30 CLEAR ALERTS.",
         f"EQU(Home{m}_State,30)OTE({o}Output_Reg_Clear_Alerts);"),
        ("State 30: when no shutdowns remain, drop the request and advance.",
         f"EQU(Home{m}_State,30)XIO({i}Status_Shutdowns_Pres)"
         f"OTU({o}Output_Reg_Clear_Alerts)MOV(40,Home{m}_State);"),
        ("State 40: wait for Ready To Home.",
         f"EQU(Home{m}_State,40)XIC({i}Status_Ready_To_Home)MOV(50,Home{m}_State);"),
        ("State 50 BEGIN HOMING MOVE: home flag + velocity move toward the prox.",
         f"EQU(Home{m}_State,50)OTL({o}Output_Reg_Home_Flag)"
         f"MOV(HOME_VEL_{m},{o}Jog_Vel)MOV(HOME_ACC,{o}Accel_Lim)"
         f"OTL({o}Output_Reg_Load_Vel_Data)MOV(55,Home{m}_State);"),
        ("State 55: on Load-Velocity ack, drop the load bit and homing flag.",
         f"EQU(Home{m}_State,55)XIC({i}Status_Load_Vel_Move_Ack)"
         f"OTU({o}Output_Reg_Load_Vel_Data)OTU({o}Output_Reg_Home_Flag)"
         f"MOV(60,Home{m}_State);"),
        ("State 60 HOMING: ClearLink zeroes at the prox -> Has Homed.",
         f"EQU(Home{m}_State,60)XIC({i}Status_Has_Homed)OTL(Ax{m}_HomeDone)"
         f"MOV(70,Home{m}_State);"),
        ("Homing timeout (timer runs while homing) -> fault.",
         f"TONR(Home{m}_Tmr)XIC(Home{m}_Tmr.DN)OTL(Ax{m}_HomeFault)"
         f"MOV(900,Home{m}_State);"),
        ("EM806 alarm or ClearLink shutdown -> fault.",
         f"[XIC(EM806_{m}_ALM),XIC({i}Status_Shutdowns_Pres)]"
         f"OTL(Ax{m}_HomeFault)MOV(900,Home{m}_State);"),
        ("State 900 FAULT: clear the move bits, hold for reset.",
         f"EQU(Home{m}_State,900)OTU({o}Output_Reg_Home_Flag)"
         f"OTU({o}Output_Reg_Load_Vel_Data);"),
    ]


COORD: List[Rung] = [
    ("R30_Homing: sequential 2-axis homing coordinator. JSRs R_HomeMotor0 / "
     "R_HomeMotor1 and publishes VisionRobot.Status. Tags from RobotTags.csv "
     "(HomeStep, HR_ons, SoftLimitsEnable, Home0_Req/Home1_Req, Ax0/Ax1_HomeDone/"
     "HomeFault, HOME_OFFSET_L/HOME_OFFSET_R, STEPS_PER_DEG). Import the "
     "VisionRobot UDT .L5X files + RobotTags.csv first.",
     "XIC(VisionRobot.Manual.HomeRequest)ONS(HR_ons)EQU(HomeStep,0)"
     "XIC(VisionRobot.Status.Enabled)XIO(VisionRobot.Status.Faulted)"
     "MOV(10,HomeStep)OTU(VisionRobot.Status.Homed)OTL(Home0_Req);"),
    ("Axis 0 (left) homed -> start Axis 1 (right).",
     "EQU(HomeStep,10)XIC(Ax0_HomeDone)OTU(Home0_Req)OTL(Home1_Req)"
     "MOV(20,HomeStep);"),
    ("Axis 1 (right) homed.",
     "EQU(HomeStep,20)XIC(Ax1_HomeDone)OTU(Home1_Req)MOV(30,HomeStep);"),
    ("Publish angles with the home offset, enable soft limits, return to idle.",
     "EQU(HomeStep,30)"
     "CPT(VisionRobot.Status.ActualLeftDeg,"
     "(ClearLink:I1.Motor0_CommandedPosn + HOME_OFFSET_L) / STEPS_PER_DEG)"
     "CPT(VisionRobot.Status.ActualRightDeg,"
     "(ClearLink:I1.Motor1_CommandedPosn + HOME_OFFSET_R) / STEPS_PER_DEG)"
     "OTE(SoftLimitsEnable)OTL(VisionRobot.Status.Homed)MOV(0,HomeStep);"),
    ("Run the per-axis homing routines every scan (they self-idle at rest).",
     "JSR(R_HomeMotor0,0)JSR(R_HomeMotor1,0);"),
    ("Either axis homing fault -> homing fault (FaultCode 4).",
     "[XIC(Ax0_HomeFault),XIC(Ax1_HomeFault)]OTL(VisionRobot.Status.Faulted)"
     "MOV(4,VisionRobot.Status.FaultCode)MOV(900,HomeStep);"),
]


# --------------------------------------------------------------------------- #
# DataType exports — one VisionRobot UDT per file (leaves first, then parent).
# BOOLs bit-pack into hidden SINT hosts exactly as Logix exports them.
# --------------------------------------------------------------------------- #
Member = Tuple[str, str]  # (name, data type)

_DEC = {"SINT", "INT", "DINT", "LINT", "USINT", "UINT", "UDINT", "ULINT", "BIT"}


def _member_line(name: str, dtype: str, *, hidden=False,
                 target: Optional[str] = None, bit: Optional[int] = None) -> str:
    if dtype == "REAL":
        radix = "Float"
    elif dtype in _DEC:
        radix = "Decimal"
    else:                      # nested UDT / structure member
        radix = "NullType"
    attrs = (f'Name="{name}" DataType="{dtype}" Dimension="0" Radix="{radix}" '
             f'Hidden="{"true" if hidden else "false"}"')
    if target is not None:
        attrs += f' Target="{target}" BitNumber="{bit}"'
    attrs += ' ExternalAccess="Read/Write"'
    return f"<Member {attrs}/>"


def _pack_members(udt: str, members: List[Member]) -> str:
    """Emit <Member> lines, bit-packing BOOL runs into hidden SINT hosts.

    A non-BOOL member terminates the current BOOL run (the next BOOL opens a new
    host); a run longer than 8 bits also rolls to a new host — exactly how Logix
    lays out a UDT on export.
    """
    lines: List[str] = []
    host: Optional[str] = None
    bit = 0
    host_idx = 0
    for name, dtype in members:
        if dtype == "BOOL":
            if host is None or bit == 8:
                host = f"ZZZZZZZZZZ{udt}{host_idx}"
                host_idx += 1
                bit = 0
                lines.append(_member_line(host, "SINT", hidden=True))
            lines.append(_member_line(name, "BIT", target=host, bit=bit))
            bit += 1
        else:
            host, bit = None, 0
            lines.append(_member_line(name, dtype))
    return "\n".join(lines)


UDT_CMD: List[Member] = [
    ("RequestPickPlace", "BOOL"), ("Abort", "BOOL"), ("Reset", "BOOL"),
    ("CommandID", "DINT"),
]
UDT_TARGET: List[Member] = [
    ("Pick_LeftDeg", "REAL"), ("Pick_RightDeg", "REAL"),
    ("Drop_LeftDeg", "REAL"), ("Drop_RightDeg", "REAL"),
    ("HoleIndex", "DINT"), ("CoverID", "DINT"),
]
UDT_MANUAL: List[Member] = [
    ("Enable", "BOOL"), ("HomeRequest", "BOOL"), ("MoveToTarget", "BOOL"),
    ("Abort", "BOOL"), ("TargetLeftDeg", "REAL"), ("TargetRightDeg", "REAL"),
    ("CommandID", "DINT"),
]
UDT_STATUS: List[Member] = [
    ("Ready", "BOOL"), ("Busy", "BOOL"), ("Done", "BOOL"), ("Faulted", "BOOL"),
    ("FaultCode", "DINT"),
    ("Enabled", "BOOL"), ("Homed", "BOOL"), ("InPosition", "BOOL"),
    ("Moving", "BOOL"),
    ("ActualLeftDeg", "REAL"), ("ActualRightDeg", "REAL"),
    ("ActiveCommandID", "DINT"), ("CompleteCommandID", "DINT"),
    ("FailedCommandID", "DINT"),
    ("VacuumOK", "BOOL"), ("CameraClear", "BOOL"), ("ReadyForVision", "BOOL"),
]
UDT_PARENT: List[Member] = [
    ("Cmd", "VisionRobot_Cmd"), ("Target", "VisionRobot_Target"),
    ("Manual", "VisionRobot_Manual"), ("Status", "VisionRobot_Status"),
]

# import order: the four leaves (any order), then the parent that references them
UDT_FILES: List[Tuple[str, str, List[Member]]] = [
    ("1_VisionRobot_Cmd.L5X", "VisionRobot_Cmd", UDT_CMD),
    ("2_VisionRobot_Target.L5X", "VisionRobot_Target", UDT_TARGET),
    ("3_VisionRobot_Manual.L5X", "VisionRobot_Manual", UDT_MANUAL),
    ("4_VisionRobot_Status.L5X", "VisionRobot_Status", UDT_STATUS),
    ("5_VisionRobot.L5X", "VisionRobot", UDT_PARENT),
]


def datatype_l5x(name: str, members: List[Member]) -> str:
    """One DataType as the single Target — no embedded dependencies, so the
    referenced leaf types must already exist (import order 1..5)."""
    return f'''{_content_open(name, "DataType")}
<Controller Use="Context" Name="{CONTROLLER}">
<DataTypes Use="Context">
<DataType Use="Target" Name="{name}" Family="NoFamily" Class="User">
<Members>
{_pack_members(name, members)}
</Members>
</DataType>
</DataTypes>
</Controller>
</RSLogix5000Content>
'''


# --------------------------------------------------------------------------- #
# Tag export — Rockwell tag CSV (definitions only; values set by hand after).
# --------------------------------------------------------------------------- #
class Tag:
    def __init__(self, name: str, dtype: str, value="0", *,
                 constant=False, desc: str = "") -> None:
        self.name, self.dtype, self.value = name, dtype, value
        self.constant, self.desc = constant, desc

    def csv_row(self) -> str:
        # Format matched byte-for-byte to a real Logix v34 export:
        # TYPE,SCOPE,NAME,DESCRIPTION,DATATYPE,SPECIFIER,ATTRIBUTES
        # SCOPE empty for controller tags; DESCRIPTION/DATATYPE/SPECIFIER/
        # ATTRIBUTES quoted; NAME bare; attribute order RADIX, Constant, Access.
        if self.dtype in ("TIMER", "VisionRobot"):        # structured: access only
            attrs = "ExternalAccess := Read/Write"
        else:
            radix = "Float" if self.dtype == "REAL" else "Decimal"
            const = "true" if self.constant else "false"
            attrs = (f"RADIX := {radix}, Constant := {const}, "
                     f"ExternalAccess := Read/Write")
        return (f"TAG,,{self.name},{_csv(self.desc)},{_csv(self.dtype)},"
                f"{_csv('')},{_csv('(' + attrs + ')')}")


def _csv(field: str) -> str:
    """Quote a CSV field, doubling any embedded double-quote."""
    return '"' + field.replace('"', '""') + '"'


def _glue_tags() -> List[Tag]:
    """Every controller-scope tag the whole PLC program references, excluding the
    ClearLink module tags (from the AOP) and the VisionRobot.* members (from the
    UDT). Grouped by the routine that owns it (plc_program.md / plc_ladder.md /
    plc_homing.md). Physical I/O tags are base BOOLs to alias/map to real points.
    """
    tags: List[Tag] = []

    def add(name, dtype, value="0", constant=False, desc=""):
        tags.append(Tag(name, dtype, value, constant=constant, desc=desc))

    # --- constants & tuning values (CSV carries no values — set after import) ---
    add("STEPS_PER_DEG", "REAL", "26.66667", True,
        "3200 pulses/rev * 3:1 / 360. Set value to 26.66667 after import.")
    add("MOVE_VEL", "DINT", "20000", desc="Move speed, steps/s (max 500000). Set ~20000.")
    add("MOVE_ACC", "DINT", "100000", desc="Move accel, steps/s^2. Set ~100000.")
    add("MOVE_DEC", "DINT", "0", desc="Move decel, steps/s^2. 0 => use accel.")
    add("HOME_VEL_0", "DINT", "-2000",
        desc="Motor 0 homing speed, steps/s, signed toward the prox. Tune.")
    add("HOME_VEL_1", "DINT", "2000",
        desc="Motor 1 homing speed, steps/s, signed toward the prox. Tune.")
    add("HOME_ACC", "DINT", "50000", desc="Homing accel, steps/s^2. Set ~50000.")
    add("HOME_OFFSET_L", "DINT", "0",
        desc="Left switch angle * STEPS_PER_DEG (ActualLeftDeg ~135.85). Set at commissioning.")
    add("HOME_OFFSET_R", "DINT", "0",
        desc="Right switch angle * STEPS_PER_DEG (ActualRightDeg ~44.15). Set at commissioning.")
    add("VAC_SETTLE", "DINT", "300", desc="Vacuum settle time, ms (VacTmr preset). Tune.")
    add("BLOWOFF_TIME", "DINT", "200", desc="Blowoff time, ms (BlowTmr preset). Tune.")
    add("CAMERA_CLEAR_L", "REAL", "0.0",
        desc="Camera-clear pose, left shoulder deg. Set to a safe out-of-view pose.")
    add("CAMERA_CLEAR_R", "REAL", "0.0",
        desc="Camera-clear pose, right shoulder deg. Set to a safe out-of-view pose.")

    # --- per-motor move + home glue (R_MoveMotor* / R_HomeMotor*) ---
    for m in (0, 1):
        add(f"Move{m}_Execute", "BOOL", desc=f"Rising edge starts a Motor {m} absolute move.")
        add(f"Move{m}_ons", "BOOL", desc=f"ONS storage for Move{m}_Execute.")
        add(f"Move{m}_Fault", "BOOL", desc=f"Motor {m} move fault (drive fault / shutdown / ALM).")
        add(f"Move{m}_InPosition", "BOOL", desc=f"Motor {m} at target.")
        add(f"Move{m}_Steps", "DINT", desc=f"Motor {m} target in steps (deg * STEPS_PER_DEG).")
        add(f"Move{m}_Target_Deg", "REAL", "0.0", desc=f"Motor {m} absolute target angle, deg.")
        add(f"Home{m}_Req", "BOOL", desc=f"Request homing of Motor {m} (set by R30_Homing).")
        add(f"Home{m}_ons", "BOOL", desc=f"ONS storage for Home{m}_Req.")
        add(f"Home{m}_State", "DINT", desc=f"Motor {m} homing sub-state.")
        add(f"Home{m}_Tmr", "TIMER", desc=f"Motor {m} homing timeout timer.")
        add(f"Ax{m}_HomeDone", "BOOL", desc=f"Motor {m} homed.")
        add(f"Ax{m}_HomeFault", "BOOL", desc=f"Motor {m} homing fault.")
        add(f"Ax{m}_Ready", "BOOL",
            desc=f"Motor {m} drive ready (map to ClearLink:I1.Motor{m}_Status_Enabled).")
        add(f"EM806_{m}_ALM", "BOOL",
            desc=f"Motor {m} EM806 ALM. Change to an ALIAS of the ClearLink DI the drive alarm is wired to.")

    # --- homing coordinator (R30_Homing) ---
    add("HomeStep", "DINT", desc="Homing coordinator state.")
    add("HR_ons", "BOOL", desc="ONS storage for VisionRobot.Manual.HomeRequest.")
    add("SoftLimitsEnable", "BOOL", desc="Enable soft limits after homing (mirror Config Register SoftLimitEnable).")

    # --- safety (R10_Safety): physical inputs + logic bits ---
    add("EStop_Pressed", "BOOL", desc="E-stop pressed input. Map/alias to the hardwired E-stop.")
    add("EStop_OK", "BOOL", desc="E-stop circuit healthy (safety-relay feedback). Map to input.")
    add("Guard_Closed", "BOOL", desc="Guard / interlock closed input. Map to input.")
    add("Ax0_LimitMin", "BOOL", desc="Left shoulder min hard-limit (-20 deg) input. Map to input.")
    add("Ax0_LimitMax", "BOOL", desc="Left shoulder max hard-limit (+200 deg) input. Map to input.")
    add("Ax1_LimitMin", "BOOL", desc="Right shoulder min hard-limit input. Map to input.")
    add("Ax1_LimitMax", "BOOL", desc="Right shoulder max hard-limit input. Map to input.")
    add("SafetyOK", "BOOL", desc="Aggregate safe: no fault, E-stop OK, guard closed.")
    add("EnableReq", "BOOL", desc="Drive-enable request (set by manual/auto).")
    add("Reset_prev", "BOOL", desc="Edge storage for VisionRobot.Cmd.Reset in R10.")

    # --- manual jog/home (R40_Manual) + mode selector ---
    add("AutoMode", "BOOL", desc="Mode: 1 = auto (R50) owns motion, 0 = manual (R40).")
    add("WithinLimits", "BOOL", desc="Manual target within -20..+200 deg soft limits.")
    add("MoveActive", "BOOL", desc="A manual coordinated move is in progress.")
    add("MTT_prev", "BOOL", desc="Edge storage for VisionRobot.Manual.MoveToTarget.")

    # --- automatic pick/place (R50_Auto) ---
    add("State", "DINT", desc="Auto pick/place state (0 idle .. 200 done, 900 fault).")
    add("RPP_prev", "BOOL", desc="Edge storage for VisionRobot.Cmd.RequestPickPlace.")
    add("AR_prev", "BOOL", desc="Edge storage for VisionRobot.Cmd.Reset in R50.")
    add("VacTmr", "TIMER", desc="Vacuum settle timer (preset VAC_SETTLE).")
    add("BlowTmr", "TIMER", desc="Blowoff timer (preset BLOWOFF_TIME).")
    add("CmdCameraClear", "BOOL", desc="Dispatch: move to the camera-clear pose.")
    add("CmdMovePick", "BOOL", desc="Dispatch: move above the pick.")
    add("CmdMoveDrop", "BOOL", desc="Dispatch: move above the drop.")
    add("Arrived", "BOOL", desc="Both axes in position after an auto move.")
    add("AtCameraClear", "BOOL", desc="Arrived at the camera-clear pose.")
    add("AtPick", "BOOL", desc="Arrived above the pick.")
    add("AtDrop", "BOOL", desc="Arrived above the drop.")
    add("AutoTL", "REAL", "0.0", desc="Auto move target, left shoulder deg.")
    add("AutoTR", "REAL", "0.0", desc="Auto move target, right shoulder deg.")
    add("AutoMove", "BOOL", desc="Auto move request (drives Move0/Move1 in auto mode).")
    add("PickL", "REAL", "0.0", desc="Working copy of VisionRobot.Target.Pick_LeftDeg (latched state 40).")
    add("PickR", "REAL", "0.0", desc="Working copy of VisionRobot.Target.Pick_RightDeg.")
    add("DropL", "REAL", "0.0", desc="Working copy of VisionRobot.Target.Drop_LeftDeg.")
    add("DropR", "REAL", "0.0", desc="Working copy of VisionRobot.Target.Drop_RightDeg.")
    add("CylinderDown", "BOOL", desc="Z down solenoid output. Map/alias to the actual output.")
    add("VacuumOn", "BOOL", desc="Vacuum solenoid output. Map/alias to the actual output.")
    add("Blowoff", "BOOL", desc="Blowoff solenoid output. Map/alias to the actual output.")
    add("PickDown", "BOOL", desc="Z-down-at-pick reed switch input. Map to input.")
    add("PickUp", "BOOL", desc="Z-up-at-pick reed switch input. Map to input.")
    add("DropDown", "BOOL", desc="Z-down-at-drop reed switch input. Map to input.")
    add("DropUp", "BOOL", desc="Z-up-at-drop reed switch input. Map to input.")
    add("VacuumSensor", "BOOL", desc="Vacuum-confirm sensor input. Map to input.")

    # --- the vision handshake surface (import the UDT files first) ---
    add("VisionRobot", "VisionRobot",
        desc="Vision-PC handshake surface (pycomm3 by name). Import the VisionRobot UDT .L5X files first.")
    return tags


def robot_tags_csv() -> str:
    header = [
        'remark,"CSV-Import-Export"',
        'remark,"Date = Mon Jul  6 12:00:00 2026"',
        'remark,"Version = RSLogix 5000 v34.01"',
        'remark,"Owner = "',
        'remark,"Company = "',
        "0.3",   # CSV format-version line — Logix expects it before the header
        "TYPE,SCOPE,NAME,DESCRIPTION,DATATYPE,SPECIFIER,ATTRIBUTES",
    ]
    rows = [t.csv_row() for t in _glue_tags()]
    # CRLF — Studio 5000 (Windows) imports nothing from an LF-only CSV.
    return "\r\n".join(header + rows) + "\r\n"


# --------------------------------------------------------------------------- #
def main() -> None:
    global CONTROLLER
    if len(sys.argv) > 1 and sys.argv[1].strip():
        CONTROLLER = sys.argv[1].strip()
        print(f"controller/scope = {CONTROLLER}")
    out = Path(__file__).resolve().parents[1] / "docs" / "l5x"
    out.mkdir(parents=True, exist_ok=True)

    # remove superseded files (old AOI_* routines, combined UDT, L5X tags)
    for stale in ("AOI_AxisMove.L5X", "AOI_HomeAxis.L5X",
                  "VisionRobot_UDT.L5X", "RobotTags.L5X"):
        p = out / stale
        if p.exists():
            p.unlink()
            print(f"removed superseded {p.name}")

    xml_files = {name: datatype_l5x(udt, members)
                 for name, udt, members in UDT_FILES}
    xml_files.update({
        "R_MoveMotor0.L5X": routine_l5x("R_MoveMotor0", move_rungs(0)),
        "R_MoveMotor1.L5X": routine_l5x("R_MoveMotor1", move_rungs(1)),
        "R_HomeMotor0.L5X": routine_l5x("R_HomeMotor0", home_rungs(0)),
        "R_HomeMotor1.L5X": routine_l5x("R_HomeMotor1", home_rungs(1)),
        "R30_Homing.L5X": routine_l5x("R30_Homing", COORD),
    })
    for name, text in xml_files.items():
        ET.fromstring(text)                      # well-formedness check
        (out / name).write_text(text)
        print(f"wrote {out / name}")

    csv_text = robot_tags_csv()
    # write_bytes so the CRLF line endings survive verbatim on any platform
    (out / "RobotTags.csv").write_bytes(csv_text.encode("ascii"))
    print(f"wrote {out / 'RobotTags.csv'}")


if __name__ == "__main__":
    main()
