#!/usr/bin/env python3
"""Emit importable Studio 5000 .L5X for the ClearLink motion side of the robot.

Three kinds of partial-export file, imported **in this order**:

    docs/l5x/VisionRobot_UDT.L5X   the VisionRobot UDT (5 nested data types)
                                   -> Assets/Data Types -> Import  (do this FIRST)
    docs/l5x/RobotTags.L5X         glue tags + tuning constants + the VisionRobot
                                   tag  -> Controller Tags -> Import (SECOND)
    docs/l5x/R_MoveMotor0.L5X      absolute move, Motor 0  \\  right-click a
    docs/l5x/R_MoveMotor1.L5X      absolute move, Motor 1   |  Program ->
    docs/l5x/R_HomeMotor0.L5X      ClearLink homing, Motor 0|  Import Routine…
    docs/l5x/R_HomeMotor1.L5X      ClearLink homing, Motor 1|  (LAST, after the
    docs/l5x/R30_Homing.L5X        2-axis homing coordinator/  tags exist)

The routines carry Rockwell **neutral rung text** — the same form used inside
Teknic's own CompactLogix `.L5K` examples. They are plain **Routines**, not
Add-On Instructions: Teknic ships no motion AOI, and a real AOI can only touch
its own parameters/locals, not the `ClearLink:O1/:I1` module tags directly — so
these implement the design's `AOI_AxisMove`/`AOI_HomeAxis` (docs/plc_program.md
§5) as one routine per motor instead. `R30_Homing` `JSR`s `R_HomeMotor0` and
`R_HomeMotor1`, so both are shipped.

They reference the ClearLink AOP tags (`ClearLink:O1/:I1/:C`, created when you
add the "Step Dir" module), the `VisionRobot` UDT tag, and the glue tags — all of
which come from the two import files above.

Run:  python scripts/render_plc_l5x.py

NOTE: generated on a machine without Studio 5000, so these are schema-conformant
but not import-verified. Import the UDT file first and confirm the Import
Configuration dialog offers to *Create* all five VisionRobot types; then the tags;
then the routines. Report any schema error and I'll fix the generator.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple
from xml.sax.saxutils import escape

Rung = Tuple[str, str]  # (comment, neutral text)

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
         f"RobotTags.L5X (Move{m}_Execute/ons/Fault/InPosition, Move{m}_Steps, "
         f"Move{m}_Target_Deg, EM806_{m}_ALM, STEPS_PER_DEG, MOVE_VEL, MOVE_ACC) "
         f"and the ClearLink module (ClearLink:O1/:I1). Import VisionRobot_UDT.L5X "
         f"+ RobotTags.L5X first. Twin routine: R_MoveMotor{other}.",
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
         f"HLFB Inversion bit3=1). Tags come from RobotTags.L5X (Home{m}_State, "
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
     "R_HomeMotor1 and publishes VisionRobot.Status. Tags from RobotTags.L5X "
     "(HomeStep, HR_ons, SoftLimitsEnable, Home0_Req/Home1_Req, Ax0/Ax1_HomeDone/"
     "HomeFault, HOME_OFFSET_L/HOME_OFFSET_R, STEPS_PER_DEG). Import "
     "VisionRobot_UDT.L5X + RobotTags.L5X first.",
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
# DataType export — the VisionRobot UDT (BOOLs bit-packed into hidden SINTs)
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


def _datatype(name: str, members: List[Member], use: str) -> str:
    return (
        f'<DataType Use="{use}" Name="{name}" Family="NoFamily" Class="User">\n'
        f'<Members>\n{_pack_members(name, members)}\n</Members>\n'
        f'</DataType>'
    )


def visionrobot_udt_l5x() -> str:
    # leaves as context (declared first), the parent as the single target
    leaves = [
        ("VisionRobot_Cmd", UDT_CMD),
        ("VisionRobot_Target", UDT_TARGET),
        ("VisionRobot_Manual", UDT_MANUAL),
        ("VisionRobot_Status", UDT_STATUS),
    ]
    body = "\n".join(_datatype(n, m, "Context") for n, m in leaves)
    parent = _datatype("VisionRobot", UDT_PARENT, "Target")
    return f'''{_content_open("VisionRobot", "DataType")}
<Controller Use="Context" Name="{CONTROLLER}">
<DataTypes Use="Context">
{body}
{parent}
</DataTypes>
</Controller>
</RSLogix5000Content>
'''


# --------------------------------------------------------------------------- #
# Tag export — glue tags, tuning values, and the VisionRobot tag
# --------------------------------------------------------------------------- #
class Tag:
    def __init__(self, name: str, dtype: str, value="0", *,
                 constant=False, desc: str = "") -> None:
        self.name, self.dtype, self.value = name, dtype, value
        self.constant, self.desc = constant, desc

    def xml(self, use: str) -> str:
        const = "true" if self.constant else "false"
        radix = {"REAL": "Float"}.get(self.dtype, "Decimal")
        # structure tags (UDT, TIMER) carry no Radix on the <Tag> element
        structure = self.dtype in ("TIMER", "VisionRobot")
        open_attrs = (f'Use="{use}" Name="{self.name}" TagType="Base" '
                      f'DataType="{self.dtype}"')
        if not structure:
            open_attrs += f' Radix="{radix}"'
        open_attrs += f' Constant="{const}" ExternalAccess="Read/Write"'
        desc = (f"<Description><![CDATA[{self.desc}]]></Description>\n"
                if self.desc else "")

        if self.dtype == "VisionRobot":
            # minimal form — Logix materializes the structure with defaults
            if not desc:
                return f"<Tag {open_attrs}/>"
            data = ""
        elif self.dtype == "TIMER":
            data = (
                '<Data Format="Decorated">\n<Structure DataType="TIMER">\n'
                '<DataValueMember Name="PRE" DataType="DINT" Radix="Decimal" Value="0"/>\n'
                '<DataValueMember Name="ACC" DataType="DINT" Radix="Decimal" Value="0"/>\n'
                '<DataValueMember Name="EN" DataType="BOOL" Value="0"/>\n'
                '<DataValueMember Name="TT" DataType="BOOL" Value="0"/>\n'
                '<DataValueMember Name="DN" DataType="BOOL" Value="0"/>\n'
                '</Structure>\n</Data>\n'
            )
        else:
            data = (f'<Data Format="Decorated">\n<DataValue DataType="{self.dtype}" '
                    f'Radix="{radix}" Value="{self.value}"/>\n</Data>\n')
        return f"<Tag {open_attrs}>\n{desc}{data}</Tag>"


def _glue_tags() -> List[Tag]:
    tags: List[Tag] = []
    # tuning + fixed values
    tags.append(Tag("STEPS_PER_DEG", "REAL", "26.66667", constant=True,
                    desc="3200 pulses/rev * 3:1 / 360. Fixed mechanical ratio."))
    tags.append(Tag("MOVE_VEL", "DINT", "20000",
                    desc="Absolute-move speed, steps/s (<= 500000). Tune."))
    tags.append(Tag("MOVE_ACC", "DINT", "100000",
                    desc="Absolute-move accel, steps/s^2. Tune."))
    tags.append(Tag("HOME_VEL_0", "DINT", "-2000",
                    desc="Motor 0 homing speed, steps/s, SIGNED toward the prox. "
                         "Tune sign+magnitude at commissioning."))
    tags.append(Tag("HOME_VEL_1", "DINT", "2000",
                    desc="Motor 1 homing speed, steps/s, SIGNED toward the prox. "
                         "The two shoulders often home in opposite directions."))
    tags.append(Tag("HOME_ACC", "DINT", "50000", desc="Homing accel, steps/s^2."))
    tags.append(Tag("HOME_OFFSET_L", "DINT", "0",
                    desc="Left switch angle * STEPS_PER_DEG so ActualLeftDeg reads "
                         "~135.85 after homing. Set at commissioning."))
    tags.append(Tag("HOME_OFFSET_R", "DINT", "0",
                    desc="Right switch angle * STEPS_PER_DEG so ActualRightDeg "
                         "reads ~44.15 after homing. Set at commissioning."))
    # per-motor glue
    for m in (0, 1):
        tags += [
            Tag(f"Move{m}_Execute", "BOOL"),
            Tag(f"Move{m}_ons", "BOOL"),
            Tag(f"Move{m}_Fault", "BOOL"),
            Tag(f"Move{m}_InPosition", "BOOL"),
            Tag(f"Move{m}_Steps", "DINT"),
            Tag(f"Move{m}_Target_Deg", "REAL", "0.0"),
            Tag(f"Home{m}_Req", "BOOL"),
            Tag(f"Home{m}_ons", "BOOL"),
            Tag(f"Home{m}_State", "DINT"),
            Tag(f"Home{m}_Tmr", "TIMER"),
            Tag(f"Ax{m}_HomeDone", "BOOL"),
            Tag(f"Ax{m}_HomeFault", "BOOL"),
            Tag(f"EM806_{m}_ALM", "BOOL",
                desc=f"Motor {m} EM806 ALM. Alias this to the ClearLink digital "
                     f"input the drive alarm is wired to (Discrete Input Point)."),
        ]
    # coordinator scope
    tags += [
        Tag("HomeStep", "DINT"),
        Tag("HR_ons", "BOOL"),
        Tag("SoftLimitsEnable", "BOOL"),
        Tag("VisionRobot", "VisionRobot",
            desc="The vision-PC handshake surface (pycomm3 reads/writes by name). "
                 "Import VisionRobot_UDT.L5X first so this type resolves."),
    ]
    return tags


def robot_tags_l5x() -> str:
    tags = _glue_tags()
    # exactly one Use="Target"; the rest are Use="Context" (all get a Create
    # option in the Import Configuration dialog)
    body = "\n".join(
        t.xml("Target" if i == 0 else "Context") for i, t in enumerate(tags)
    )
    return f'''{_content_open(tags[0].name, "Tag")}
<Controller Use="Context" Name="{CONTROLLER}">
<Tags Use="Context">
{body}
</Tags>
</Controller>
</RSLogix5000Content>
'''


# --------------------------------------------------------------------------- #
def main() -> None:
    out = Path(__file__).resolve().parents[1] / "docs" / "l5x"
    out.mkdir(parents=True, exist_ok=True)

    # remove the old, misleadingly-named "AOI_*" routine files
    for stale in ("AOI_AxisMove.L5X", "AOI_HomeAxis.L5X"):
        p = out / stale
        if p.exists():
            p.unlink()
            print(f"removed stale {p.name}")

    files = {
        "VisionRobot_UDT.L5X": visionrobot_udt_l5x(),
        "RobotTags.L5X": robot_tags_l5x(),
        "R_MoveMotor0.L5X": routine_l5x("R_MoveMotor0", move_rungs(0)),
        "R_MoveMotor1.L5X": routine_l5x("R_MoveMotor1", move_rungs(1)),
        "R_HomeMotor0.L5X": routine_l5x("R_HomeMotor0", home_rungs(0)),
        "R_HomeMotor1.L5X": routine_l5x("R_HomeMotor1", home_rungs(1)),
        "R30_Homing.L5X": routine_l5x("R30_Homing", COORD),
    }
    for name, text in files.items():
        ET.fromstring(text)                      # well-formedness check
        (out / name).write_text(text)
        print(f"wrote {out / name}")


if __name__ == "__main__":
    main()
