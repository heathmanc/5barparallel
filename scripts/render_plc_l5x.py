#!/usr/bin/env python3
"""Emit importable Studio 5000 routine .L5X for the ClearLink motion logic.

Same logic as the ladder SVGs / Structured Text (docs/plc_*.md), expressed as
Rockwell **neutral rung text** — the exact form used inside Teknic's own
CompactLogix `.L5K` examples — wrapped in the L5X routine partial-export XML.
Import each with **Studio 5000 → right-click a Program → Import Routine…**

    docs/l5x/AOI_AxisMove.L5X    absolute move, Motor 0 (copy for Motor 1)
    docs/l5x/AOI_HomeAxis.L5X    ClearLink homing move, Motor 0 (copy for Motor 1)
    docs/l5x/R30_Homing.L5X      sequential 2-axis homing coordinator

These reference the ClearLink AOP tags (`ClearLink:O1/:I1/:C`, created when you
add the "Step Dir" module) plus the local/constant tags listed in each routine's
rung-0 comment. Undefined tags flag on import — create them per docs/plc_setup.md.

Run:  python scripts/render_plc_l5x.py

NOTE: generated on a machine without Studio 5000, so these are schema-conformant
but not import-verified — treat as v1, import once, and report any error.
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Tuple
from xml.sax.saxutils import escape

Rung = Tuple[str, str]  # (comment, neutral text)


def _rung(i: int, comment: str, text: str) -> str:
    return (
        f'<Rung Number="{i}" Type="N">\n'
        f'<Comment><![CDATA[{comment}]]></Comment>\n'
        f'<Text><![CDATA[{text}]]></Text>\n'
        f'</Rung>'
    )


def routine_l5x(name: str, rungs: List[Rung], program: str = "Robot") -> str:
    body = "\n".join(_rung(i, c, t) for i, (c, t) in enumerate(rungs))
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<RSLogix5000Content SchemaRevision="1.0" TargetName="{escape(name)}" TargetType="Routine" \
TargetSubType="RLL" ContainsContext="true" ExportOptions="References NoRawData L5KData \
DecoratedData Context Dependencies ForceProtectedEncoding AllProjDocTrans">
<Controller Use="Context" Name="RobotController">
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


# tag prefixes (Motor 0 = left shoulder; duplicate with Motor1_ for the right)
O = "ClearLink:O1.Motor0_"      # noqa: E741  output assembly
I = "ClearLink:I1.Motor0_"      # noqa: E741  input assembly


# --------------------------------------------------------------------------- #
# AOI_AxisMove — absolute move (mirrors SD_Position_Move), Motor 0
# --------------------------------------------------------------------------- #
AXISMOVE: List[Rung] = [
    ("Motor 0 absolute move. Create tags: Move0_Execute/Move0_ons/Move0_Fault/"
     "Move0_InPosition (BOOL), Move0_Steps (DINT), Move0_Target_Deg (REAL), "
     "EM806_0_ALM (BOOL, alias to the EM806 ALM digital input). Constants: "
     "STEPS_PER_DEG (REAL 26.66667), MOVE_VEL/MOVE_ACC (DINT). Copy this routine "
     "for Motor 1 (Motor0_ -> Motor1_).",
     f"OTE({O}Output_Reg_Enable);"),
    ("Convert the target angle to steps.",
     "CPT(Move0_Steps,TRUNC(Move0_Target_Deg * STEPS_PER_DEG));"),
    ("Rising edge of Execute: load Move Distance / limits, set Absolute, and "
     "latch Load Position Data.",
     f"XIC(Move0_Execute)ONS(Move0_ons)XIO(Move0_Fault)"
     f"MOV(Move0_Steps,{O}Move_Dist)MOV(MOVE_VEL,{O}Vel_Limit)"
     f"MOV(MOVE_ACC,{O}Accel_Lim)OTL({O}Output_Reg_Abs_Flag)"
     f"OTL({O}Output_Reg_Load_Posn_Data);"),
    ("ClearLink acknowledges the load -> drop Load Position Data.",
     f"XIC({I}Status_Load_Posn_Move_Ack)OTU({O}Output_Reg_Load_Posn_Data);"),
    ("Move done = At Target Position (needs HLFB Inversion for the EM806).",
     f"XIC({I}Status_At_Target_Posn)XIO({O}Output_Reg_Load_Posn_Data)"
     f"OTE(Move0_InPosition);"),
    ("Motor fault, ClearLink shutdown, or EM806 alarm -> Fault.",
     f"[XIC({I}Status_Motor_In_Fault),XIC({I}Status_Shutdowns_Pres),"
     f"XIC(EM806_0_ALM)]OTE(Move0_Fault);"),
]

# --------------------------------------------------------------------------- #
# AOI_HomeAxis — ClearLink homing move (mirrors SD_Homing), Motor 0
# --------------------------------------------------------------------------- #
HOMEAXIS: List[Rung] = [
    ("Motor 0 homing. First set the Configuration assembly ClearLink:C.Motor0Config: "
     "Home_Sensor = prox input, Config Register Homing Enable (bit0)=1, HLFB "
     "Inversion (bit3)=1. Create tags: Home0_State (DINT), Home0_Req/Home0_ons/"
     "Ax0_HomeDone/Ax0_HomeFault (BOOL), Home0_Tmr (TIMER). Constants: HOME_VEL "
     "(signed, toward the prox) / HOME_ACC (DINT). Copy for Motor 1.",
     "XIC(Home0_Req)ONS(Home0_ons)EQU(Home0_State,0)XIO(Ax0_HomeFault)"
     "MOV(10,Home0_State);"),
    ("State 10 ENABLING: hold the Enable output.",
     f"EQU(Home0_State,10)OTE({O}Output_Reg_Enable);"),
    ("State 10: advance once HLFB is asserted.",
     f"EQU(Home0_State,10)XIC({I}Status_HLFB_ON)MOV(20,Home0_State);"),
    ("State 20 CLEAR MOTOR FAULTS.",
     f"EQU(Home0_State,20)OTE({O}Output_Reg_Clear_Fault);"),
    ("State 20: on Clear-Fault ack, drop the request and advance.",
     f"EQU(Home0_State,20)XIC({I}Status_Clear_Motor_Fault_Ack)"
     f"OTU({O}Output_Reg_Clear_Fault)MOV(30,Home0_State);"),
    ("State 30 CLEAR ALERTS.",
     f"EQU(Home0_State,30)OTE({O}Output_Reg_Clear_Alerts);"),
    ("State 30: when no shutdowns remain, drop the request and advance.",
     f"EQU(Home0_State,30)XIO({I}Status_Shutdowns_Pres)"
     f"OTU({O}Output_Reg_Clear_Alerts)MOV(40,Home0_State);"),
    ("State 40: wait for Ready To Home.",
     f"EQU(Home0_State,40)XIC({I}Status_Ready_To_Home)MOV(50,Home0_State);"),
    ("State 50 BEGIN HOMING MOVE: home flag + velocity move toward the prox.",
     f"EQU(Home0_State,50)OTL({O}Output_Reg_Home_Flag)"
     f"MOV(HOME_VEL,{O}Jog_Vel)MOV(HOME_ACC,{O}Accel_Lim)"
     f"OTL({O}Output_Reg_Load_Vel_Data)MOV(55,Home0_State);"),
    ("State 55: on Load-Velocity ack, drop the load bit and homing flag.",
     f"EQU(Home0_State,55)XIC({I}Status_Load_Vel_Move_Ack)"
     f"OTU({O}Output_Reg_Load_Vel_Data)OTU({O}Output_Reg_Home_Flag)"
     f"MOV(60,Home0_State);"),
    ("State 60 HOMING: ClearLink zeroes at the prox -> Has Homed.",
     f"EQU(Home0_State,60)XIC({I}Status_Has_Homed)OTL(Ax0_HomeDone)"
     f"MOV(70,Home0_State);"),
    ("Homing timeout (timer runs while homing) -> fault.",
     "TONR(Home0_Tmr)XIC(Home0_Tmr.DN)OTL(Ax0_HomeFault)MOV(900,Home0_State);"),
    ("EM806 alarm or ClearLink shutdown -> fault.",
     f"[XIC(EM806_0_ALM),XIC({I}Status_Shutdowns_Pres)]"
     f"OTL(Ax0_HomeFault)MOV(900,Home0_State);"),
    ("State 900 FAULT: clear the move bits, hold for reset.",
     f"EQU(Home0_State,900)OTU({O}Output_Reg_Home_Flag)"
     f"OTU({O}Output_Reg_Load_Vel_Data);"),
]

# --------------------------------------------------------------------------- #
# R30_Homing — sequential two-axis coordinator
# --------------------------------------------------------------------------- #
COORD: List[Rung] = [
    ("Homing coordinator. Runs the two per-axis homing routines (JSR) and "
     "publishes VisionRobot.Status. Create: HomeStep (DINT), HR_ons (BOOL), "
     "SoftLimitsEnable (BOOL), Home0_Req/Home1_Req (BOOL). Constants: "
     "STEPS_PER_DEG (REAL), HOME_OFFSET_L/HOME_OFFSET_R (DINT = switch angle x "
     "STEPS_PER_DEG). Requires routines R_HomeMotor0 / R_HomeMotor1 (this file's "
     "AOI_HomeAxis, one per motor).",
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


def main() -> None:
    out = Path(__file__).resolve().parents[1] / "docs" / "l5x"
    out.mkdir(parents=True, exist_ok=True)
    for name, rungs in (("AOI_AxisMove", AXISMOVE),
                        ("AOI_HomeAxis", HOMEAXIS),
                        ("R30_Homing", COORD)):
        text = routine_l5x(name, rungs)
        # well-formedness check
        import xml.etree.ElementTree as ET
        ET.fromstring(text)
        path = out / f"{name}.L5X"
        path.write_text(text)
        print(f"wrote {path}  ({len(rungs)} rungs)")


if __name__ == "__main__":
    main()
