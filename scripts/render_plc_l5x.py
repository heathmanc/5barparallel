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

    docs/l5x/R00_Main.L5X       scan dispatcher (set as the Program's Main) \\
    docs/l5x/R10_Safety.L5X     E-stop / limits / drive-alarm -> faults      |
    docs/l5x/R20_Drives.L5X     owns the axis Enable outputs                 |  right-
    docs/l5x/R30_Homing.L5X     2-axis homing coordinator                    |  click a
    docs/l5x/R40_Manual.L5X     manual jog/home surface                      |  Program
    docs/l5x/R50_Auto.L5X       automatic pick/place state machine           |  -> Import
    docs/l5x/R60_Status.L5X     publishes the derived status bits            |  Routine…
    docs/l5x/R_MoveMotor0.L5X   absolute move engine, Motor 0                |  (LAST,
    docs/l5x/R_MoveMotor1.L5X   absolute move engine, Motor 1                |  after the
    docs/l5x/R_HomeMotor0.L5X   ClearLink homing, Motor 0 (JSR by R30)       |  UDT +
    docs/l5x/R_HomeMotor1.L5X   ClearLink homing, Motor 1 (JSR by R30)       /  tags)

R00_Main JSRs the others each scan (Manual vs Auto by the AutoMode tag), so this
is a complete program, not just the motion pieces. These command real motion, the
Z cylinder and vacuum — REVIEW before running; the hardware E-stop safety relay
is primary and the PLC bits only mirror it.

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
    offs = "HOME_OFFSET_L" if m == 0 else "HOME_OFFSET_R"
    other = 1 - m
    return [
        (f"R_MoveMotor{m}: Motor {m} absolute move. R20_Drives owns the axis Enable "
         f"output (this routine no longer sets it). Called each scan by R00_Main. "
         f"Tags/constants from RobotTags.csv (Move{m}_Execute/Fault/InPosition/Loaded, "
         f"Move{m}_Steps, Move{m}_Target_Deg, EM806_{m}_ALM, STEPS_PER_DEG, MOVE_VEL, "
         f"MOVE_ACC, {offs}) and the ClearLink module. Convert the target angle to "
         f"ClearLink steps. Subtract {offs} because the ClearLink zeroes CommandedPosn "
         f"at the home prox, not at 0 deg: R30 publishes ActualDeg = (CommandedPosn + "
         f"{offs})/STEPS_PER_DEG, so the inverse (command) must be Target*SPD - {offs}. "
         f"Without it, the first move after homing jumps by the whole home offset.",
         f"CPT(Move{m}_Steps,TRN(Move{m}_Target_Deg * STEPS_PER_DEG) - {offs});"),
        ("Load the move whenever Execute is commanded but not yet loaded or done "
         "(level-triggered, NOT an Execute edge): a new command clears Loaded + "
         "InPosition in R40/R50, so this loads once, then XIO(Loaded)/XIO(InPosition) "
         "hold it off. Level-triggering avoids the deadlock where a sticky Execute "
         "latch (move never completed) leaves a one-shot that can never re-fire. "
         "Gated by the motor-local fault AND the latched controller fault so a "
         "code-4 homing failure inhibits the move until Reset clears it. Loads Move "
         "Distance / limits, sets Absolute, latches Load Position Data + Loaded.",
         f"XIC(Move{m}_Execute)XIO(Move{m}_Loaded)XIO(Move{m}_InPosition)"
         f"XIO(Move{m}_Fault)XIO(VisionRobot.Status.Faulted)"
         f"MOV(Move{m}_Steps,{o}Move_Dist)MOV(MOVE_VEL,{o}Vel_Limit)"
         f"MOV(MOVE_ACC,{o}Accel_Lim)OTL({o}Output_Reg_Abs_Flag)"
         f"OTL({o}Output_Reg_Load_Posn_Data)OTL(Move{m}_Loaded);"),
        ("ClearLink acknowledges the load -> drop Load Position Data.",
         f"XIC({i}Status_Load_Posn_Move_Ack)OTU({o}Output_Reg_Load_Posn_Data);"),
        ("Move done = a loaded move reached At Target Position (Loaded gate stops a "
         "stale/at-rest At_Target_Posn from reading as done). Latch InPosition, clear "
         "Loaded. Needs HLFB Inversion for the EM806.",
         f"XIC(Move{m}_Loaded)XIC({i}Status_At_Target_Posn)XIO({o}Output_Reg_Load_Posn_Data)"
         f"OTL(Move{m}_InPosition)OTU(Move{m}_Loaded);"),
        ("Motor fault, ClearLink shutdown, or EM806 alarm -> Fault.",
         f"[XIC({i}Status_Motor_In_Fault),XIC({i}Status_Shutdowns_Pres),"
         f"XIC(EM806_{m}_ALM)]OTE(Move{m}_Fault);"),
        ("Abort: on a motor-local fault or the latched controller fault, drop any "
         "in-flight load so a faulted axis cannot keep or re-issue a move. Reset "
         "(R10) clears Status.Faulted; the operator re-Executes to resume.",
         f"[XIC(Move{m}_Fault),XIC(VisionRobot.Status.Faulted)]"
         f"OTU({o}Output_Reg_Load_Posn_Data)OTU(Move{m}_Loaded);"),
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
         f"HOME_VEL_{m} signed toward the prox, HOME_ACC, HOME_TMO_MS). R30_Homing JSRs "
         f"R_HomeMotor0/R_HomeMotor1.",
         f"XIC(Home{m}_Req)ONS(Home{m}_ons)EQU(Home{m}_State,0)XIO(Ax{m}_HomeFault)"
         f"OTU(Home{m}_Moved)MOV(10,Home{m}_State);"),
        ("State 10 ENABLING: advance once the axis reports Enabled (Status bit 10). "
         "NOT HLFB_ON (bit 14): a no-HLFB step/dir drive (EM806) never asserts "
         "HLFB_ON even when fully enabled, so gating on it hangs homing at state 10 "
         "forever. R20_Drives holds the Enable; Enabled is the real enable-complete.",
         f"EQU(Home{m}_State,10)XIC({i}Status_Enabled)MOV(20,Home{m}_State);"),
        ("State 20 CLEAR MOTOR FAULTS. Also fires on a bench DriveClearReq so a "
         "latched Motor-Faulted shutdown can be cleared without homing (bypass "
         "never reaches this state otherwise). Single owner of this coil.",
         f"[EQU(Home{m}_State,20),XIC(DriveClearReq)]OTE({o}Output_Reg_Clear_Fault);"),
        ("State 20: on Clear-Fault ack, drop the request and advance.",
         f"EQU(Home{m}_State,20)XIC({i}Status_Clear_Motor_Fault_Ack)"
         f"OTU({o}Output_Reg_Clear_Fault)MOV(30,Home{m}_State);"),
        ("State 30 CLEAR ALERTS. Also fires on a bench DriveClearReq to clear the "
         "OR-accumulating Shutdown register without homing. Single owner of this coil.",
         f"[EQU(Home{m}_State,30),XIC(DriveClearReq)]OTE({o}Output_Reg_Clear_Alerts);"),
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
        ("Latch real motion: while the homing move runs (States 50-60), any "
         "Steps_Active sets Moved. Proof the axis actually stepped this attempt.",
         f"GEQ(Home{m}_State,50)LES(Home{m}_State,70)XIC({i}Status_Steps_Active)"
         f"OTL(Home{m}_Moved);"),
        ("State 60 HOMING: ClearLink zeroes at the prox -> Has Homed. Requires "
         "Moved so a stale/power-up Has_Homed can't complete a home with no motion; "
         "if the motor never steps, homing times out to FaultCode 4 instead.",
         f"EQU(Home{m}_State,60)XIC({i}Status_Has_Homed)XIC(Home{m}_Moved)"
         f"OTL(Ax{m}_HomeDone)MOV(70,Home{m}_State);"),
        ("Keep the timeout preset loaded (CSV imports Home_Tmr.PRE as 0, which "
         "would fault instantly). Set HOME_TMO_MS to the allowed homing time, ms.",
         f"MOV(HOME_TMO_MS,Home{m}_Tmr.PRE);"),
        ("Run the homing timeout while homing is active (States 10-60). TON is "
         "non-retentive, so it auto-resets when homing completes/faults/idles and "
         "each attempt starts fresh (use RTO + RES instead if you want retentive).",
         f"NEQ(Home{m}_State,0)LES(Home{m}_State,70)TON(Home{m}_Tmr,?,?);"),
        ("Homing timed out -> fault.",
         f"XIC(Home{m}_Tmr.DN)OTL(Ax{m}_HomeFault)MOV(900,Home{m}_State);"),
        ("EM806 alarm or ClearLink shutdown DURING the homing move (States 40-60, "
         "i.e. after clear-alerts) -> fault. Gated by state so a normal power-up "
         "Shutdowns_Pres doesn't latch a homing fault at idle.",
         f"GEQ(Home{m}_State,40)LES(Home{m}_State,70)"
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
     "XIC(VisionRobot.Status.Enabled)XIO(VisionRobot.Status.Faulted)XIO(Bypass_Homing)"
     "MOV(10,HomeStep)OTU(VisionRobot.Status.Homed)"
     "OTU(Ax0_HomeDone)OTU(Ax1_HomeDone)OTU(Ax0_HomeFault)OTU(Ax1_HomeFault)"
     "MOV(0,Home0_State)MOV(0,Home1_State)OTL(Home0_Req);"),
    ("BENCH BYPASS: with Bypass_Homing set, HomeRequest marks referenced instantly "
     "(publishes the nominal home angles, enables soft limits) without running the "
     "ClearLink prox homing move. Lets you jog motors on the bench with no prox.",
     "XIC(VisionRobot.Manual.HomeRequest)ONS(HRB_ons)XIC(Bypass_Homing)"
     "XIC(VisionRobot.Status.Enabled)XIO(VisionRobot.Status.Faulted)"
     "MOV(HOME_ANGLE_L,VisionRobot.Status.ActualLeftDeg)"
     "MOV(HOME_ANGLE_R,VisionRobot.Status.ActualRightDeg)"
     "OTL(VisionRobot.Status.Homed)OTL(SoftLimitsEnable)MOV(0,HomeStep);"),
    ("Axis 0 (left) homed -> start Axis 1 (right).",
     "EQU(HomeStep,10)XIC(Ax0_HomeDone)OTU(Home0_Req)OTL(Home1_Req)"
     "MOV(20,HomeStep);"),
    ("Axis 1 (right) homed.",
     "EQU(HomeStep,20)XIC(Ax1_HomeDone)OTU(Home1_Req)MOV(30,HomeStep);"),
    ("Publish angles with the home offset, latch soft limits on, return to idle.",
     "EQU(HomeStep,30)"
     "CPT(VisionRobot.Status.ActualLeftDeg,"
     "(ClearLink:I1.Motor0_CommandedPosn + HOME_OFFSET_L) / STEPS_PER_DEG)"
     "CPT(VisionRobot.Status.ActualRightDeg,"
     "(ClearLink:I1.Motor1_CommandedPosn + HOME_OFFSET_R) / STEPS_PER_DEG)"
     "OTL(SoftLimitsEnable)OTL(VisionRobot.Status.Homed)MOV(0,HomeStep);"),
    ("Run the per-axis homing routines every scan (they self-idle at rest).",
     "JSR(R_HomeMotor0,0)JSR(R_HomeMotor1,0);"),
    ("Disable invalidates the reference. An open-loop stepper has no feedback, so "
     "the instant the drive disables the datum is gone (the shaft can back-drive / "
     "lose steps unseen). Drop Homed + per-axis HomeDone + soft limits whenever the "
     "drive is not commanded on, forcing a re-home after any disable. Keyed on "
     "EnableReq (the commanded enable), NOT Status.Enabled: the latter is the "
     "ClearLink's HLFB-derived bit 10, which can momentarily dip as a move stops or "
     "during a Clear-Fault enable cycle - keying on it would spuriously un-home "
     "between jogs. The homing state machines re-zero on the next HomeRequest.",
     "XIO(EnableReq)OTU(VisionRobot.Status.Homed)"
     "OTU(Ax0_HomeDone)OTU(Ax1_HomeDone)OTU(SoftLimitsEnable);"),
    ("Drive power-cycle invalidates the reference AND drops the enable, even while "
     "still commanded on. The ClearLink's Has_Homed (bit 13) de-asserts the instant "
     "HLFB drops (drive powered down) - dip-sensitive, so it catches even a quick "
     "unplug that a level like Status.Enabled misses. If we think we're homed (real "
     "homing, not bypass) but either axis dropped Has_Homed: clear the reference "
     "(force a re-home) AND drop Manual.Enable so the drive powers back up DISABLED, "
     "not holding torque. Gated by Status.Homed so a normal re-home (which itself "
     "clears Has_Homed) can't trip it. Excluded in bypass.",
     "XIC(VisionRobot.Status.Homed)XIO(Bypass_Homing)"
     "[XIO(ClearLink:I1.Motor0_Status_Has_Homed),"
     "XIO(ClearLink:I1.Motor1_Status_Has_Homed)]"
     "OTU(VisionRobot.Status.Homed)OTU(Ax0_HomeDone)OTU(Ax1_HomeDone)"
     "OTU(SoftLimitsEnable)OTU(VisionRobot.Manual.Enable);"),
    ("Cmd.Reset (rising edge) while safe restores a fresh, re-homeable state: "
     "clear the coordinator + per-axis states and latched home faults/requests. "
     "MUST scan before the fault-latch rung below so the cleared HomeFault can't "
     "immediately re-latch the fault in the same scan.",
     "XIC(VisionRobot.Cmd.Reset)ONS(HomeRst_ons)XIC(EStop_OK)XIC(Guard_Closed)"
     "MOV(0,HomeStep)MOV(0,Home0_State)MOV(0,Home1_State)"
     "OTU(Ax0_HomeFault)OTU(Ax1_HomeFault)OTU(Home0_Req)OTU(Home1_Req);"),
    ("Either axis homing fault -> homing fault (FaultCode 4).",
     "[XIC(Ax0_HomeFault),XIC(Ax1_HomeFault)]OTL(VisionRobot.Status.Faulted)"
     "MOV(4,VisionRobot.Status.FaultCode)MOV(900,HomeStep);"),
]

# --------------------------------------------------------------------------- #
# The program that ties it together — R00_Main dispatcher + R10/R20/R40/R50/R60.
# Flat neutral text using the RobotTags.csv glue tags. These command real motion,
# the Z cylinder, and vacuum — REVIEW before running; the hardware E-stop safety
# relay is primary and these PLC bits only mirror it.
# --------------------------------------------------------------------------- #
MAIN: List[Rung] = [
    ("R00_Main: scan dispatcher — calls every routine in order each scan. Manual "
     "and Auto are mutually exclusive on AutoMode (RobotTags.csv). Put this routine "
     "as the Program's Main; import all the other routines first.",
     "JSR(R10_Safety,0)JSR(R20_Drives,0)JSR(R30_Homing,0);"),
    ("Auto mode runs the pick/place sequence; manual mode runs the jog/home surface.",
     "XIC(AutoMode)JSR(R50_Auto,0);"),
    ("(Manual when not in auto.)",
     "XIO(AutoMode)JSR(R40_Manual,0);"),
    ("Service the per-axis move engines after the commanding routine, then publish "
     "status.",
     "JSR(R_MoveMotor0,0)JSR(R_MoveMotor1,0)JSR(R60_Status,0);"),
]

SAFETY: List[Rung] = [
    ("R10_Safety: E-stop / guard / hard limits / drive alarm -> Status.Faulted + "
     "FaultCode; compute SafetyOK; Cmd.Reset clears a latched fault when safe. Tags "
     "(RobotTags.csv): EStop_Pressed/EStop_OK/Guard_Closed/Ax*_LimitMin/Max/"
     "EM806_*_ALM/SafetyOK/Reset_prev. The hardware safety relay is primary; these "
     "bits only mirror it. E-stop or guard open -> fault (code 2).",
     "[XIC(EStop_Pressed),XIO(Guard_Closed)]OTL(VisionRobot.Status.Faulted)"
     "MOV(2,VisionRobot.Status.FaultCode);"),
    ("Any hard limit tripped -> fault (code 3).",
     "[XIC(Ax0_LimitMin),XIC(Ax0_LimitMax),XIC(Ax1_LimitMin),XIC(Ax1_LimitMax)]"
     "OTL(VisionRobot.Status.Faulted)MOV(3,VisionRobot.Status.FaultCode);"),
    ("Either EM806 drive alarm -> fault (code 1). The EM806 ALM output is wired to "
     "the ClearLink HLFB input, so a drive alarm OR a drive power-loss de-asserts "
     "HLFB and shows up as Motor_In_Fault (Status bit 9 = HLFB de-asserted AND enable "
     "asserted) - NOT as a standalone EM806_x_ALM DI. Fault on both. Bit 9 is "
     "dip-sensitive and independent of homed state, so this catches a drive "
     "power-cycle while enabled whether or not the machine was homed; the fault then "
     "drops Manual.Enable (anti-restart) and the reference (via EnableReq).",
     "[XIC(EM806_0_ALM),XIC(EM806_1_ALM),"
     "XIC(ClearLink:I1.Motor0_Status_Motor_In_Fault),"
     "XIC(ClearLink:I1.Motor1_Status_Motor_In_Fault)]"
     "OTL(VisionRobot.Status.Faulted)MOV(1,VisionRobot.Status.FaultCode);"),
    ("SafetyOK = no active fault, E-stop healthy, guard closed.",
     "XIO(VisionRobot.Status.Faulted)XIC(EStop_OK)XIC(Guard_Closed)OTE(SafetyOK);"),
    ("Cmd.Reset (rising edge) while physically safe clears the latched fault.",
     "XIC(VisionRobot.Cmd.Reset)ONS(Reset_prev)XIC(EStop_OK)XIC(Guard_Closed)"
     "OTU(VisionRobot.Status.Faulted)MOV(0,VisionRobot.Status.FaultCode);"),
    # --- PC<->PLC heartbeat watchdog ---
    ("Watchdog: keep the PC-heartbeat timer preset loaded (HB_TIMEOUT_MS ms).",
     "MOV(HB_TIMEOUT_MS,HB_Tmr.PRE);"),
    ("PC heartbeat changed -> HB_seen pulses one scan (which resets the watchdog "
     "timer) and we latch the new value. The PC (PlcRobotDriver) increments "
     "VisionRobot.Cmd.Heartbeat continuously while connected.",
     "NEQ(VisionRobot.Cmd.Heartbeat,HB_last)OTE(HB_seen)"
     "MOV(VisionRobot.Cmd.Heartbeat,HB_last);"),
    ("Watchdog timer runs while NO new heartbeat; each change resets it. Expiry "
     "= the PC stopped talking.",
     "XIO(HB_seen)TON(HB_Tmr,?,?);"),
    ("PcAlive = a PC heartbeat arrived within HB_TIMEOUT_MS. R20 gates the drive "
     "Enable on this, so the drives can't be (or stay) enabled without a live app.",
     "XIO(HB_Tmr.DN)OTE(VisionRobot.Status.PcAlive);"),
    ("Heartbeat lost while the operator has Enable requested -> comms-loss fault "
     "(code 10). Dead-man: a crashed/closed app drops the drives via R20 losing "
     "PcAlive, and latches this fault. Clears on Reset once the heartbeat returns.",
     "XIC(HB_Tmr.DN)XIC(VisionRobot.Manual.Enable)OTL(VisionRobot.Status.Faulted)"
     "MOV(10,VisionRobot.Status.FaultCode);"),
    ("PLC heartbeat: increment each scan so the PC can confirm the ladder is "
     "actually scanning (not just that the tag read succeeded).",
     "ADD(VisionRobot.Status.Heartbeat,1,VisionRobot.Status.Heartbeat);"),
    ("Wrap the PLC heartbeat to avoid DINT overflow.",
     "GRT(VisionRobot.Status.Heartbeat,1000000000)"
     "MOV(0,VisionRobot.Status.Heartbeat);"),
]

DRIVES: List[Rung] = [
    ("Anti-restart: a latched fault DROPS the operator's Enable request. Enable is "
     "a maintained level; without this, hitting Reset (which clears Faulted) would "
     "instantly re-satisfy EnableReq and auto-energize the drives - an auto-restart "
     "hazard. Clearing it here forces a deliberate Enable AFTER Reset. Done by the "
     "PLC (not the app) so it still works when the app died (comms loss).",
     "XIC(VisionRobot.Status.Faulted)OTU(VisionRobot.Manual.Enable);"),
    ("R20_Drives: owns the axis Enable outputs. EnableReq mirrors Manual.Enable "
     "gated by SafetyOK, no fault, AND a live PC heartbeat (Status.PcAlive) so the "
     "drives cannot be enabled or stay enabled without the app talking. Drives both "
     "ClearLink Enable outputs and publishes Status.Enabled. Tag: EnableReq.",
     "XIC(VisionRobot.Manual.Enable)XIC(SafetyOK)XIO(VisionRobot.Status.Faulted)"
     "XIC(VisionRobot.Status.PcAlive)OTE(EnableReq);"),
    ("Drive Motor 0 enable.",
     "XIC(EnableReq)OTE(ClearLink:O1.Motor0_Output_Reg_Enable);"),
    ("Drive Motor 1 enable.",
     "XIC(EnableReq)OTE(ClearLink:O1.Motor1_Output_Reg_Enable);"),
    ("Enabled when requested and both drives report Enabled.",
     "XIC(EnableReq)XIC(ClearLink:I1.Motor0_Status_Enabled)"
     "XIC(ClearLink:I1.Motor1_Status_Enabled)OTE(VisionRobot.Status.Enabled);"),
    ("Drop-out debounce preset (EN_DROP_TMO_MS).",
     "MOV(EN_DROP_TMO_MS,EnDrop_Tmr.PRE);"),
    ("Time how long the drive is commanded on (EnableReq) but not actually Enabled. "
     "Resets the instant it enables, so the enable startup / a Clear-Fault enable "
     "cycle / an HLFB blip don't trip it - only a sustained drop-out does.",
     "XIC(EnableReq)XIO(VisionRobot.Status.Enabled)TON(EnDrop_Tmr,?,?);"),
    ("Sustained drop-out (a drive power-cycle: commanded on but dark past the "
     "debounce) -> drop the operator's Enable request so the drive does NOT come "
     "back energized when it powers up. Re-enabling is then a deliberate Enable.",
     "XIC(EnDrop_Tmr.DN)OTU(VisionRobot.Manual.Enable);"),
]

MANUAL: List[Rung] = [
    ("R40_Manual: absolute jog on Manual.MoveToTarget (rising edge), gated by "
     "Enabled + Homed + soft limits; drives Move0/Move1; publishes InPosition + "
     "CompleteCommandID. (Homing is Manual.HomeRequest -> R30_Homing.) Tags "
     "(RobotTags.csv): WithinLimits/MoveActive/MTT_prev. Soft-limit check first.",
     "GEQ(VisionRobot.Manual.TargetLeftDeg,-20.0)LEQ(VisionRobot.Manual.TargetLeftDeg,200.0)"
     "GEQ(VisionRobot.Manual.TargetRightDeg,-20.0)LEQ(VisionRobot.Manual.TargetRightDeg,200.0)"
     "OTE(WithinLimits);"),
    ("Accept a move on the rising edge when enabled, homed and within limits: latch "
     "targets + CommandID and start both axes.",
     "XIC(VisionRobot.Manual.MoveToTarget)ONS(MTT_prev)XIC(VisionRobot.Status.Enabled)"
     "XIC(VisionRobot.Status.Homed)XIC(WithinLimits)"
     "MOV(VisionRobot.Manual.TargetLeftDeg,Move0_Target_Deg)"
     "MOV(VisionRobot.Manual.TargetRightDeg,Move1_Target_Deg)"
     "MOV(VisionRobot.Manual.CommandID,VisionRobot.Status.ActiveCommandID)"
     "OTU(VisionRobot.Status.InPosition)OTU(Move0_InPosition)OTU(Move1_InPosition)"
     "OTU(Move0_Loaded)OTU(Move1_Loaded)"
     "OTL(Move0_Execute)OTL(Move1_Execute)OTL(MoveActive);"),
    ("Move requested while not enabled -> fault (code 5).",
     "XIC(VisionRobot.Manual.MoveToTarget)XIO(VisionRobot.Status.Enabled)"
     "OTL(VisionRobot.Status.Faulted)MOV(5,VisionRobot.Status.FaultCode);"),
    ("Move requested while not homed -> fault (code 6).",
     "XIC(VisionRobot.Manual.MoveToTarget)XIC(VisionRobot.Status.Enabled)"
     "XIO(VisionRobot.Status.Homed)OTL(VisionRobot.Status.Faulted)"
     "MOV(6,VisionRobot.Status.FaultCode);"),
    ("Both axes in position -> publish InPosition + CompleteCommandID, drop execute.",
     "XIC(MoveActive)XIC(Move0_InPosition)XIC(Move1_InPosition)"
     "MOV(VisionRobot.Status.ActiveCommandID,VisionRobot.Status.CompleteCommandID)"
     "OTL(VisionRobot.Status.InPosition)OTU(Move0_Execute)OTU(Move1_Execute)OTU(MoveActive);"),
    ("Manual.Abort or a fault stops the move.",
     "[XIC(VisionRobot.Manual.Abort),XIC(VisionRobot.Status.Faulted)]"
     "OTU(Move0_Execute)OTU(Move1_Execute)OTU(MoveActive);"),
]

AUTO: List[Rung] = [
    ("R50_Auto: automatic pick/place state machine (Claude.md §11). Drives "
     "Move0/Move1 to the camera-clear/pick/drop poses, the Z cylinder, vacuum and "
     "blowoff; advances on status bits (timers only for vacuum settle + blowoff). "
     "Tags (RobotTags.csv): State/RPP_prev/AR_prev/VacTmr/BlowTmr/Arrived/poses/"
     "pneumatics/sensors. Keep the timer presets loaded from the constants.",
     "MOV(VAC_SETTLE,VacTmr.PRE)MOV(BLOWOFF_TIME,BlowTmr.PRE);"),
    ("Arrived (status/HMI only) = both axes in position. Transitions check the two "
     "InPosition bits directly so a stale value can't fall through.",
     "XIC(Move0_InPosition)XIC(Move1_InPosition)OTE(Arrived);"),
    ("Vacuum settle timer runs in the vacuum states.",
     "[EQU(State,90),EQU(State,100)]TON(VacTmr,?,?);"),
    ("Blowoff timer runs in the blowoff state.",
     "EQU(State,170)TON(BlowTmr,?,?);"),
    ("State 0 IDLE: on Cmd.RequestPickPlace (edge) while enabled, homed and safe, "
     "latch the CommandID and start.",
     "EQU(State,0)XIC(VisionRobot.Cmd.RequestPickPlace)ONS(RPP_prev)"
     "XIC(VisionRobot.Status.Enabled)XIC(VisionRobot.Status.Homed)XIC(SafetyOK)"
     "MOV(VisionRobot.Cmd.CommandID,VisionRobot.Status.ActiveCommandID)"
     "OTU(VisionRobot.Status.Done)MOV(10,State);"),
    ("State 10 MOVE_CAMERA_CLEAR: command both axes to the camera-clear pose "
     "(clear stale InPosition so the wait state can't fall through).",
     "EQU(State,10)MOV(CAMERA_CLEAR_L,Move0_Target_Deg)MOV(CAMERA_CLEAR_R,Move1_Target_Deg)"
     "OTU(Move0_InPosition)OTU(Move1_InPosition)OTU(Move0_Loaded)OTU(Move1_Loaded)OTL(Move0_Execute)OTL(Move1_Execute)MOV(20,State);"),
    ("State 20: both axes in position at camera-clear -> drop execute, mark CameraClear.",
     "EQU(State,20)XIC(Move0_InPosition)XIC(Move1_InPosition)OTU(Move0_Execute)OTU(Move1_Execute)"
     "OTL(VisionRobot.Status.CameraClear)MOV(30,State);"),
    ("State 30 READY_FOR_VISION: signal the PC (targets already written by it).",
     "EQU(State,30)OTL(VisionRobot.Status.ReadyForVision)MOV(40,State);"),
    ("State 40 LATCH_TARGETS: copy the PC's pick/drop angles into working tags.",
     "EQU(State,40)MOV(VisionRobot.Target.Pick_LeftDeg,PickL)"
     "MOV(VisionRobot.Target.Pick_RightDeg,PickR)MOV(VisionRobot.Target.Drop_LeftDeg,DropL)"
     "MOV(VisionRobot.Target.Drop_RightDeg,DropR)OTU(VisionRobot.Status.ReadyForVision)"
     "MOV(50,State);"),
    ("State 50 MOVE_ABOVE_PICK.",
     "EQU(State,50)MOV(PickL,Move0_Target_Deg)MOV(PickR,Move1_Target_Deg)"
     "OTU(Move0_InPosition)OTU(Move1_InPosition)OTU(Move0_Loaded)OTU(Move1_Loaded)OTL(Move0_Execute)OTL(Move1_Execute)MOV(60,State);"),
    ("State 60: at pick -> drop execute.",
     "EQU(State,60)XIC(Move0_InPosition)XIC(Move1_InPosition)OTU(Move0_Execute)OTU(Move1_Execute)MOV(70,State);"),
    ("State 70 CYLINDER_DOWN_PICK.",
     "EQU(State,70)OTL(CylinderDown)MOV(80,State);"),
    ("State 80: Z down at pick confirmed (or Bypass_Vision on the bench).",
     "EQU(State,80)[XIC(PickDown),XIC(Bypass_Vision)]MOV(90,State);"),
    ("State 90 VACUUM_ON.",
     "EQU(State,90)OTL(VacuumOn);"),
    ("State 90: vacuum settle timer done -> verify.",
     "EQU(State,90)XIC(VacTmr.DN)MOV(100,State);"),
    ("State 100 VERIFY_VACUUM: confirmed (or Bypass_Vision) -> continue.",
     "EQU(State,100)[XIC(VisionRobot.Status.VacuumOK),XIC(Bypass_Vision)]MOV(110,State);"),
    ("State 100: vacuum not confirmed by the settle timeout -> fault (code 9).",
     "EQU(State,100)XIC(VacTmr.DN)XIO(VisionRobot.Status.VacuumOK)"
     "OTL(VisionRobot.Status.Faulted)MOV(9,VisionRobot.Status.FaultCode)MOV(900,State);"),
    ("State 110 CYLINDER_UP_PICK.",
     "EQU(State,110)OTU(CylinderDown)MOV(120,State);"),
    ("State 120: Z up at pick confirmed (or Bypass_Vision).",
     "EQU(State,120)[XIC(PickUp),XIC(Bypass_Vision)]MOV(130,State);"),
    ("State 130 MOVE_ABOVE_DROP.",
     "EQU(State,130)MOV(DropL,Move0_Target_Deg)MOV(DropR,Move1_Target_Deg)"
     "OTU(Move0_InPosition)OTU(Move1_InPosition)OTU(Move0_Loaded)OTU(Move1_Loaded)OTL(Move0_Execute)OTL(Move1_Execute)MOV(140,State);"),
    ("State 140: at drop -> drop execute.",
     "EQU(State,140)XIC(Move0_InPosition)XIC(Move1_InPosition)OTU(Move0_Execute)OTU(Move1_Execute)MOV(150,State);"),
    ("State 150 CYLINDER_DOWN_DROP.",
     "EQU(State,150)OTL(CylinderDown)MOV(160,State);"),
    ("State 160: Z down at drop confirmed (or Bypass_Vision).",
     "EQU(State,160)[XIC(DropDown),XIC(Bypass_Vision)]MOV(170,State);"),
    ("State 170 VACUUM_OFF_BLOWOFF: release vacuum, start blowoff.",
     "EQU(State,170)OTU(VacuumOn)OTL(Blowoff);"),
    ("State 170: blowoff timer done -> stop blowoff, advance.",
     "EQU(State,170)XIC(BlowTmr.DN)OTU(Blowoff)MOV(180,State);"),
    ("State 180 CYLINDER_UP_DROP.",
     "EQU(State,180)OTU(CylinderDown)MOV(190,State);"),
    ("State 190: Z up at drop confirmed (or Bypass_Vision).",
     "EQU(State,190)[XIC(DropUp),XIC(Bypass_Vision)]MOV(200,State);"),
    ("State 200 COMPLETE_JOB: publish CompleteCommandID + Done, back to idle.",
     "EQU(State,200)MOV(VisionRobot.Status.ActiveCommandID,VisionRobot.Status.CompleteCommandID)"
     "OTL(VisionRobot.Status.Done)OTU(VisionRobot.Status.CameraClear)MOV(0,State);"),
    ("State 900 FAULT: on Cmd.Reset (edge) while safe, publish FailedCommandID and "
     "return to idle.",
     "EQU(State,900)XIC(VisionRobot.Cmd.Reset)ONS(AR_prev)XIC(SafetyOK)"
     "MOV(VisionRobot.Status.ActiveCommandID,VisionRobot.Status.FailedCommandID)"
     "OTU(VisionRobot.Status.Faulted)MOV(0,State);"),
    ("Any state: Cmd.Abort or unsafe -> stop outputs and go to FAULT.",
     "[XIC(VisionRobot.Cmd.Abort),XIO(SafetyOK)]OTU(Move0_Execute)OTU(Move1_Execute)"
     "OTU(VacuumOn)OTU(Blowoff)MOV(900,State);"),
]

STATUS: List[Rung] = [
    ("R60_Status: publish the derived status bits (the rest are set where they "
     "occur). VacuumOK from the vacuum sensor.",
     "XIC(VacuumSensor)OTE(VisionRobot.Status.VacuumOK);"),
    ("Ready = idle, homed, not homing.",
     "EQU(State,0)EQU(HomeStep,0)XIC(VisionRobot.Status.Homed)"
     "OTE(VisionRobot.Status.Ready);"),
    ("Busy = a job is running.",
     "NEQ(State,0)OTE(VisionRobot.Status.Busy);"),
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
    ("Heartbeat", "DINT"),   # PC increments continuously; PLC watchdogs it
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
    ("PcAlive", "BOOL"),      # PC heartbeat seen within HB_TIMEOUT_MS
    ("Heartbeat", "DINT"),    # PLC increments each scan; PC verifies the ladder scans
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
                 constant=False, desc: str = "", unit: str = "",
                 set_by_hand: bool = False) -> None:
        self.name, self.dtype, self.value = name, dtype, value
        self.constant, self.desc = constant, desc
        # unit + set_by_hand drive the "values to set after import" reference doc
        self.unit, self.set_by_hand = unit, set_by_hand

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

    def add(name, dtype, value="0", constant=False, desc="", unit="", hand=False):
        tags.append(Tag(name, dtype, value, constant=constant, desc=desc,
                        unit=unit, set_by_hand=hand))

    # --- constants & tuning values (CSV carries no values — set after import) ---
    add("STEPS_PER_DEG", "REAL", "26.66667", True,
        "3200 pulses/rev * 3:1 / 360. Set value to 26.66667 after import.",
        unit="steps/deg", hand=True)
    add("MOVE_VEL", "DINT", "20000", desc="Move speed, steps/s (max 500000). Set ~20000.",
        unit="steps/s", hand=True)
    add("MOVE_ACC", "DINT", "100000", desc="Move accel, steps/s^2. Set ~100000.",
        unit="steps/s^2", hand=True)
    add("MOVE_DEC", "DINT", "0", desc="Move decel, steps/s^2. 0 => use accel.",
        unit="steps/s^2", hand=True)
    add("HOME_VEL_0", "DINT", "-2000",
        desc="Motor 0 homing speed, steps/s, signed toward the prox. Tune.",
        unit="steps/s", hand=True)
    add("HOME_VEL_1", "DINT", "2000",
        desc="Motor 1 homing speed, steps/s, signed toward the prox. Tune.",
        unit="steps/s", hand=True)
    add("HOME_ACC", "DINT", "50000", desc="Homing accel, steps/s^2. Set ~50000.",
        unit="steps/s^2", hand=True)
    add("HOME_TMO_MS", "DINT", "15000",
        desc="Homing timeout, ms (loaded into Home*_Tmr.PRE). Homing beyond this faults.",
        unit="ms", hand=True)
    add("HOME_OFFSET_L", "DINT", "0",
        desc="Left switch angle * STEPS_PER_DEG (ActualLeftDeg ~135.85). Set at commissioning.",
        unit="steps", hand=True)
    add("HOME_OFFSET_R", "DINT", "0",
        desc="Right switch angle * STEPS_PER_DEG (ActualRightDeg ~44.15). Set at commissioning.",
        unit="steps", hand=True)
    add("VAC_SETTLE", "DINT", "300", desc="Vacuum settle time, ms (VacTmr preset). Tune.",
        unit="ms", hand=True)
    add("BLOWOFF_TIME", "DINT", "200", desc="Blowoff time, ms (BlowTmr preset). Tune.",
        unit="ms", hand=True)
    add("CAMERA_CLEAR_L", "REAL", "0.0",
        desc="Camera-clear pose, left shoulder deg. Set to a safe out-of-view pose.",
        unit="deg", hand=True)
    add("CAMERA_CLEAR_R", "REAL", "0.0",
        desc="Camera-clear pose, right shoulder deg. Set to a safe out-of-view pose.",
        unit="deg", hand=True)

    # --- per-motor move + home glue (R_MoveMotor* / R_HomeMotor*) ---
    for m in (0, 1):
        add(f"Move{m}_Execute", "BOOL", desc=f"Commanded (level): a Motor {m} absolute move is requested until it completes.")
        add(f"Move{m}_Fault", "BOOL", desc=f"Motor {m} move fault (drive fault / shutdown / ALM).")
        add(f"Move{m}_InPosition", "BOOL", desc=f"Motor {m} at target (only after a loaded move).")
        add(f"Move{m}_Loaded", "BOOL", desc=f"Motor {m} has a move loaded - gates InPosition off a stale At_Target_Posn.")
        add(f"Move{m}_Steps", "DINT", desc=f"Motor {m} target in steps (deg * STEPS_PER_DEG).")
        add(f"Move{m}_Target_Deg", "REAL", "0.0", desc=f"Motor {m} absolute target angle, deg.")
        add(f"Home{m}_Req", "BOOL", desc=f"Request homing of Motor {m} (set by R30_Homing).")
        add(f"Home{m}_ons", "BOOL", desc=f"ONS storage for Home{m}_Req.")
        add(f"Home{m}_State", "DINT", desc=f"Motor {m} homing sub-state.")
        add(f"Home{m}_Tmr", "TIMER", desc=f"Motor {m} homing timeout timer.")
        add(f"Ax{m}_HomeDone", "BOOL", desc=f"Motor {m} homed.")
        add(f"Ax{m}_HomeFault", "BOOL", desc=f"Motor {m} homing fault.")
        add(f"Home{m}_Moved", "BOOL",
            desc=f"Motor {m} actually stepped during this homing attempt (latched from "
                 f"Status_Steps_Active). Gates Has_Homed so a stale/power-up reference "
                 f"can't false-complete a home with no motion.")
        add(f"Ax{m}_Ready", "BOOL",
            desc=f"Motor {m} drive ready (map to ClearLink:I1.Motor{m}_Status_Enabled).")
        add(f"EM806_{m}_ALM", "BOOL",
            desc=f"Motor {m} EM806 ALM. Change to an ALIAS of the ClearLink DI the drive alarm is wired to.")

    # --- homing coordinator (R30_Homing) ---
    add("HomeStep", "DINT", desc="Homing coordinator state.")
    add("HR_ons", "BOOL", desc="ONS storage for VisionRobot.Manual.HomeRequest.")
    add("HomeRst_ons", "BOOL", desc="ONS storage for the R30 homing reset-recovery rung.")
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
    add("EnDrop_Tmr", "TIMER",
        desc="Debounce for an unexpected drive drop-out (commanded on but not "
             "actually Enabled) - e.g. a drive power-cycle.")
    add("EN_DROP_TMO_MS", "DINT", "1000",
        desc="If the drive is commanded on but Status.Enabled stays false this long "
             "(ms), treat it as a drop-out (power-cycle) and drop Manual.Enable so it "
             "can't auto-re-enable. > drive enable time, < a real power-off.",
        unit="ms", hand=True)

    # --- PC<->PLC heartbeat watchdog (R10_Safety) ---
    add("HB_last", "DINT", desc="Last-seen VisionRobot.Cmd.Heartbeat value (watchdog).")
    add("HB_seen", "BOOL", desc="Pulses one scan when the PC heartbeat changes (resets HB_Tmr).")
    add("HB_Tmr", "TIMER", desc="PC-heartbeat watchdog timer (preset HB_TIMEOUT_MS).")
    add("HB_TIMEOUT_MS", "DINT", "1000",
        desc="PC heartbeat must change within this many ms or the PLC declares comms "
             "loss (code 10) and drops the drives. Set > 4x the PC heartbeat period.",
        unit="ms", hand=True)

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

    # --- bench-test bypass (NOT for production; see the GUI Bypass tab) ---
    add("Bypass_Homing", "BOOL",
        desc="BENCH: R30 marks referenced instantly on HomeRequest (skips the "
             "ClearLink prox homing move). Leave 0 in production.")
    add("Bypass_Vision", "BOOL",
        desc="BENCH: R50 auto-satisfies the Z reed switches + vacuum sensor so the "
             "pick/place motion runs open-loop. Leave 0 in production.")
    add("DriveClearReq", "BOOL",
        desc="BENCH: pulse true to clear ClearLink alerts + motor faults on both axes "
             "(pulses Clear_Alerts + Clear_Fault). Lets you clear a latched Shutdown "
             "without running homing. Leave 0 in production.")
    add("HRB_ons", "BOOL", desc="ONS storage for the R30 bypass-home rung.")
    add("HOME_ANGLE_L", "REAL", "135.8504", desc="Left home angle published on a "
        "bypass home, deg (nominal reference).", unit="deg", hand=True)
    add("HOME_ANGLE_R", "REAL", "44.1496", desc="Right home angle published on a "
        "bypass home, deg (nominal reference).", unit="deg", hand=True)

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


# groups for the values reference (name order within each group is preserved)
_VALUE_GROUPS: List[Tuple[str, List[str]]] = [
    ("Motion — absolute moves (R_MoveMotor0/1)",
     ["STEPS_PER_DEG", "MOVE_VEL", "MOVE_ACC", "MOVE_DEC"]),
    ("Homing (R_HomeMotor0/1)",
     ["HOME_VEL_0", "HOME_VEL_1", "HOME_ACC", "HOME_TMO_MS"]),
    ("Home offsets — measure at commissioning (R30_Homing)",
     ["HOME_OFFSET_L", "HOME_OFFSET_R"]),
    ("Auto pick/place process timers (R50_Auto)",
     ["VAC_SETTLE", "BLOWOFF_TIME"]),
    ("Poses — set to safe positions (R50_Auto)",
     ["CAMERA_CLEAR_L", "CAMERA_CLEAR_R"]),
    ("Heartbeat watchdog + drop-out debounce (R10/R20)",
     ["HB_TIMEOUT_MS", "EN_DROP_TMO_MS"]),
]


def values_reference_md() -> str:
    """Human list of the values the CSV/L5X import can't carry — every tag that
    must be given a value by hand after import (CSV creates definitions only)."""
    by_name = {t.name: t for t in _glue_tags()}
    lines = [
        "# PLC values to set after import",
        "",
        "Studio 5000's tag CSV/L5X import creates tag **definitions only** — every",
        "tag comes in at `0`/`0.0`. After importing `docs/l5x/RobotTags.csv`, set",
        "these values by hand (Controller Tags → Monitor/Edit). Most are starting",
        "points you refine at commissioning; `STEPS_PER_DEG` is fixed and",
        "`HOME_OFFSET_L/R` you measure. Source of truth: `scripts/render_plc_l5x.py`.",
        "",
    ]
    for title, names in _VALUE_GROUPS:
        lines += [f"## {title}", "",
                  "| Tag | Type | Value to set | Unit | Notes |",
                  "|---|---|---|---|---|"]
        for n in names:
            t = by_name[n]
            fixed = " (fixed)" if t.constant else ""
            lines.append(f"| `{t.name}` | {t.dtype} | `{t.value}`{fixed} | "
                         f"{t.unit} | {t.desc} |")
        lines.append("")
    # timer presets are members, loaded from the constants above
    lines += [
        "## Timer presets (`.PRE`)",
        "",
        "Timers import with `.PRE = 0`, which would make `.DN` true immediately.",
        "The homing timer is loaded by ladder each scan; the auto timers are loaded",
        "by `R50_Auto` when you build it. Set the **source constant** above and the",
        "preset follows.",
        "",
        "| Timer | `.PRE` loaded from | Value | Loaded by |",
        "|---|---|---|---|",
        f"| `Home0_Tmr` / `Home1_Tmr` | `HOME_TMO_MS` | `{by_name['HOME_TMO_MS'].value}` ms | `R_HomeMotor0/1` (MOV each scan) |",
        f"| `VacTmr` | `VAC_SETTLE` | `{by_name['VAC_SETTLE'].value}` ms | `R50_Auto` |",
        f"| `BlowTmr` | `BLOWOFF_TIME` | `{by_name['BLOWOFF_TIME'].value}` ms | `R50_Auto` |",
        "",
        "> Physical-I/O tags (E-stop, guards, limits, Z reed switches, and the",
        "> `CylinderDown`/`VacuumOn`/`Blowoff` outputs and `EM806_*_ALM`) take no",
        "> value — alias/map each to its real module point instead (see",
        "> `plc_setup.md` §6).",
        "",
    ]
    return "\n".join(lines)


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
        "R00_Main.L5X": routine_l5x("R00_Main", MAIN),
        "R10_Safety.L5X": routine_l5x("R10_Safety", SAFETY),
        "R20_Drives.L5X": routine_l5x("R20_Drives", DRIVES),
        "R30_Homing.L5X": routine_l5x("R30_Homing", COORD),
        "R40_Manual.L5X": routine_l5x("R40_Manual", MANUAL),
        "R50_Auto.L5X": routine_l5x("R50_Auto", AUTO),
        "R60_Status.L5X": routine_l5x("R60_Status", STATUS),
        "R_MoveMotor0.L5X": routine_l5x("R_MoveMotor0", move_rungs(0)),
        "R_MoveMotor1.L5X": routine_l5x("R_MoveMotor1", move_rungs(1)),
        "R_HomeMotor0.L5X": routine_l5x("R_HomeMotor0", home_rungs(0)),
        "R_HomeMotor1.L5X": routine_l5x("R_HomeMotor1", home_rungs(1)),
    })
    for name, text in xml_files.items():
        ET.fromstring(text)                      # well-formedness check
        (out / name).write_text(text)
        print(f"wrote {out / name}")

    csv_text = robot_tags_csv()
    # write_bytes so the CRLF line endings survive verbatim on any platform
    (out / "RobotTags.csv").write_bytes(csv_text.encode("ascii"))
    print(f"wrote {out / 'RobotTags.csv'}")

    # the "values to set after import" reference (docs/, next to the other guides)
    docs = out.parent
    (docs / "plc_tag_values.md").write_text(values_reference_md())
    print(f"wrote {docs / 'plc_tag_values.md'}")


if __name__ == "__main__":
    main()
