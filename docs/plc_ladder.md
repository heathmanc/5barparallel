# Ladder + Structured Text build sheets — all routines

Drop-in build for every PLC routine except homing (which has its own sheet,
[`docs/plc_homing.md`](plc_homing.md)). This complements
[`docs/plc_program.md`](plc_program.md) (architecture, UDT, tag contract) by
giving the actual logic in two equivalent forms:

- **Ladder** — a rung diagram to rebuild in Studio 5000.
- **Structured Text** — paste directly into an ST routine/AOI.

They are the same logic; use whichever your routine type is. ST is literally
pasteable; the ladder is the visual to rebuild rung-by-rung.

**Scan order** — call from `R00_Main` each scan: `R10_Safety` → `R20_Drives` →
`R30_Homing` → (`R40_Manual` **or** `R50_Auto`, by mode) → `R60_Status`.

---

## Shared: `AxisIF` UDT & constants

One `AxisIF` per shoulder (`Ax0`, `Ax1`), aliasing the ClearLink **Step-Dir**
`:O1`/`:I1` motor block. Member names below are the real ones from the Teknic
*Object Data Reference* Rev. 1.15 (see `docs/plc_program.md` §3); `*.n` are
register bits:

| Member | Type | Real ClearLink member |
|---|---|---|
| `MoveDistance` | DINT | Output · Move Distance (steps target) |
| `VelocityLimit` | UDINT | Output · Velocity Limit (steps/s, ≤500k) |
| `AccelLimit`/`DecelLimit` | UDINT | Output · Acceleration/Deceleration Limit |
| `JogVelocity` | DINT | Output · Jog Velocity (steps/s) |
| `OutReg` | DWORD | Output · **Output Register** — `.0` Enable, `.1` Absolute, `.2` HomingMoveFlag, `.3` LoadPositionMove, `.4` LoadVelocityMove, `.5` SW-E-Stop, `.6` ClearAlerts |
| `CmdPosition` | DINT | Input · Commanded Position (open-loop position) |
| `StatusReg` | DWORD | Input · **Status Register** — `.1` StepsActive, `.10` Enabled, `.13` HasHomed, `.16` ReadyToHome, `.17` ShutdownsPresent, `.19` LoadPosMoveAck |
| `ALM` | BOOL | EM806 alarm → a ClearLink digital input (DIP object) |

> **No Teknic motion AOI** — the AOP only exposes these assembly tags; the AOIs
> below are ours. `Enable` is `OutReg.0`; the "drive ready" flag is `StatusReg.10`
> (with `HLFB Inversion` set for the HLFB-less EM806, §3); "stop" is a SW E-Stop
> (`OutReg.5`); "move done" is `NOT StatusReg.1` (Steps Active). There is no
> `MoveTrigger`/`MoveDone`/`Redefine` member — those were invented; use the
> Load-Position-Move handshake and, for homing, the ClearLink's built-in homing
> move (`plc_homing.md`).

Constants: `STEPS_PER_DEG := 26.66667`, `MOVE_VEL/MOVE_ACC/MOVE_DEC` (move
profile), plus `VAC_SETTLE`, `BLOWOFF_TIME` (timer presets, ms) and the pose
angles `PickL/PickR/DropL/DropR`, `CAMERA_CLEAR_L/CAMERA_CLEAR_R`. Fault codes are
the table in `plc_program.md` §9.

---

## `AOI_AxisMove` — move one shoulder to an absolute angle

Params: `In` TargetDeg (REAL), Execute (BOOL), StepsPerDeg (REAL) · `InOut` Ax
(`AxisIF`) · `Out` InPosition (BOOL), Fault (BOOL) · `Local` TargetSteps (DINT),
prevExec (BOOL).

> ⚠️ **`plc_axismove_ladder.svg` shows the superseded `MoveTrigger`/`MoveDone`
> interface.** Rebuild from the corrected ST below: load the move, pulse
> **Load Position Move** with its ack handshake, and take *move done* from
> **Steps Active** (there is no closed-loop `MoveDone` on an open-loop drive).

```pascal
TargetSteps := TRUNC(TargetDeg * StepsPerDeg);

(* rising edge: load targets + pulse Load Position Move (Output Register bit 3) *)
IF Execute AND NOT prevExec AND NOT Fault THEN
    Ax.MoveDistance  := TargetSteps;
    Ax.VelocityLimit := MOVE_VEL;
    Ax.AccelLimit    := MOVE_ACC;
    Ax.DecelLimit    := MOVE_DEC;
    Ax.OutReg.1 := 1;              (* Absolute Flag *)
    Ax.OutReg.3 := 1;              (* Load Position Move *)
END_IF;
prevExec := Execute;

IF Ax.StatusReg.19 THEN Ax.OutReg.3 := 0; END_IF;   (* clear on Load-Move Ack *)

InPosition := NOT Ax.StatusReg.1 AND NOT Ax.OutReg.3; (* Steps Active == 0 *)
Fault      := Ax.StatusReg.17 OR Ax.ALM;              (* shutdown OR drive alarm *)
```

---

## `AOI_CmdHandshake` — CommandID acknowledgement

Params: `In` NewCommandID (DINT), Trigger, Complete, Failed (BOOL) · `InOut`
Status (`VisionRobot_Status`) · `Local` Trig_prev (BOOL).

![AOI_CmdHandshake ladder](plc_handshake_ladder.svg)

```pascal
IF Trigger AND NOT Trig_prev THEN
    Status.ActiveCommandID := NewCommandID;
END_IF;
Trig_prev := Trigger;
IF Complete THEN Status.CompleteCommandID := Status.ActiveCommandID; END_IF;
IF Failed   THEN Status.FailedCommandID   := Status.ActiveCommandID; END_IF;
```

---

## `R10_Safety` — E-stop, limits, drive-fault aggregation

![R10_Safety ladder](plc_safety_ladder.svg)

```pascal
IF EStop_Pressed OR NOT Guard_Closed THEN
    VisionRobot.Status.Faulted := 1; VisionRobot.Status.FaultCode := 2;
    EnableReq := 0;
END_IF;
IF Ax0_LimitMin OR Ax0_LimitMax OR Ax1_LimitMin OR Ax1_LimitMax THEN
    VisionRobot.Status.Faulted := 1; VisionRobot.Status.FaultCode := 3;
END_IF;
IF Ax0.Alarm OR Ax1.Alarm THEN
    VisionRobot.Status.Faulted := 1; VisionRobot.Status.FaultCode := 1;
END_IF;
SafetyOK := (NOT VisionRobot.Status.Faulted) AND EStop_OK AND Guard_Closed;
IF VisionRobot.Cmd.Reset AND NOT Reset_prev AND SafetyOK THEN
    VisionRobot.Status.Faulted := 0; VisionRobot.Status.FaultCode := 0;
END_IF;
Reset_prev := VisionRobot.Cmd.Reset;
```

---

## `R20_Drives` — drive enable & ready feedback

![R20_Drives ladder](plc_drives_ladder.svg)

```pascal
Ax0.Enable := EnableReq AND SafetyOK AND NOT VisionRobot.Status.Faulted;
Ax1.Enable := Ax0.Enable;
VisionRobot.Status.Enabled := Ax0.Ready AND Ax1.Ready AND EnableReq;
```

---

## `R40_Manual` — manual jog/home command logic

Handles the `VisionRobot.Manual.*` surface (enable, absolute move, abort).
Homing is delegated to `R30_Homing`; physical drive enable is done by `R20_Drives`.

![R40_Manual ladder](plc_manual_ladder.svg)

```pascal
EnableReq := VisionRobot.Manual.Enable;

WithinLimits :=
    (VisionRobot.Manual.TargetLeftDeg  >= -20.0) AND
    (VisionRobot.Manual.TargetLeftDeg  <= 200.0) AND
    (VisionRobot.Manual.TargetRightDeg >= -20.0) AND
    (VisionRobot.Manual.TargetRightDeg <= 200.0);

(* accept / reject a move on the rising edge (ladder splits the reject into
   three rungs; ST folds them into one ELSIF chain) *)
IF VisionRobot.Manual.MoveToTarget AND NOT MTT_prev THEN
    IF VisionRobot.Status.Enabled AND VisionRobot.Status.Homed AND WithinLimits THEN
        VisionRobot.Status.ActiveCommandID := VisionRobot.Manual.CommandID;
        MoveActive := 1;
    ELSIF NOT VisionRobot.Status.Enabled THEN
        VisionRobot.Status.Faulted := 1; VisionRobot.Status.FaultCode := 5;
    ELSIF NOT VisionRobot.Status.Homed THEN
        VisionRobot.Status.Faulted := 1; VisionRobot.Status.FaultCode := 6;
    ELSE
        VisionRobot.Status.Faulted := 1; VisionRobot.Status.FaultCode := 7;
    END_IF;
END_IF;
MTT_prev := VisionRobot.Manual.MoveToTarget;

AOI_AxisMove(MoveL, TargetDeg:=VisionRobot.Manual.TargetLeftDeg,
             Execute:=MoveActive, StepsPerDeg:=STEPS_PER_DEG, Ax:=Ax0);
AOI_AxisMove(MoveR, TargetDeg:=VisionRobot.Manual.TargetRightDeg,
             Execute:=MoveActive, StepsPerDeg:=STEPS_PER_DEG, Ax:=Ax1);

IF MoveActive AND MoveL.InPosition AND MoveR.InPosition THEN
    VisionRobot.Status.CompleteCommandID := VisionRobot.Status.ActiveCommandID;
    VisionRobot.Status.InPosition := 1;
    MoveActive := 0;
END_IF;
VisionRobot.Status.Moving := MoveActive AND NOT VisionRobot.Status.InPosition;

IF VisionRobot.Manual.Abort THEN
    MoveActive := 0; Ax0.Stop := 1; Ax1.Stop := 1;
END_IF;
```

---

## `R50_Auto` — automatic pick/place sequence

The Claude.md §11 state machine. Advances on real status bits; timers only for
vacuum settle and blowoff.

![R50_Auto ladder](plc_auto_ladder.svg)

```pascal
(* service the two process timers each scan *)
VacTmr.PRE := VAC_SETTLE;  VacTmr.TimerEnable := (State = 90) OR (State = 100);
TONR(VacTmr);
BlowTmr.PRE := BLOWOFF_TIME;  BlowTmr.TimerEnable := (State = 170);
TONR(BlowTmr);

CASE State OF
    0:   VisionRobot.Status.Ready := 1;
         IF VisionRobot.Cmd.RequestPickPlace AND NOT RPP_prev THEN
             VisionRobot.Status.ActiveCommandID := VisionRobot.Cmd.CommandID;
             VisionRobot.Status.Ready := 0;  State := 10;
         END_IF;
    10:  CmdCameraClear := 1;  State := 20;              (* MOVE_CAMERA_CLEAR *)
    20:  IF AtCameraClear THEN VisionRobot.Status.CameraClear := 1; State := 30; END_IF;
    30:  VisionRobot.Status.ReadyForVision := 1;  State := 40;
    40:  (* latch Target.Pick_* / Drop_* into PickL/PickR/DropL/DropR *) State := 50;
    50:  CmdMovePick := 1;  State := 60;                 (* MOVE_ABOVE_PICK *)
    60:  IF AtPick THEN State := 70; END_IF;
    70:  CylinderDown := 1;  State := 80;                (* CYLINDER_DOWN_PICK *)
    80:  IF PickDown THEN State := 90; END_IF;
    90:  VacuumOn := 1;                                  (* VACUUM_ON *)
         IF VacTmr.DN THEN State := 100; END_IF;
    100: IF VisionRobot.Status.VacuumOK THEN State := 110;   (* VERIFY_VACUUM *)
         ELSIF VacTmr.DN THEN
             VisionRobot.Status.Faulted := 1;
             VisionRobot.Status.FaultCode := 9;  State := 900;
         END_IF;
    110: CylinderDown := 0;  State := 120;               (* CYLINDER_UP_PICK *)
    120: IF PickUp THEN State := 130; END_IF;
    130: CmdMoveDrop := 1;  State := 140;                (* MOVE_ABOVE_DROP *)
    140: IF AtDrop THEN State := 150; END_IF;
    150: CylinderDown := 1;  State := 160;               (* CYLINDER_DOWN_DROP *)
    160: IF DropDown THEN State := 170; END_IF;
    170: VacuumOn := 0;  Blowoff := 1;                   (* VACUUM_OFF_BLOWOFF *)
         IF BlowTmr.DN THEN Blowoff := 0;  State := 180; END_IF;
    180: CylinderDown := 0;  State := 190;               (* CYLINDER_UP_DROP *)
    190: IF DropUp THEN State := 200; END_IF;
    200: VisionRobot.Status.CompleteCommandID := VisionRobot.Status.ActiveCommandID;
         VisionRobot.Status.Done := 1;  State := 0;      (* COMPLETE_JOB *)
    900: IF VisionRobot.Cmd.Reset AND NOT AR_prev AND SafetyOK THEN
             VisionRobot.Status.Faulted := 0;
             VisionRobot.Status.FailedCommandID := VisionRobot.Status.ActiveCommandID;
             State := 0;
         END_IF;
END_CASE;
RPP_prev := VisionRobot.Cmd.RequestPickPlace;
AR_prev  := VisionRobot.Cmd.Reset;

(* any state: abort or unsafe -> FAULT *)
IF VisionRobot.Cmd.Abort OR NOT SafetyOK THEN State := 900; END_IF;
```

**Move dispatcher** — the `Cmd*`/`At*` bits above resolve to `AOI_AxisMove`
against the pose constants (keeps the state machine readable):

```pascal
IF CmdCameraClear THEN AutoTL:=CAMERA_CLEAR_L; AutoTR:=CAMERA_CLEAR_R; AutoMove:=1; CmdCameraClear:=0; END_IF;
IF CmdMovePick    THEN AutoTL:=PickL; AutoTR:=PickR; AutoMove:=1; CmdMovePick:=0; END_IF;
IF CmdMoveDrop    THEN AutoTL:=DropL; AutoTR:=DropR; AutoMove:=1; CmdMoveDrop:=0; END_IF;

AOI_AxisMove(AutoL, TargetDeg:=AutoTL, Execute:=AutoMove, StepsPerDeg:=STEPS_PER_DEG, Ax:=Ax0);
AOI_AxisMove(AutoR, TargetDeg:=AutoTR, Execute:=AutoMove, StepsPerDeg:=STEPS_PER_DEG, Ax:=Ax1);

Arrived       := AutoL.InPosition AND AutoR.InPosition;
AtCameraClear := Arrived AND (AutoTL = CAMERA_CLEAR_L);
AtPick        := Arrived AND (AutoTL = PickL);
AtDrop        := Arrived AND (AutoTL = DropL);
```

`CylinderDown`, `VacuumOn`, `Blowoff` are the pneumatic solenoid outputs;
`PickDown/PickUp/DropDown/DropUp` are the Z reed switches.

---

## `R60_Status` — publish status bits

Bits not already set by other routines (Enabled/Homed/InPosition/CommandIDs/
Actual angles are set where they occur).

![R60_Status ladder](plc_status_ladder.svg)

```pascal
VisionRobot.Status.Ready      := (State = 0) AND (HomeStep = 0)
                                 AND VisionRobot.Status.Homed;
VisionRobot.Status.Busy       := (State <> 0);
VisionRobot.Status.VacuumOK   := VacuumSensor;
VisionRobot.Status.CameraClear := AtCameraClear;
```
