# PLC program outline (Studio 5000 / CompactLogix)

Implementation guide for the PLC side of the 5-bar robot. The Python side is
done and simulated-tested; this document tells the controls engineer what to
build in Studio 5000 so `PlcRobotDriver` (and later the pick/place handshake)
drives real hardware.

> **New to this? Start with [`plc_setup.md`](plc_setup.md)** — the step-by-step
> bring-up (network the ClearLink, import Teknic's example `.L5K`, create the
> tags, commission). This document is the reference behind it.

The authoritative tag list is the **PLC tab** in the app (generated from
`plc/tags.py`) — implement every tag there. This doc adds the *behaviour* behind
them. Structured Text (ST) is used for the outlines; implement as ladder or ST.

**Contents**
1. [Scope & responsibility split](#1-scope)
2. [The `VisionRobot` UDT](#2-udt)
3. [Axis / ClearLink interface](#3-axis)
4. [Program organization](#4-org)
5. [Add-On Instructions](#5-aoi)
6. [Manual jog / home logic](#6-manual)
7. [Automatic pick/place state machine](#7-auto)
8. [Homing routine](#8-homing)
9. [Faults & safety](#9-faults)
10. [CommandID handshake](#10-handshake)
11. [Mapping to the Python driver](#11-mapping)
12. [Commissioning checklist](#12-checklist)

---

## 1. Scope & responsibility split <a id="1-scope"></a>

Per Claude.md §7 — **Python computes joint angles; the PLC executes them.**

**PLC owns:** the sequence state machines, ClearLink EtherNet/IP command/status,
drive enable/fault, homing, degrees→steps scaling, soft/hard limits, pneumatics
(Z + vacuum + blowoff), and E-stop/safety. The PLC does **not** know 5-bar
kinematics — it receives absolute shoulder angles and moves to them.

**Python owns:** vision, calibration, cover selection, inverse kinematics,
workspace/singularity validation, and the tag handshake.

---

## 2. The `VisionRobot` UDT <a id="2-udt"></a>

Build one top-level UDT `VisionRobot` from four nested UDTs, so member paths
match the tag contract exactly (`VisionRobot.Manual.Enable`, …). Create a single
controller tag `VisionRobot` of this type.

```
UDT: VisionRobot_Cmd            // PC -> PLC (automatic job)
    RequestPickPlace   BOOL
    Abort              BOOL
    Reset              BOOL
    CommandID          DINT

UDT: VisionRobot_Target         // PC -> PLC (automatic job)
    Pick_LeftDeg       REAL
    Pick_RightDeg      REAL
    Drop_LeftDeg       REAL
    Drop_RightDeg      REAL
    HoleIndex          DINT
    CoverID            DINT

UDT: VisionRobot_Manual         // PC -> PLC (jog/home)
    Enable             BOOL
    HomeRequest        BOOL
    MoveToTarget       BOOL
    Abort              BOOL
    TargetLeftDeg      REAL
    TargetRightDeg     REAL
    CommandID          DINT

UDT: VisionRobot_Status         // PLC -> PC
    Ready              BOOL
    Busy               BOOL
    Done               BOOL
    Faulted            BOOL
    FaultCode          DINT
    Enabled            BOOL
    Homed              BOOL
    InPosition         BOOL
    Moving             BOOL
    ActualLeftDeg      REAL
    ActualRightDeg     REAL
    ActiveCommandID    DINT
    CompleteCommandID  DINT
    FailedCommandID    DINT
    VacuumOK           BOOL
    CameraClear        BOOL
    ReadyForVision     BOOL

UDT: VisionRobot
    Cmd     VisionRobot_Cmd
    Target  VisionRobot_Target
    Manual  VisionRobot_Manual
    Status  VisionRobot_Status
```

> The vision PC connects over EtherNet/IP with pycomm3 and reads/writes these by
> name. Keep the UDT names/paths exactly as above.

---

## 3. Axis / ClearLink interface <a id="3-axis"></a>

> **Grounded in the Teknic *ClearLink EtherNet/IP Setup and Object Data
> Reference*, Rev. 1.15.** The member/register names below are the real ones.

Motion goes CompactLogix (EtherNet/IP **scanner**) → **Teknic ClearLink
CLNK-4-13** (EtherNet/IP **adapter/slave**) → M-0/M-1 STEP/DIR/ENABLE →
**Leadshine EM806** → NEMA 23. Add the ClearLink to the I/O tree from its **EDS**
(→ **Add-On Profile**, module revision 2.091) and pick the **"Step Dir"**
connection type (the EM806 is a third-party step & direction drive; this is the
same connection type used for ClearPath-SD). Axis map: **Motor 0 = left shoulder,
Motor 1 = right shoulder** (M-2/M-3 spare). Z is pneumatic, not a ClearLink axis.

**Start from Teknic's example projects.** Teknic ships no motion AOI, but it
*does* ship working **CompactLogix example projects** (import the `.L5K`):
`SD_Homing`, `SD_Jog`, `SD_Position_Move`, `SD_Velocity_Move` (+ the
`clearlink_2.92.eds`). Each is a small ladder state machine on one motor —
**build from these**, adapt for two axes (Motor 0 = left, Motor 1 = right), and
tie them to the `VisionRobot` handshake. The tag/sequence details below are taken
directly from those examples.

**Teknic ships no motion AOI.** The AOP creates three implicit (cyclic) I/O
assemblies, exposed as flat controller tags — you drive motion by writing the
output assembly and reading the input assembly. For the "Step Dir" connection:

| Assembly | Instance | Tag (typical) | Direction |
|---|---|---|---|
| Input (T2O), `SINT[332]` | **100** | `<module>:I1` | ClearLink → PLC (feedback) |
| Output (O2T), `SINT[280]` | **112** | `<module>:O1` | PLC → ClearLink (commands) |
| Configuration, `SINT[232]` | **150** | `<module>:C` | one-time, sent on connect |

Each assembly carries a block **per motor** (Motor 0 = left, Motor 1 = right).
The members that matter for this robot:

**Output (`ClearLink:O1`, per motor) — exact AOP tag names:**
| AOP tag (Motor 0) | Type | Use |
|---|---|---|
| `Motor0_Move_Dist` | DINT | target, in **steps** (absolute or incremental per Abs flag) |
| `Motor0_Vel_Limit` | UDINT | positional-move speed, steps/s (max 500,000) |
| `Motor0_Accel_Lim` | UDINT | steps/s² (min resolution 1527) |
| `Motor0_Decel_Lim` | UDINT | steps/s² (0 ⇒ use accel) |
| `Motor0_Jog_Vel` | DINT | velocity/homing-move speed, steps/s |
| `Motor0_Output_Reg_Enable` | BOOL | enable this axis |
| `Motor0_Output_Reg_Abs_Flag` | BOOL | 1 = absolute move (our case), 0 = incremental |
| `Motor0_Output_Reg_Home_Flag` | BOOL | Homing Move Flag |
| `Motor0_Output_Reg_Load_Posn_Data` | BOOL | load & run a **position** move (handshake) |
| `Motor0_Output_Reg_Load_Vel_Data` | BOOL | load & run a **velocity** move (handshake) |
| `Motor0_Output_Reg_Clear_Alerts` | BOOL | clear the shutdown register |
| `Motor0_Output_Reg_Clear_Fault` | BOOL | clear a motor fault (cycles enable) |

**Input (`ClearLink:I1`, per motor) — exact AOP tag names:**
| AOP tag (Motor 0) | Type | Use |
|---|---|---|
| `Motor0_CommandedPosn` | DINT | **position feedback** (open-loop: commanded == actual, steps) |
| `Motor0_Status` | DWORD | full status register word |
| `Motor0_Shutdowns` | DWORD | latched cancel reasons (Table 25) |
| `Motor0_Status_HLFB_ON` | BOOL | HLFB asserted (enable-complete gate; see caveats) |
| `Motor0_Status_Enabled` | BOOL | axis enabled (bit 10) |
| `Motor0_Status_At_Target_Posn` | BOOL | position move done (bit 0) |
| `Motor0_Status_Steps_Active` | BOOL | axis moving (bit 1) |
| `Motor0_Status_Has_Homed` | BOOL | **reference established** (bit 13) |
| `Motor0_Status_Ready_To_Home` | BOOL | ok to command a homing move (bit 16) |
| `Motor0_Status_In_Home_Sensor` | BOOL | home prox state (bit 7) |
| `Motor0_Status_Load_Posn_Move_Ack` | BOOL | position-move-load ack (bit 19) |
| `Motor0_Status_Load_Vel_Move_Ack` | BOOL | velocity-move-load ack (bit 20) |
| `Motor0_Status_Motor_In_Fault` | BOOL | motor fault (bit 9, HLFB-derived) |
| `Motor0_Status_Shutdowns_Pres` | BOOL | a ClearLink shutdown is latched (bit 17) |
| `Motor0_Status_Clear_Motor_Fault_Ack` | BOOL | clear-fault handshake (bit 21) |

Motor 1 (right shoulder) is the same with the `Motor1_` prefix. The homing config
(Home Sensor connector, Config Register bits) lives in `ClearLink:C.Motor0Config`
/ `Motor1Config` (Configuration assembly), set once — see `docs/homing.md` and
the `SD_Homing` example.

**Configuration (`:C`, per motor — Step & Direction Motor Configuration Object,
class `0x64`), set once:** `Home Sensor` connector (the ClearLink input the
shoulder prox is wired to; −1 = hard-stop homing), `Positive/Negative Limit`
connectors, `Soft Limit 1/2`, `Max Deceleration`, and the `Config Register`
(Table 21): **bit 0 Homing Enable**, **bit 1 Home Sensor Active Level**, **bit 2
Enable Inversion**, **bit 3 HLFB Inversion**, **bit 5 Soft Limit Enable**.

**Degrees → steps** (Claude.md §5):

```
STEPS_PER_DEG := 26.66667          // 3200 pulses/rev * 3:1 / 360
steps := ROUND(angle_deg * STEPS_PER_DEG)
```

**Commanding one absolute move** — the exact sequence from Teknic's
`SD_Position_Move` (this replaces the invented "CmdPosition + MoveTrigger +
MoveDone" from earlier drafts). There is **no CommandID at the ClearLink level** —
the handshake is Load-Data → Ack:
1. **Enable:** latch `Motor0_Output_Reg_Enable`; wait `Motor0_Status_HLFB_ON`
   (Teknic's enable-complete gate).
2. **Load targets:** `Motor0_Move_Dist := ROUND(deg*STEPS_PER_DEG)`,
   `Motor0_Vel_Limit`, `Motor0_Accel_Lim`; **latch `Motor0_Output_Reg_Abs_Flag`**
   (absolute — the example uses incremental, we want absolute), then latch
   `Motor0_Output_Reg_Load_Posn_Data`.
3. **Ack:** when `Motor0_Status_Load_Posn_Move_Ack` comes true, unlatch
   `Motor0_Output_Reg_Load_Posn_Data`. The move is now running.
4. **Move done** = `Motor0_Status_At_Target_Posn`. Faults: `Motor0_Status_
   Motor_In_Fault` → clear with `..._Clear_Fault`; `Motor0_Status_Shutdowns_Pres`
   → clear with `..._Clear_Alerts` (read `Motor0_Shutdowns` for the reason).

> **HLFB caveat for the EM806 — this bites hard.** Teknic's examples gate enable
> on `HLFB_ON` and move-done on `At_Target_Posn`, both of which need HLFB asserted.
> The EM806 has **no HLFB**, so `HLFB Inversion` (Config Register bit 3) must be set
> to whatever makes the ClearLink read HLFB as **asserted**. Per the manual's
> third-party-drive troubleshooting (p.72) that is **OFF (0)** — the *opposite* of
> the ClearPath default of 1. If you leave it at **1**, the ClearLink reads HLFB
> de-asserted → **`Motor_In_Fault` (Status bit 9) latches → it cancels every move →
> a "Motor Faulted" shutdown (Shutdowns bit 10) latches** and blocks all motion
> (the axis just holds; no step pulses come out). Set bit 3 = **0**, confirm
> `Motor_In_Fault` (bit 9) reads 0 and `Enabled` (bit 10) reads 1, then clear the
> shutdown (Clear Alerts). `Steps_Active == 0` is the HLFB-independent move-done
> fallback used by the `SD_Jog`/`SD_Velocity_Move` examples.

**Open-loop / third-party-drive reality** — the EM806 is a plain step/dir drive
with **no HLFB** (High-Level Feedback), so:
- `Commanded Position` is the only position source; there is no missed-step
  detection. Absolute position is trustworthy only relative to the last **home**
  — which is why homing and the 85 % reach / singularity margins (enforced in
  Python) matter.
- Several status bits are HLFB-derived — **At Target Position** (0), **Motor in
  Fault** (9), **Enabled** (10). With no HLFB wired, set `HLFB Inversion` (Config
  Register bit 3) to **0 (OFF)** — the opposite of the ClearPath default — so HLFB
  reads as asserted; leaving it at 1 makes `Motor in Fault` trip and latch a
  "Motor Faulted" shutdown that cancels all motion. Set `Enable Inversion` (bit 2)
  if the EM806 enables on the opposite electrical sense.
- The EM806 must accept the ClearLink's **fixed 1 µs step pulse / 500 kHz max**
  — set its pulse-width/filter accordingly.
- The ClearLink's own "Motor Fault" is HLFB-based and will **not** see an EM806
  alarm. Wire the **EM806 ALM** output to a spare ClearLink digital input and
  read it via the Discrete Input Point object as the drive-fault signal (§9).

Wrap the per-axis assembly access in an AOI (`AOI_AxisMove`, §5) so these
device-specific members are touched in exactly one place — but note the AOI is
**ours**, not Teknic's.

---

## 4. Program organization <a id="4-org"></a>

```
Task: MainTask (periodic, 10–20 ms)
  Program: Robot
    R00_Main            // calls the routines below in order
    R10_Safety          // E-stop, guard, drive-fault aggregation
    R20_Drives          // enable/disable, map drive faults
    R30_Homing          // homing routine (called by manual + auto)
    R40_Manual          // manual jog/home state machine  (§6)
    R50_Auto            // automatic pick/place state machine (§7)
    R60_Status          // publish internal state -> VisionRobot.Status
```

`R40_Manual` and `R50_Auto` are mutually exclusive — gate on a `Mode` selector
(manual vs auto) or simply let Manual own the drives whenever
`VisionRobot.Cmd.RequestPickPlace` isn't active. Only one may command motion at a
time.

---

## 5. Add-On Instructions <a id="5-aoi"></a>

> **Full drop-in build (ladder visuals + Structured Text) for every routine in
> §5–§7 is in [`docs/plc_ladder.md`](plc_ladder.md).** The outlines below are the
> design intent; that sheet is what you build from.

These AOIs are **ours** — Teknic provides the EDS/AOP plus the CompactLogix
example projects (`SD_Position_Move`, `SD_Homing`, `SD_Jog`, …), not a motion
AOI. Each AOI wraps the ClearLink `ClearLink:O1`/`:I1` tags so the rest of the
program never touches them directly; the bodies below follow those examples. `Ax`
is an `AxisIF` alias onto one motor's tag block (`docs/plc_homing.md` §1).

### `AOI_AxisMove` — move one ClearLink axis to an absolute angle
Mirrors `SD_Position_Move`: Enable→HLFB_ON, load Move_Dist/Vel/Accel + Abs +
Load_Posn_Data, clear on Ack, done on At_Target_Posn.
```
INPUT   TargetDeg   REAL
INPUT   Execute     BOOL      // rising edge = start one move
IN_OUT  Ax          AxisIF    // alias onto ClearLink:O1/:I1 motor block (§3)
OUTPUT  InPosition  BOOL
OUTPUT  Fault       BOOL
LOCAL   Loaded : BOOL

Ax.Enable := TRUE;                              // Motor0_Output_Reg_Enable

// Level-triggered, NOT an Execute edge. Execute is a sticky latch (R40/R50 hold
// it until the move completes); edge-triggering it deadlocks if the first load is
// ever missed (e.g. a transient Shutdowns_Pres at the edge) — the latch never
// returns to 0, so a one-shot can never re-fire and every later command is
// ignored. A new command clears Loaded + InPosition in R40/R50, so this loads
// exactly once, then Loaded/InPosition hold it off.
IF Execute AND NOT Loaded AND NOT InPosition AND NOT Fault THEN
    Ax.MoveDist  := ROUND(TargetDeg * STEPS_PER_DEG);
    Ax.VelLimit  := MOVE_VEL;                   // steps/s (<= 500000)
    Ax.AccelLim  := MOVE_ACC;
    Ax.AbsFlag   := TRUE;                        // absolute (example uses incremental)
    Ax.LoadPosnData := TRUE;                     // Motor0_Output_Reg_Load_Posn_Data
    Loaded := TRUE;
END_IF;

IF Ax.LoadPosnMoveAck THEN Ax.LoadPosnData := FALSE; END_IF;   // handshake ack

InPosition := Loaded AND Ax.AtTargetPosn AND NOT Ax.LoadPosnData;
IF InPosition THEN Loaded := FALSE; END_IF;
Fault := Ax.MotorInFault OR Ax.ShutdownsPres OR Ax.ALM;
```
> `Ax.*` alias the AOP tags: `Ax.Enable`→`Motor0_Output_Reg_Enable`,
> `Ax.LoadPosnData`→`..._Load_Posn_Data`, `Ax.AtTargetPosn`→`Motor0_Status_At_Target_Posn`,
> `Ax.MotorInFault`/`Ax.ShutdownsPres`→the matching status bits. On the HLFB-less
> EM806, set `HLFB Inversion` so `At_Target_Posn` behaves, or gate `InPosition` on
> `NOT Ax.StepsActive` (as `SD_Jog` does).

### `AOI_HomeAxis` — reference one shoulder (ClearLink runs the homing)
**The ClearLink executes homing internally** — you do *not* hand-roll a
fast/back-off/slow jog. Mirrors `SD_Homing`. One-time config
(`ClearLink:C.Motor0Config`, §3): `Home Sensor` connector = the shoulder prox
input, `Config Register.Homing Enable` (bit 0) = 1, `Home Sensor Active Level`
(bit 1) to match, `HLFB Inversion` (bit 3) = **0 (OFF)** for the no-HLFB EM806
(=1 latches Motor-Faulted — see §3 HLFB caveat). Then:
```
INPUT   Execute       BOOL     // rising edge = home this axis
IN_OUT  Ax            AxisIF
OUTPUT  Homed         BOOL
OUTPUT  Fault         BOOL     // timeout / not ready

// 1. Enable, wait Ax.HLFB_ON; clear faults (Ax.ClearFault -> Clear_Motor_Fault_Ack)
//    and alerts (Ax.ClearAlerts -> NOT ShutdownsPres) so the axis is ready.
// 2. wait Ax.ReadyToHome                             (Motor0_Status_Ready_To_Home)
// 3. Ax.HomeFlag := TRUE;                            (Motor0_Output_Reg_Home_Flag)
//    Ax.JogVel := HOME_VEL; Ax.AccelLim := HOME_ACC; // signed: toward the prox
//    Ax.LoadVelData := TRUE;                          (Motor0_Output_Reg_Load_Vel_Data)
// 4. on Ax.LoadVelMoveAck: Ax.LoadVelData := FALSE; Ax.HomeFlag := FALSE;
// 5. ClearLink drives to the sensor, cancels motion, zeroes position there.
//    Homed := Ax.HasHomed                             (Motor0_Status_Has_Homed)
```
**Home-offset gotcha:** ClearLink zeroes position **at the switch trip point**,
so `Commanded Position = 0` means "at the home prox", *not* 140.54°/39.46°. Apply
the mechanical offset in the PLC's step↔degree mapping — define
`HOME_OFFSET_STEPS` per axis (= switch angle × `STEPS_PER_DEG`) and report
`ActualDeg := (Commanded Position + HOME_OFFSET_STEPS) / STEPS_PER_DEG`. (See
`docs/homing.md` for the switch angle.)
> **Full drop-in build (Structured Text + coordinator) for `AOI_HomeAxis` and
> `R30_Homing` is in [`docs/plc_homing.md`](plc_homing.md).**

### `AOI_CmdHandshake` — CommandID acknowledgement
```
INPUT   NewCommandID  DINT     // Manual.CommandID or Cmd.CommandID
INPUT   Trigger       BOOL     // rising edge = accept command
INPUT   Complete      BOOL     // move finished OK this scan
INPUT   Failed        BOOL
IN_OUT  Status        VisionRobot_Status

// on rising Trigger: Status.ActiveCommandID := NewCommandID
// on Complete:        Status.CompleteCommandID := Status.ActiveCommandID
// on Failed:          Status.FailedCommandID   := Status.ActiveCommandID
```

Edge detection in ST: keep a `prev` bit and detect `Trigger AND NOT prev`
(equivalent to a ladder ONS).

---

## 6. Manual jog / home logic (`R40_Manual`) <a id="6-manual"></a>

Drives the `VisionRobot.Manual.*` surface. Absolute-incremental: Python sends a
*validated* absolute angle target; the PLC does one coordinated move.

```
// --- enable ---
IF VisionRobot.Manual.Enable AND NOT Status.Faulted THEN
    EnableDrives();                 // ClearLink ENABLE, EM806 energized
    Status.Enabled := DrivesReady;
ELSE
    DisableDrives();
    Status.Enabled := FALSE;
END_IF;

// --- home (rising edge) ---
IF ONS(Manual.HomeRequest) AND Status.Enabled THEN
    StartHoming();                  // R30_Homing, both axes
END_IF;

// --- absolute move (rising edge) ---
IF ONS(Manual.MoveToTarget) AND Status.Enabled AND Status.Homed
      AND WithinSoftLimits(Manual.TargetLeftDeg, Manual.TargetRightDeg) THEN
    ActiveCmd := Manual.CommandID;   // AOI_CmdHandshake accept
    MoveActive := TRUE;
END_IF;

IF MoveActive THEN
    AOI_AxisMove(Manual.TargetLeftDeg,  Execute:=TRUE, Axis:=Axis0, ...);
    AOI_AxisMove(Manual.TargetRightDeg, Execute:=TRUE, Axis:=Axis1, ...);
    IF Axis0.InPosition AND Axis1.InPosition THEN
        Status.CompleteCommandID := ActiveCmd;   // handshake done
        Status.InPosition := TRUE;
        MoveActive := FALSE;
    END_IF;
END_IF;

// --- abort ---
IF Manual.Abort THEN
    StopMotion();  Status.Moving := FALSE;  MoveActive := FALSE;
END_IF;
```

Guards: refuse `MoveToTarget` unless `Enabled AND Homed AND WithinSoftLimits`.
If a move is requested while disabled/not-homed, set the matching `FaultCode`
(§9) instead of moving — Python already validates, but the PLC re-checks
defensively.

---

## 7. Automatic pick/place state machine (`R50_Auto`) <a id="7-auto"></a>

Reproduce the Claude.md §11 sequence. **Advance on real status bits, not timing
guesses.** Timers only for vacuum settle, blowoff, and debounce.

```
CASE State OF
   0:  // IDLE — wait for a job
       Status.Ready := TRUE;
       IF ONS(Cmd.RequestPickPlace) THEN
           ActiveCmd := Cmd.CommandID;  State := 10;  Status.Ready := FALSE;
       END_IF;
       // NOTE: the PC HOLDS Cmd.RequestPickPlace TRUE until it sees the command
       // accepted (Status.ActiveCommandID echoes Cmd.CommandID, or Status.Busy),
       // then drops it. So this ONS is guaranteed to see the level regardless of
       // scan phase — a brief fire-and-forget pulse would be lost between scans
       // and the job would silently never start.
  10:  MoveCameraClear();               State := 20;   // to camera-clear pose
  20:  IF AtCameraClear THEN Status.CameraClear := TRUE; State := 30; END_IF;
  30:  Status.ReadyForVision := TRUE;                  // (targets already written)
       State := 40;
  40:  LoadTargets();                   State := 50;   // latch Pick/Drop angles
  50:  MoveAbove(Pick);                 State := 60;
  60:  IF AtPick THEN State := 70; END_IF;
  70:  CylinderDown();                  State := 80;
  80:  IF PickDown THEN State := 90; END_IF;
  90:  VacuumOn();  T_settle(...);      State := 100;
 100:  IF Status.VacuumOK THEN State := 110; ELSE Fault(FC_VACUUM); END_IF;
 110:  CylinderUp();                    State := 120;
 120:  IF PickUp THEN State := 130; END_IF;
 130:  MoveAbove(Drop);                 State := 140;
 140:  IF AtDrop THEN State := 150; END_IF;
 150:  CylinderDown();                  State := 160;
 160:  IF DropDown THEN State := 170; END_IF;
 170:  VacuumOff();  Blowoff();  T_blow(...);  State := 180;
 180:  CylinderUp();                    State := 190;
 190:  IF DropUp THEN State := 192; END_IF;             // return to home
 192:  MoveAbove(HOME_ANGLE_L, HOME_ANGLE_R); State := 194;  // home pose (abs) — the
 194:  IF AtHome THEN State := 200; END_IF;             //  same angles homing references
 200:  Status.CompleteCommandID := ActiveCmd;  Status.Done := TRUE;  State := 0;
 900:  // FAULT — hold; wait for Cmd.Reset
       Status.Faulted := TRUE;
       IF ONS(Cmd.Reset) AND FaultCleared THEN
           Status.Faulted := FALSE;  Status.FailedCommandID := ActiveCmd;
           State := 0;
       END_IF;
END_CASE;

// any state: E-stop / drive fault / Cmd.Abort -> State := 900
```

`MoveAbove()` / `MoveCameraClear()` use `AOI_AxisMove` with the angle Python
provided (`Target.Pick_*Deg` / `Target.Drop_*Deg`).

---

## 8. Homing routine (`R30_Homing`) <a id="8-homing"></a>

See `docs/homing.md` for the mechanical placement. Home flag = tab on the
**proximal link L1** at r≈40 mm; sensor on the base. Both shoulders home
independently. **The ClearLink runs the homing move itself** (§3, §5) — the PLC
configures the home sensor + homing-enable, commands a homing move per axis, and
waits for the `Has Homed` status bit.

```
One-time config (per axis, Configuration assembly):
  Home Sensor connector := <prox input pin>      // 0..12 = local ClearLink input
  Config Register.HomingEnable (bit0) := 1
  Config Register.HomeSensorActiveLevel (bit1)   // to match the prox
  Config Register.HLFBInversion (bit3) := 0      // OFF for the no-HLFB EM806; =1 latches Motor-Faulted (§3)

Per axis (AOI_HomeAxis, mirrors SD_Homing):
  1. Motor0_Output_Reg_Enable := 1; wait Motor0_Status_HLFB_ON.
  2. Clear faults (Motor0_Output_Reg_Clear_Fault -> ..._Clear_Motor_Fault_Ack)
     and alerts (Motor0_Output_Reg_Clear_Alerts -> NOT ..._Shutdowns_Pres).
  3. Require Motor0_Status_Ready_To_Home.
  4. Command the homing move toward the switch:
        Motor0_Output_Reg_Home_Flag := 1; Motor0_Jog_Vel := HOME_VEL (signed);
        Motor0_Accel_Lim := HOME_ACC; then latch Motor0_Output_Reg_Load_Vel_Data.
        On Motor0_Status_Load_Vel_Move_Ack: clear Load_Vel_Data + Home_Flag.
  5. ClearLink drives to the sensor, cancels motion, and sets position 0 there.
  6. Wait Motor0_Status_Has_Homed.   Timeout -> Fault(FC_HOME_TIMEOUT).

After both axes homed: enable soft limits (Config Register.SoftLimitEnable +
Soft Limit 1/2), set Status.Homed := TRUE, and publish
  ActualDeg := (CommandedPosition + HOME_OFFSET_STEPS) / STEPS_PER_DEG.
```

**Home-offset:** the ClearLink datum is the switch trip point (position 0), not
the home angle — apply `HOME_OFFSET_STEPS` per axis so the published angle reads
140.54° (left) / 39.46° (right). See §5 and `docs/homing.md`.

Home is **mid-travel**, so the flag passes the sensor during normal motion. The
ClearLink only consults the home sensor during a homing move (a move with the
Homing Move Flag set), so it is safely ignored otherwise. Hard limits at
−20°/+200° stay wired into the drive fault chain at all times, and can also be
given to the ClearLink as `Positive/Negative Limit` connectors (§3).

Config source (read/mirror these; single source is `config/robot_config.yaml`):

```yaml
homing:
  home_left_deg: 140.5406
  home_right_deg: 39.4594
  flag_radius_mm: 40.0
  limit_min_deg: -20.0
  limit_max_deg: 200.0
```

---

## 9. Faults & safety <a id="9-faults"></a>

- **E-stop / guard** drop `EnableDrives` immediately (hardware safety relay
  first; the PLC bit mirrors it). E-stop → State 900, `Status.Enabled := FALSE`.
- **Hard limits (−20°/+200°, per shoulder)** wire into the drive ENABLE/fault
  chain so they act even if logic hangs; also read them as PLC inputs → fault.
- **EM806 ALM** (drive alarm) → fault.
- **Soft limits** clamp/refuse targets before any move.
- `Cmd.Reset` / (recommended) a manual reset clears a latched fault once the
  condition is gone. Reset is **level-driven** (not one-shot):
  - **Two different objects.** (1) The **Motor Shutdowns** register is
    Or-accumulating and latched; it is cleared by **Clear Alerts** (Output bit 6),
    which needs *no* enable. (2) A **Motor Fault** — `Motor_In_Fault`, Motor Status
    bit 9 — *sets* when HLFB de-asserts while the Enable output is asserted, and
    **latches**: it does **not** fall on its own when HLFB returns. Per the manual it
    clears **only** via **Clear Motor Fault** (Output bit 7), a momentary
    disable/re-enable **enable cycle** — which does nothing unless the Enable output
    is asserted at the time.
  - **The deadlock and how Reset breaks it.** Anti-restart drops `Manual.Enable` on
    any fault and `EnableReq` refuses to enable while `Faulted`, so the Enable output
    is off — and the enable cycle can't run, so bit 9 can never clear (this is why
    hitting Clear Alerts alone does nothing to bit 9). While `Cmd.Reset` (or the
    bench `DriveClearReq`) is held and safe, R20 `DriveClearActive` **force-enables
    each axis whose HLFB is present** (`HLFB_ON` bit 14 = 1) so the Clear-Motor-Fault
    enable cycle can run and clear bit 9. It is **gated on HLFB_ON** so an axis whose
    HLFB is still de-asserted (drive genuinely alarmed) is **never** energized — its
    fault correctly persists until HLFB is restored. `DriveClearActive` does **not**
    set `Status.Enabled` (that needs `EnableReq`/`Manual.Enable`), so no motion is
    possible and the drives end **disabled** — re-running is a deliberate Enable.
  - **Honest reporting.** The R10 reset rung runs **before** the fault-latch rungs,
    so after re-detection end-of-scan `Faulted` reflects the true live state. The
    PC's `reset()` holds `Cmd.Reset` and polls `Status.Faulted` (needs two
    consecutive clear reads, to dodge the mid-scan clear/re-latch window) until it
    genuinely clears or times out. If the fault source is gone the reset sticks; if
    not, it honestly fails instead of a fixed pulse masking it as OK.
  - **If bit 9 still won't clear** with HLFB restored and the enable cycle run, the
    drive is genuinely still faulted or the ClearLink is mis-reading HLFB — per the
    manual's troubleshooting, check **HLFB Inversion** / **Enable Inversion** (Motor
    Config Register, set in the ClearLink web UI) and the fault-signal **cable**.

**Recommended `Status.FaultCode` values** (PLC-defined; keep 0 = none):

| Code | Meaning |
|---|---|
| 0 | No fault |
| 1 | Drive alarm (EM806 ALM) |
| 2 | E-stop / guard open |
| 3 | Hard limit tripped |
| 4 | Homing failed / timeout |
| 5 | Move commanded while not enabled |
| 6 | Move commanded while not homed |
| 7 | Target outside soft limits |
| 8 | Move timeout (profile did not complete) |
| 9 | Vacuum not confirmed |
| 10 | Command watchdog / comms loss |

The `FaultCode` numbering is PLC-defined — `PlcRobotDriver` only checks
`Status.Faulted` and surfaces whatever `FaultCode` it reads in the error
message, so pick any consistent scheme (the repo's simulator uses its own
placeholder codes purely for GUI testing).

### 9a. Heartbeat watchdog (dead-man) — code 10

The app and the PLC exchange a liveness heartbeat so an app that crashes, hangs,
or is closed **cannot leave the drives energized**, and a fresh app can never
find drives "already enabled" by a dead session:

- **PC → PLC:** `PlcRobotDriver` runs a background thread that increments
  `VisionRobot.Cmd.Heartbeat` every `heartbeat_interval_s` (0.2 s) while
  connected.
- **PLC watchdog (R10_Safety):** a timer (`HB_Tmr`, preset `HB_TIMEOUT_MS` ≈ 1 s)
  resets every time `Cmd.Heartbeat` changes. If it expires, `Status.PcAlive`
  drops; if the operator had Enable requested, it latches **FaultCode 10**.
- **Enable is gated on `PcAlive`** (R20): `EnableReq = Manual.Enable · SafetyOK ·
  ¬Faulted · PcAlive`. So the drives **cannot be enabled — or stay enabled —
  without a live app heartbeat.** A crashed app drops the drives within
  `HB_TIMEOUT_MS`.
- **PLC → PC:** the PLC increments `VisionRobot.Status.Heartbeat` each scan so the
  app can confirm the ladder is actually *scanning* (not just that a tag read
  ACKed). Watch both on the Diagnostics tab.
- **Startup reconciliation:** on connect, the controller reads the PLC's real
  `Status.Homed`/angles and adopts the reference if valid, instead of assuming a
  fixed state — real handshaking, not a blind reset.

Set `HB_TIMEOUT_MS` to at least 4× the PC heartbeat period (default 0.2 s → ≥ 1 s)
so a momentary comms jitter doesn't nuisance-trip the dead-man.

---

## 10. CommandID handshake <a id="10-handshake"></a>

`CommandID` rejects stale/duplicated commands and lets Python confirm *its*
command finished. Both the manual and auto surfaces use the same pattern:

```
Python                                   PLC
------                                   ---
write Target*Deg                         (idle, Ready/Enabled)
write Manual.CommandID = N               ActiveCommandID := N   (on trigger edge)
pulse Manual.MoveToTarget --------------> begin coordinated move
                                         ... InPosition ...
wait CompleteCommandID == N  <---------- CompleteCommandID := N
      AND InPosition
```

On failure the PLC sets `FailedCommandID := N` and (for a real fault)
`Faulted := TRUE` + `FaultCode`. Python treats *neither Complete nor a clean
Faulted within the timeout* as its own recoverable state (Claude.md §11 step 5) —
so make sure the PLC always resolves a command to Complete **or** Failed.

---

## 11. Mapping to the Python driver <a id="11-mapping"></a>

`PlcRobotDriver` (`plc/plc_robot_driver.py`) does exactly this:

| Driver method | Writes | Waits on |
|---|---|---|
| `enable()` | `Manual.Enable := 1` | `Status.Enabled` |
| `disable()` | `Manual.Enable := 0` | — |
| `home()` | pulse `Manual.HomeRequest` | `Status.Homed` |
| `move_to_angles(l,r)` | `Manual.TargetLeftDeg`, `Manual.TargetRightDeg`, bump `Manual.CommandID`, pulse `Manual.MoveToTarget` | `Status.CompleteCommandID == CommandID` **and** `Status.InPosition` |
| `read_angles()` | — | reads `Status.ActualLeft/RightDeg` (None until `Homed`) |
| `stop()` | pulse `Manual.Abort` | — |

Waits poll status with a timeout and abort on `Status.Faulted`. The
`SimulatedPlcClient` in the repo emulates exactly these reactions, so you can
validate the sequence logic against the GUI before touching hardware.

---

## 12. Commissioning checklist <a id="12-checklist"></a>

1. ClearLink added from its **EDS/AOP**, **"Step Dir"** connection type; Motor
   0/1 map to left/right; `HLFB Inversion` + (if needed) `Enable Inversion` set
   for the EM806; EM806 configured for **1 µs / 500 kHz** step pulses;
   `STEPS_PER_DEG = 26.6667` verified (command 90°, measure the shoulder).
2. Home prox on L1 (r≈40) wired to a ClearLink input and set as each motor's
   `Home Sensor` connector; hard limits (−20/+200) in the drive fault chain (and
   optionally as `Positive/Negative Limit` connectors); EM806 ALM read as a fault.
3. Homing: `Homing Enable` set, `HOME_VEL`/approach direction tuned, and
   `HOME_OFFSET_STEPS` set so after homing `ActualLeftDeg ≈ 140.54`,
   `ActualRightDeg ≈ 39.46`. Confirm `Has Homed` asserts.
4. Soft limits enforced; a move past −20/+200 is refused with `FaultCode 7`.
5. Manual handshake: from the app's **PLC tab → Connect PLC**, then **Robot
   Test → Enable → Home (find ref) → jog**. Confirm `CompleteCommandID` tracks
   each jog and `InPosition` gates it.
6. E-stop drops enable and forces State 900; `Reset` recovers.
7. Only then bring up the automatic pick/place sequence (§7).
```
