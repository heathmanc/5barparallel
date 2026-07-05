# PLC program outline (Studio 5000 / CompactLogix)

Implementation guide for the PLC side of the 5-bar robot. The Python side is
done and simulated-tested; this document tells the controls engineer what to
build in Studio 5000 so `PlcRobotDriver` (and later the pick/place handshake)
drives real hardware.

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

Motion goes PLC → **Teknic ClearLink CLNK-4-13** (EtherNet/IP, add via its
AOP/EDS) → STEP/DIR/ENABLE → **Leadshine EM806** → NEMA 23. Axis map:
**Axis 0 = left shoulder, Axis 1 = right shoulder** (2/3 spare). Z is pneumatic,
not a ClearLink axis.

**Degrees → steps** (Claude.md §5):

```
STEPS_PER_DEG := 26.66667          // 3200 pulses/rev * 3:1 / 360
steps := ROUND(angle_deg * STEPS_PER_DEG)
```

**Open-loop reality:** the EM806 are open-loop steppers — the ClearLink generates
the step profile but gets no position feedback. So:
- `Status.InPosition` = **commanded move profile complete** (ClearLink "move
  done"), not a closed-loop confirmation.
- Absolute position is only trustworthy relative to the last **home**; step
  integrity (no missed steps) is assumed. This is why homing and the 85 % reach
  / singularity margins (enforced in Python) matter.
- Use the EM806 alarm/ALM output as a drive-fault input; wire it into the fault
  chain (§9).

Wrap the ClearLink per-axis command/status in an AOI (`AOI_AxisMove`, §5) so the
device-specific assembly members are touched in exactly one place.

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

Three reusable AOIs keep the logic clean and testable.

### `AOI_AxisMove` — move one ClearLink axis to an absolute angle
```
INPUT   TargetDeg   REAL
INPUT   Execute     BOOL      // level: hold true while the move is wanted
IN_OUT  Axis        <ClearLink axis alias / assembly>
OUTPUT  InPosition  BOOL
OUTPUT  Fault       BOOL

// body
steps := ROUND(TargetDeg * STEPS_PER_DEG);
IF Execute AND NOT Fault THEN
    Axis.CmdPosition := steps;           // load target (device-specific member)
    Axis.MoveTrigger := TRUE;            // request absolute move
END_IF;
InPosition := Axis.MoveDone AND (Axis.CmdPosition = steps);
Fault      := Axis.Alarm;               // EM806 ALM via ClearLink input
```

### `AOI_HomeAxis` — reference one shoulder to its home switch
```
INPUT   HomeSwitch    BOOL
INPUT   Execute       BOOL
INPUT   HomeAngleDeg  REAL     // 135.85 (left) / 44.15 (right)
IN_OUT  Axis
OUTPUT  Homed         BOOL
OUTPUT  Fault         BOOL     // set on timeout

// sequence (see §8): fast approach -> switch -> back off -> slow re-approach
// -> set Axis reference so ActualDeg = HomeAngleDeg -> Homed := TRUE
```
> **Full drop-in build (ladder visuals + Structured Text) for `AOI_HomeAxis`
> and the `R30_Homing` coordinator is in [`docs/plc_homing.md`](plc_homing.md).**

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
 190:  IF DropUp THEN State := 200; END_IF;
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
independently.

```
Per axis (AOI_HomeAxis):
  1. Enable drives.
  2. Jog toward the home switch at HOMING_FAST_VEL, one consistent direction.
  3. On HomeSwitch rising edge: stop, back off a fixed distance.
  4. Re-approach at HOMING_SLOW_VEL; on switch, stop.
  5. Set the axis reference so ActualDeg = HomeAngleDeg
        (left 135.85, right 44.15  — from config homing block).
  6. Homed := TRUE.
  Timeout on each approach -> Fault(FC_HOME_TIMEOUT).

After both axes Homed: enable soft limits (MIN -20, MAX +200) and set
Status.Homed := TRUE; publish ActualLeft/RightDeg.
```

Home is **mid-travel**, so the flag passes the sensor during normal motion — read
the home switch **only during homing**, ignore it otherwise. Hard limits at
−20°/+200° stay wired into the drive fault chain at all times.

Config source (read/mirror these; single source is `config/robot_config.yaml`):

```yaml
homing:
  home_left_deg: 135.8504
  home_right_deg: 44.1496
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
  condition is gone.

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

1. ClearLink AOP added; Axis 0/1 map to left/right; `STEPS_PER_DEG = 26.6667`
   verified (command 90°, measure the shoulder).
2. Home switches on L1 (r≈40) wired; hard limits (−20/+200) in the drive fault
   chain; EM806 ALM read as a fault.
3. Homing: fast/slow approach directions and back-off distance tuned; after
   homing, `ActualLeftDeg ≈ 135.85`, `ActualRightDeg ≈ 44.15`.
4. Soft limits enforced; a move past −20/+200 is refused with `FaultCode 7`.
5. Manual handshake: from the app's **PLC tab → Connect PLC**, then **Robot
   Test → Enable → Home (find ref) → jog**. Confirm `CompleteCommandID` tracks
   each jog and `InPosition` gates it.
6. E-stop drops enable and forces State 900; `Reset` recovers.
7. Only then bring up the automatic pick/place sequence (§7).
```
