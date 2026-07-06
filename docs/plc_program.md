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

> **Grounded in the Teknic *ClearLink EtherNet/IP Setup and Object Data
> Reference*, Rev. 1.15.** The member/register names below are the real ones.

Motion goes CompactLogix (EtherNet/IP **scanner**) → **Teknic ClearLink
CLNK-4-13** (EtherNet/IP **adapter/slave**) → M-0/M-1 STEP/DIR/ENABLE →
**Leadshine EM806** → NEMA 23. Add the ClearLink to the I/O tree from its **EDS**
(→ **Add-On Profile**, module revision 2.091) and pick the **"Step Dir"**
connection type (the EM806 is a third-party step & direction drive; this is the
same connection type used for ClearPath-SD). Axis map: **Motor 0 = left shoulder,
Motor 1 = right shoulder** (M-2/M-3 spare). Z is pneumatic, not a ClearLink axis.

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

**Output (`:O1`, per motor — Step & Direction Motor Output Object, class `0x66`):**
| Member | Type | Use |
|---|---|---|
| `Move Distance` | DINT | target, in **steps** (absolute or incremental per bit 1) |
| `Velocity Limit` | UDINT | positional-move speed, steps/s (max 500,000) |
| `Acceleration Limit` | UDINT | steps/s² (min resolution 1527) |
| `Deceleration Limit` | UDINT | steps/s² (0 ⇒ use accel) |
| `Jog Velocity` | DINT | velocity-move speed, steps/s |
| `Output Register` | DWORD | command bits, below |

`Output Register` bits (Table 28): **0 Enable**, **1 Absolute Flag** (1 = absolute
move), **2 Homing Move Flag**, **3 Load Position Move** (rising edge loads &
executes a position move), **4 Load Velocity Move**, **5 SW E-Stop**, **6 Clear
Alerts** (clears the shutdown register), **7 Clear Motor Fault**.

**Input (`:I1`, per motor — Step & Direction Motor Input Object, class `0x65`):**
| Member | Type | Use |
|---|---|---|
| `Commanded Position` | DINT | **the position feedback** (open-loop: commanded == actual, in steps) |
| `Target Position` | DINT | where the loaded move ends |
| `Commanded Velocity` | DINT | current step rate |
| `Status Register` | DWORD | status bits, below |
| `Motor Shutdowns` | DWORD | latched cancel reasons (Table 25) |

`Status Register` bits (Table 24): **0 At Target Position** (needs HLFB — see
below), **1 Steps Active** (`Commanded Velocity ≠ 0` — the reliable *moving*
flag), **3 Move Direction**, **4/5 In Positive/Negative Limit**, **7 In Home
Sensor**, **9 Motor in Fault** (HLFB), **10 Enabled** (HLFB), **13 Has Homed**
(the reference-established bit), **16 Ready to Home**, **17 Shutdowns Present**,
**19 Load Position Move Ack** (move-load handshake), **20 Load Velocity Move
Ack**.

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

**Commanding one absolute move** (this replaces the invented "CmdPosition +
MoveTrigger + MoveDone" from earlier drafts):
1. **Enable:** set `Output Register.Enable` (bit 0); wait `Status.Enabled`
   (bit 10).
2. **Load targets:** write `Move Distance := ROUND(deg*STEPS_PER_DEG)`,
   `Velocity Limit`, `Acceleration Limit`, `Deceleration Limit`; set
   `Absolute Flag` (bit 1).
3. **Fire:** pulse `Load Position Move` (bit 3) — on the rising edge the
   ClearLink loads the move and sets `Load Position Move Ack` (status bit 19);
   drop bit 3 to clear the ack (that is the handshake — there is no CommandID at
   the ClearLink level).
4. **Move done** = `Steps Active` (bit 1) `== 0`. (`At Target Position`, bit 0,
   also requires an asserted HLFB, so on the HLFB-less EM806 use **Steps
   Active**.)

**Open-loop / third-party-drive reality** — the EM806 is a plain step/dir drive
with **no HLFB** (High-Level Feedback), so:
- `Commanded Position` is the only position source; there is no missed-step
  detection. Absolute position is trustworthy only relative to the last **home**
  — which is why homing and the 85 % reach / singularity margins (enforced in
  Python) matter.
- Several status bits are HLFB-derived — **At Target Position** (0), **Motor in
  Fault** (9), **Enabled** (10). With no HLFB wired you **must set `HLFB
  Inversion` (Config Register bit 3)** so HLFB reads as asserted; otherwise
  `Enabled` never comes true and `Motor in Fault` trips immediately. Set `Enable
  Inversion` (bit 2) if the EM806 enables on the opposite electrical sense.
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

These AOIs are **ours** — Teknic provides only the EDS/AOP (which creates the
assembly tags) plus a Micro800 *explicit-messaging* example. Each AOI just wraps
the ClearLink `:O1` / `:I1` assembly members from §3 so the rest of the program
never touches them directly.

### `AOI_AxisMove` — move one ClearLink axis to an absolute angle
```
INPUT   TargetDeg   REAL
INPUT   Execute     BOOL      // rising edge = start one move
IN_OUT  Ax          AxisIF    // alias onto <module>:O1/:I1 motor block (§3)
OUTPUT  InPosition  BOOL
OUTPUT  Fault       BOOL
LOCAL   prevExec    BOOL

// rising edge: load the move and pulse Load Position Move
IF Execute AND NOT prevExec AND NOT Fault THEN
    Ax.MoveDistance   := ROUND(TargetDeg * STEPS_PER_DEG);
    Ax.VelocityLimit  := MOVE_VEL;          // steps/s (<= 500000)
    Ax.AccelLimit     := MOVE_ACC;
    Ax.DecelLimit     := MOVE_DEC;
    Ax.OutReg.1       := TRUE;              // Absolute Flag
    Ax.OutReg.3       := TRUE;              // Load Position Move (rising edge)
END_IF;
prevExec := Execute;

// clear the load bit once the ClearLink acknowledges (handshake, status bit 19)
IF Ax.StatusReg.19 THEN Ax.OutReg.3 := FALSE; END_IF;

// open-loop move-done = Steps Active (bit 1) went back to 0 after a load
InPosition := NOT Ax.StatusReg.1 AND NOT Ax.OutReg.3;
Fault      := Ax.StatusReg.17 OR Ax.ALM;      // ClearLink shutdown OR EM806 ALM (via DIP)
```
> `Ax.OutReg.n` / `Ax.StatusReg.n` are the Output/Status **Register** bits from
> §3. `AbsoluteFlag`=1, `LoadPositionMove`=3, `StepsActive`=1, `LoadPositionMoveAck`=19.

### `AOI_HomeAxis` — reference one shoulder (ClearLink runs the homing)
**The ClearLink executes homing internally** — you do *not* hand-roll a
fast/back-off/slow jog. One-time config (Configuration assembly, §3): set the
motor's `Home Sensor` connector to the shoulder prox input, `Config Register`
`Homing Enable` (bit 0) = 1, and `Home Sensor Active Level` (bit 1) to match the
prox. Then:
```
INPUT   Execute       BOOL     // rising edge = home this axis
IN_OUT  Ax            AxisIF
OUTPUT  Homed         BOOL
OUTPUT  Fault         BOOL     // timeout / not ready

// 1. require Ready to Home (status bit 16: homing enabled, motor enabled,
//    no shutdowns, valid sensor)
// 2. rising edge -> load a slow homing move toward the switch:
//       Ax.OutReg.0 := TRUE;        // Enable
//       Ax.OutReg.2 := TRUE;        // Homing Move Flag
//       Ax.JogVelocity := HOME_VEL; // slow, toward the switch
//       Ax.OutReg.4 := TRUE;        // Load Velocity Move (pulse; ack = status 20)
// 3. ClearLink drives to the sensor, cancels motion, and zeroes position there
//    (Commanded Position := 0 at the switch).
// 4. Homed := Ax.StatusReg.13 (Has Homed).  Timeout -> Fault.
```
**Home-offset gotcha:** ClearLink zeroes position **at the switch trip point**,
so `Commanded Position = 0` means "at the home prox", *not* 135.85°/44.15°. Apply
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
independently. **The ClearLink runs the homing move itself** (§3, §5) — the PLC
configures the home sensor + homing-enable, commands a homing move per axis, and
waits for the `Has Homed` status bit.

```
One-time config (per axis, Configuration assembly):
  Home Sensor connector := <prox input pin>      // 0..12 = local ClearLink input
  Config Register.HomingEnable (bit0) := 1
  Config Register.HomeSensorActiveLevel (bit1)   // to match the prox
  Config Register.HLFBInversion (bit3) := 1      // no HLFB on the EM806 (§3)

Per axis (AOI_HomeAxis):
  1. Enable drives; wait Status.Enabled.
  2. Require Status.ReadyToHome (bit16).
  3. Command a slow homing move toward the switch:
        OutReg.Enable(0)=1, OutReg.HomingMoveFlag(2)=1,
        JogVelocity := HOME_VEL, then pulse OutReg.LoadVelocityMove(4).
  4. ClearLink drives to the sensor, cancels motion, and sets position 0 there.
  5. Wait Status.HasHomed (bit13).   Timeout -> Fault(FC_HOME_TIMEOUT).

After both axes homed: enable soft limits (Config Register.SoftLimitEnable +
Soft Limit 1/2), set Status.Homed := TRUE, and publish
  ActualDeg := (CommandedPosition + HOME_OFFSET_STEPS) / STEPS_PER_DEG.
```

**Home-offset:** the ClearLink datum is the switch trip point (position 0), not
the home angle — apply `HOME_OFFSET_STEPS` per axis so the published angle reads
135.85° (left) / 44.15° (right). See §5 and `docs/homing.md`.

Home is **mid-travel**, so the flag passes the sensor during normal motion. The
ClearLink only consults the home sensor during a homing move (a move with the
Homing Move Flag set), so it is safely ignored otherwise. Hard limits at
−20°/+200° stay wired into the drive fault chain at all times, and can also be
given to the ClearLink as `Positive/Negative Limit` connectors (§3).

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

1. ClearLink added from its **EDS/AOP**, **"Step Dir"** connection type; Motor
   0/1 map to left/right; `HLFB Inversion` + (if needed) `Enable Inversion` set
   for the EM806; EM806 configured for **1 µs / 500 kHz** step pulses;
   `STEPS_PER_DEG = 26.6667` verified (command 90°, measure the shoulder).
2. Home prox on L1 (r≈40) wired to a ClearLink input and set as each motor's
   `Home Sensor` connector; hard limits (−20/+200) in the drive fault chain (and
   optionally as `Positive/Negative Limit` connectors); EM806 ALM read as a fault.
3. Homing: `Homing Enable` set, `HOME_VEL`/approach direction tuned, and
   `HOME_OFFSET_STEPS` set so after homing `ActualLeftDeg ≈ 135.85`,
   `ActualRightDeg ≈ 44.15`. Confirm `Has Homed` asserts.
4. Soft limits enforced; a move past −20/+200 is refused with `FaultCode 7`.
5. Manual handshake: from the app's **PLC tab → Connect PLC**, then **Robot
   Test → Enable → Home (find ref) → jog**. Confirm `CompleteCommandID` tracks
   each jog and `InPosition` gates it.
6. E-stop drops enable and forces State 900; `Reset` recovers.
7. Only then bring up the automatic pick/place sequence (§7).
```
