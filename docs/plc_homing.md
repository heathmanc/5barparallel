# Homing routine — Studio 5000 build sheet

Complete, near-drop-in implementation of the homing sequence outlined in
`docs/plc_program.md` §8. Two parts:

- **`AOI_HomeAxis`** — an Add-On Instruction that homes **one** shoulder by
  commanding the **ClearLink's built-in homing move** and waiting for `Has
  Homed`.
- **`R30_Homing`** — the coordinator that homes both shoulders **sequentially**
  and publishes `VisionRobot.Status.Homed`.

> **Corrected against the Teknic *ClearLink EtherNet/IP Object Data Reference*,
> Rev. 1.15.** Earlier drafts hand-rolled a fast/back-off/slow jog state machine
> in ladder — that is **wrong for the ClearLink**, which performs the homing move
> itself (Step & Direction Motor Output/Status objects, class `0x66`/`0x65`). The
> PLC configures the home sensor + homing-enable, commands **one** homing move,
> and polls `Has Homed`. See `docs/plc_program.md` §3 for the assembly map.

Mechanical placement of the switches is in `docs/homing.md`; the reference
values come from the `homing:` block in `config/robot_config.yaml`.

---

## 1. Tags & constants to create

**Controller tags**

| Tag | Type | Notes |
|---|---|---|
| `Ax0`, `Ax1` | `AxisIF` (alias, below) | left / right shoulder Step-Dir assembly block |
| `Ax0_HomeReq`, `Ax1_HomeReq` | BOOL | per-axis "run homing" request |
| `Ax0_HomeDone`, `Ax1_HomeDone` | BOOL | per-axis homed |
| `Ax0_HomeFault`, `Ax1_HomeFault` | BOOL | per-axis fault |
| `HomeAxis0`, `HomeAxis1` | `AOI_HomeAxis` | AOI backing tags (one per axis) |
| `HomeStep` | DINT | coordinator state |
| `HR_prev` | BOOL | HomeRequest edge-detect storage |
| `SoftLimitsEnable` | BOOL | mirror of per-axis `Config Register.SoftLimitEnable` |

> The **home prox switches wire to ClearLink inputs**, not PLC tags — each motor's
> `Home Sensor` connector (Configuration assembly) points at its prox, and the
> ClearLink reads it during the homing move. You may still read those inputs via
> the Discrete Input Point object for HMI/diagnostics, but the homing logic does
> not need `HS_Left`/`HS_Right` PLC tags.

**Tuning constants** (starting values — verify on the bench)

| Constant | Type | Suggested | Meaning |
|---|---|---|---|
| `STEPS_PER_DEG` | REAL | `26.66667` | 3200 × 3 / 360 |
| `HOME_VEL` | DINT | `800` | homing-move speed toward the switch, steps/s (~30°/s) |
| `HOME_ACC` | DINT | `20000` | homing accel, steps/s² |
| `HOME_TIMEOUT` | DINT | `10000` | homing-move timeout, ms |
| `HOME_OFFSET_L` | DINT | `ROUND(135.8504*26.66667)` | switch angle → steps, left |
| `HOME_OFFSET_R` | DINT | `ROUND(44.1496*26.66667)` | switch angle → steps, right |

> **`HOME_VEL` sets repeatability.** The ClearLink zeroes position when the home
> sensor trips during a homing move; a slower approach = a more repeatable datum.
> A single slow approach is enough — you do **not** need the old back-off /
> re-approach dance (that was a hand-rolled workaround the ClearLink doesn't need).

**Axis interface UDT `AxisIF`** — an alias/wrapper over the ClearLink Step-Dir
assembly members for one motor (`<module>:O1` / `:I1`, `docs/plc_program.md` §3).
Map each member to the real assembly tag; `*.n` are register bits:

| Member | Type | Real ClearLink member (Step & Direction) |
|---|---|---|
| `MoveDistance` | DINT | Output · Move Distance (steps) |
| `JogVelocity` | DINT | Output · Jog Velocity (steps/s) |
| `AccelLimit` | UDINT | Output · Acceleration Limit |
| `OutReg` | DWORD | Output · **Output Register** — `.0` Enable, `.1` Absolute, `.2` HomingMoveFlag, `.3` LoadPositionMove, `.4` LoadVelocityMove, `.6` ClearAlerts |
| `CmdPosition` | DINT | Input · Commanded Position (open-loop position) |
| `StatusReg` | DWORD | Input · **Status Register** — `.1` StepsActive, `.10` Enabled, `.13` HasHomed, `.16` ReadyToHome, `.17` ShutdownsPresent, `.20` LoadVelMoveAck |
| `ALM` | BOOL | EM806 alarm, wired to a ClearLink digital input (read via the DIP object) |

> Open-loop reminder (`docs/plc_program.md` §3): `CmdPosition` is the ClearLink's
> *commanded* step count, not encoder feedback. The ClearLink homing move
> establishes the datum (position 0 **at the switch**); step integrity is assumed
> thereafter. The home *angle* is applied by `HOME_OFFSET_*` when publishing
> `ActualDeg`, since position 0 = the prox trip point, not 135.85°/44.15°.

---

## 2. `AOI_HomeAxis` — per-shoulder homing (commands the ClearLink homing move)

**Prerequisite — one-time Configuration assembly (`<module>:C`, per motor):**
set `Home Sensor` connector = the ClearLink input the shoulder prox is wired to,
`Config Register.HomingEnable` (bit 0) = 1, `Config Register.HomeSensorActiveLevel`
(bit 1) to match the prox, and `Config Register.HLFBInversion` (bit 3) = 1 (the
EM806 has no HLFB — `docs/plc_program.md` §3). These are sent once when the
EtherNet/IP connection is established.

**Parameters:** `In` HomeReq, HomeVel, HomeAccel, TimeoutPreset · `InOut` Ax
(`AxisIF`) · `Out` Done, Fault · `Local` Step (DINT), HomeTmr (TIMER),
prevReq (BOOL).

### Ladder

> ⚠️ **`plc_homing_axis_ladder.svg` shows the superseded hand-rolled jog state
> machine and must not be built as-is.** Rebuild the rungs from the corrected
> Structured Text below (it is short — three states). The coordinator ladder in
> §3 is still valid.

### Structured Text (drop-in)

```pascal
(* AOI_HomeAxis — command the ClearLink's built-in homing move for one shoulder.
   Homing itself (approach, sensor detection, zeroing) is done by ClearLink. *)

HomeTmr.PRE := TimeoutPreset;
HomeTmr.TimerEnable := (Step = 10);      (* time only the homing move *)
TONR(HomeTmr);

CASE Step OF
    0:  (* idle — wait for a request, and for the axis to be ready to home *)
        Done := 0;
        IF HomeReq AND NOT prevReq AND NOT Fault THEN
            IF Ax.StatusReg.16 THEN          (* Ready to Home *)
                (* load one slow homing velocity move toward the switch *)
                Ax.OutReg.0 := 1;            (* Enable *)
                Ax.OutReg.2 := 1;            (* Homing Move Flag *)
                Ax.JogVelocity := HomeVel;   (* signed: toward the prox *)
                Ax.AccelLimit  := HomeAccel;
                Ax.OutReg.4 := 1;            (* Load Velocity Move (rising edge) *)
                Step := 10;
            ELSE
                Fault := 1;  Step := 900;    (* not ready: homing not enabled,
                                                not enabled, or shutdown present *)
            END_IF;
        END_IF;

    10: (* homing move running — ClearLink drives to the sensor and zeroes there *)
        IF Ax.StatusReg.20 THEN Ax.OutReg.4 := 0; END_IF;   (* clear load ack *)
        IF Ax.StatusReg.13 THEN              (* Has Homed -> datum established *)
            Ax.OutReg.2 := 0;                (* drop Homing Move Flag *)
            Done := 1;
            Step := 50;
        ELSIF HomeTmr.DN THEN
            Fault := 1;  Step := 900;
        END_IF;

    50: (* homed / idle — hold *)
        ;

    900:(* fault — clear move bits, wait for coordinator reset *)
        Ax.OutReg.2 := 0;  Ax.OutReg.4 := 0;
END_CASE;

prevReq := HomeReq;

(* drive alarm or a ClearLink shutdown at any time -> fault *)
IF Ax.ALM OR Ax.StatusReg.17 THEN
    Fault := 1;  Step := 900;
END_IF;
```

`Ax.JogVelocity` is **signed** — its sign sets the approach direction, so flip
the sign per shoulder without touching the logic. The ClearLink stops the move
and sets `Commanded Position := 0` the instant the home sensor trips; the home
*angle* is applied by `HOME_OFFSET_*` when the coordinator publishes `ActualDeg`
(§3).

---

## 3. `R30_Homing` — coordinator (both shoulders, sequential)

Homes Axis 0 (left), then Axis 1 (right), then sets `Status.Homed`. Sequential,
not simultaneous: only one proximal link sweeps at a time, so the two arms can't
drive into each other. Verify approach directions give a collision-free sweep
from any startup pose (§5).

### Ladder

![R30_Homing ladder](plc_homing_coord_ladder.svg)

> The sequencing (home Ax0 → Ax1 → publish) is still correct, but the **AOI-call
> parameters shown in this SVG predate the ClearLink correction** — use the
> parameter list in the Structured Text below (`HomeVel`/`HomeAccel`, no
> `HomeSwitch`/`HomeAngleDeg`), and the offset-aware `ActualDeg` publish.

### Structured Text (equivalent)

```pascal
(* R30_Homing — sequential homing coordinator *)

(* rising-edge detect on the vision-PC HomeRequest *)
HR_edge := VisionRobot.Manual.HomeRequest AND NOT HR_prev;
HR_prev := VisionRobot.Manual.HomeRequest;

CASE HomeStep OF
    0:  IF HR_edge AND VisionRobot.Status.Enabled
             AND NOT VisionRobot.Status.Faulted THEN
            VisionRobot.Status.Homed := 0;
            Ax0_HomeReq := 1;                 (* start left *)
            HomeStep := 10;
        END_IF;

    10: IF Ax0_HomeDone THEN                  (* left done -> start right *)
            Ax0_HomeReq := 0;
            Ax1_HomeReq := 1;
            HomeStep := 20;
        END_IF;

    20: IF Ax1_HomeDone THEN                  (* right done *)
            Ax1_HomeReq := 0;
            HomeStep := 30;
        END_IF;

    30: (* both homed — publish with the home offset, enable soft limits, idle *)
        VisionRobot.Status.ActualLeftDeg  := (Ax0.CmdPosition + HOME_OFFSET_L) / STEPS_PER_DEG;
        VisionRobot.Status.ActualRightDeg := (Ax1.CmdPosition + HOME_OFFSET_R) / STEPS_PER_DEG;
        VisionRobot.Status.Homed := 1;
        SoftLimitsEnable := 1;              (* Config Register.SoftLimitEnable per axis *)
        HomeStep := 0;

    900:(* homing fault — hold until Status.Faulted is cleared elsewhere *)
        ;
END_CASE;

(* run the axis AOIs every scan; they self-idle at Step 0/50.
   HomeVel is signed: +toward the prox on Ax0, sign per shoulder on Ax1. *)
AOI_HomeAxis(HomeAxis0, HomeReq:=Ax0_HomeReq, HomeVel:=HOME_VEL,
             HomeAccel:=HOME_ACC, TimeoutPreset:=HOME_TIMEOUT, Ax:=Ax0);
Ax0_HomeDone  := HomeAxis0.Done;
Ax0_HomeFault := HomeAxis0.Fault;

AOI_HomeAxis(HomeAxis1, HomeReq:=Ax1_HomeReq, HomeVel:=-HOME_VEL,
             HomeAccel:=HOME_ACC, TimeoutPreset:=HOME_TIMEOUT, Ax:=Ax1);
Ax1_HomeDone  := HomeAxis1.Done;
Ax1_HomeFault := HomeAxis1.Fault;

(* fault rollup -> FaultCode 4 (homing) *)
IF Ax0_HomeFault OR Ax1_HomeFault THEN
    VisionRobot.Status.Faulted   := 1;
    VisionRobot.Status.FaultCode := 4;
    HomeStep := 900;
END_IF;
```

> In the ladder, the AOI calls are gated by `Ax0_HomeReq` / `Ax1_HomeReq` — a
> harmless optimization, since the AOI idles at Step 0/50 anyway. Calling every
> scan (as the ST does) is equivalent and simpler.

Call `R30_Homing` from `R00_Main` every scan. `Manual.HomeRequest` (from the
GUI's **Home (find ref)**) triggers it; the automatic sequence (§7) should also
require `Status.Homed` before its first run.

---

## 4. How this satisfies the Python handshake

`PlcRobotDriver.home()` pulses `VisionRobot.Manual.HomeRequest` and waits for
`VisionRobot.Status.Homed`. This routine: edge-detects that pulse (rung 0),
runs both axes, and sets `Status.Homed` + `ActualLeft/RightDeg` on success
(rung 5) — exactly what the driver polls for. On failure it sets `Status.Faulted`
+ `FaultCode = 4`, which the driver surfaces as a `RobotDriverError`.

---

## 5. Commissioning the homing routine

1. In MSP/config, confirm each motor's `Home Sensor` connector points at the
   right prox input and `Homing Enable` (Config Register bit 0) is set. Read the
   prox via the DIP object and confirm it toggles when you pass the L1 flag.
2. Jog each axis manually and confirm the **sign of `HOME_VEL`** drives it
   **toward** its home prox. Flip the sign per shoulder if reversed.
3. Verify the homing sweep from any startup pose does **not** collide the two
   arms (sequential homing means the idle arm is held). Adjust approach direction
   or pre-park if a sweep is unsafe.
4. Tune `HOME_VEL`: slower approach = more repeatable datum. Re-home a few times
   and confirm the switch trip (`Has Homed`) repeats to your tolerance.
5. Set `HOME_OFFSET_L`/`HOME_OFFSET_R` so that after homing
   `Status.ActualLeftDeg ≈ 135.85`, `Status.ActualRightDeg ≈ 44.15` (position 0
   is the prox trip point, not the home angle — the offset bridges the two).
6. Block a prox so the switch never trips to confirm `HOME_TIMEOUT` →
   `FaultCode 4`.
7. Confirm the soft limits (−20/+200, `Config Register.SoftLimitEnable` + `Soft
   Limit 1/2`) go active only after `Status.Homed`.
```
