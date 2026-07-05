# Homing routine — Studio 5000 build sheet

Complete, near-drop-in implementation of the homing sequence outlined in
`docs/plc_program.md` §8. Two parts:

- **`AOI_HomeAxis`** — an Add-On Instruction that homes **one** shoulder to its
  L1 flag switch (fast approach → back off → slow re-approach → set reference).
- **`R30_Homing`** — the coordinator that homes both shoulders **sequentially**
  and publishes `VisionRobot.Status.Homed`.

Each is given as a **ladder logic visual** (rebuild it rung-by-rung) and the
**equivalent Structured Text** (paste directly into an ST routine/AOI). The two
forms are identical logic — use whichever your routine type is.

Mechanical placement of the switches is in `docs/homing.md`; the reference
values come from the `homing:` block in `config/robot_config.yaml`.

---

## 1. Tags & constants to create

**Controller tags**

| Tag | Type | Notes |
|---|---|---|
| `Ax0`, `Ax1` | `AxisIF` (UDT, below) | left / right shoulder interface to the ClearLink |
| `HS_Left`, `HS_Right` | BOOL | home flag prox inputs (L1 tab, per `docs/homing.md`) |
| `Ax0_HomeReq`, `Ax1_HomeReq` | BOOL | per-axis "run homing" request |
| `Ax0_HomeDone`, `Ax1_HomeDone` | BOOL | per-axis homed |
| `Ax0_HomeFault`, `Ax1_HomeFault` | BOOL | per-axis fault |
| `HomeAxis0`, `HomeAxis1` | `AOI_HomeAxis` | AOI backing tags (one per axis) |
| `HomeStep` | DINT | coordinator state |
| `HR_prev` | BOOL | HomeRequest edge-detect storage |
| `SoftLimitsEnable` | BOOL | gate the −20/+200 soft-limit checks |

**Tuning constants** (starting values — verify on the bench)

| Constant | Type | Suggested | Meaning |
|---|---|---|---|
| `STEPS_PER_DEG` | REAL | `26.66667` | 3200 × 3 / 360 |
| `FAST_VEL` | DINT | `800` | fast approach, steps/s (~30°/s) |
| `SLOW_VEL` | DINT | `80` | slow approach / back-off, steps/s (~3°/s) |
| `BACKOFF_STEPS` | DINT | `55` | back-off distance (~2°), must clear the switch |
| `HOME_TIMEOUT` | DINT | `10000` | per-approach timeout, ms |

**Axis interface UDT `AxisIF`** — thin wrapper over the ClearLink AOP assembly
(member names vary by AOP revision; map each to the real assembly member):

| Member | Type | Maps to |
|---|---|---|
| `JogHome` | BOOL | jog in the home-approach direction |
| `JogBack` | BOOL | jog in the opposite direction |
| `JogVel` | DINT | jog velocity command (steps/s) |
| `ActualSteps` | DINT | current commanded step count (open-loop) |
| `Redefine` | BOOL | pulse to redefine current position |
| `PosRef` | DINT | value written on redefine (steps) |
| `Alarm` | BOOL | drive alarm (EM806 ALM via ClearLink input) |

> Open-loop reminder (`docs/plc_program.md` §3): `ActualSteps` is the ClearLink's
> *commanded* count, not encoder feedback. Homing establishes the datum; step
> integrity is assumed thereafter.

---

## 2. `AOI_HomeAxis` — per-shoulder homing state machine

**Parameters:** `In` HomeReq, HomeSwitch, HomeAngleDeg, FastVel, SlowVel,
BackoffSteps, TimeoutPreset, StepsPerDeg · `InOut` Ax (`AxisIF`) · `Out` Done,
Fault · `Local` Step (DINT), HomeTmr (TIMER), BackoffStart (DINT),
BackMoved (DINT).

### Ladder

![AOI_HomeAxis ladder](plc_homing_axis_ladder.svg)

### Structured Text (equivalent)

```pascal
(* AOI_HomeAxis — home one shoulder to its L1 flag switch *)

(* one timer, enabled only during the two approaches *)
HomeTmr.PRE := TimeoutPreset;
HomeTmr.TimerEnable := (Step = 10) OR (Step = 30);
TONR(HomeTmr);

CASE Step OF
    0:  (* idle — wait for a request *)
        Done := 0;
        IF HomeReq AND NOT Fault THEN
            Step := 10;
        END_IF;

    10: (* fast approach toward the switch *)
        Ax.JogHome := 1;  Ax.JogBack := 0;  Ax.JogVel := FastVel;
        IF HomeSwitch THEN
            Ax.JogHome := 0;
            BackoffStart := Ax.ActualSteps;      (* capture trip position *)
            Step := 20;
        ELSIF HomeTmr.DN THEN
            Fault := 1;  Step := 900;
        END_IF;

    20: (* back off far enough to clear the switch *)
        Ax.JogBack := 1;  Ax.JogHome := 0;  Ax.JogVel := SlowVel;
        BackMoved := ABS(Ax.ActualSteps - BackoffStart);
        IF BackMoved >= BackoffSteps THEN
            Ax.JogBack := 0;
            Step := 30;
        END_IF;

    30: (* slow re-approach — this switch edge is the datum *)
        Ax.JogHome := 1;  Ax.JogBack := 0;  Ax.JogVel := SlowVel;
        IF HomeSwitch THEN
            Ax.JogHome := 0;
            Ax.PosRef  := TRUNC(HomeAngleDeg * StepsPerDeg);
            Ax.Redefine := 1;                    (* set current pos = home *)
            Step := 40;
        ELSIF HomeTmr.DN THEN
            Fault := 1;  Step := 900;
        END_IF;

    40: (* reference latched -> axis homed *)
        Ax.Redefine := 0;
        Done := 1;
        Step := 50;

    50: (* homed / idle — hold *)
        ;

    900:(* fault — motion off, wait for coordinator reset *)
        Ax.JogHome := 0;  Ax.JogBack := 0;
END_CASE;

(* drive alarm at any time -> fault *)
IF Ax.Alarm THEN
    Fault := 1;  Step := 900;
END_IF;
```

Back-off uses `ABS(ActualSteps - BackoffStart)` so it is **direction-agnostic**
— flip `JogHome`/`JogBack` wiring per shoulder without touching the logic.

---

## 3. `R30_Homing` — coordinator (both shoulders, sequential)

Homes Axis 0 (left), then Axis 1 (right), then sets `Status.Homed`. Sequential,
not simultaneous: only one proximal link sweeps at a time, so the two arms can't
drive into each other. Verify approach directions give a collision-free sweep
from any startup pose (§5).

### Ladder

![R30_Homing ladder](plc_homing_coord_ladder.svg)

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

    30: (* both homed — publish, enable soft limits, idle *)
        VisionRobot.Status.ActualLeftDeg  := Ax0.ActualSteps / STEPS_PER_DEG;
        VisionRobot.Status.ActualRightDeg := Ax1.ActualSteps / STEPS_PER_DEG;
        VisionRobot.Status.Homed := 1;
        SoftLimitsEnable := 1;
        HomeStep := 0;

    900:(* homing fault — hold until Status.Faulted is cleared elsewhere *)
        ;
END_CASE;

(* run the axis AOIs every scan; they self-idle at Step 0/50 *)
AOI_HomeAxis(HomeAxis0, HomeReq:=Ax0_HomeReq, HomeSwitch:=HS_Left,
             HomeAngleDeg:=135.8504, FastVel:=FAST_VEL, SlowVel:=SLOW_VEL,
             BackoffSteps:=BACKOFF_STEPS, TimeoutPreset:=HOME_TIMEOUT,
             StepsPerDeg:=STEPS_PER_DEG, Ax:=Ax0);
Ax0_HomeDone  := HomeAxis0.Done;
Ax0_HomeFault := HomeAxis0.Fault;

AOI_HomeAxis(HomeAxis1, HomeReq:=Ax1_HomeReq, HomeSwitch:=HS_Right,
             HomeAngleDeg:=44.1496, FastVel:=FAST_VEL, SlowVel:=SLOW_VEL,
             BackoffSteps:=BACKOFF_STEPS, TimeoutPreset:=HOME_TIMEOUT,
             StepsPerDeg:=STEPS_PER_DEG, Ax:=Ax1);
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

1. Jog each axis manually; confirm `Ax.JogHome` drives **toward** its home switch
   and `Ax.JogBack` away. Fix wiring/sign if reversed.
2. Verify the fast sweep from any startup pose does **not** collide the two arms
   (sequential homing means the idle arm is held). Adjust approach direction or
   pre-park if a sweep is unsafe.
3. Tune `FAST_VEL` / `SLOW_VEL` / `BACKOFF_STEPS`; slower re-approach = more
   repeatable datum.
4. After homing, `Status.ActualLeftDeg ≈ 135.85`, `Status.ActualRightDeg ≈ 44.15`
   (from the config `homing` block). Re-home a few times and confirm the datum
   repeats to your tolerance.
5. Trip a home switch by hand mid-approach to confirm the datum sets and `Done`
   latches; block a switch to confirm `HOME_TIMEOUT` → `FaultCode 4`.
6. Confirm the soft limits (−20/+200) go active only after `Status.Homed`.
```
