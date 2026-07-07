# PLC setup guide — CompactLogix + Teknic ClearLink + EM806

Step-by-step bring-up of the PLC side of the 5-bar robot: network the ClearLink,
add it to Studio 5000, **import Teknic's example projects**, build the two
shoulder axes from them, create the `VisionRobot` tags the vision PC talks to,
and commission.

This is the "do this, in this order" guide. The **why/behaviour** lives in
[`plc_program.md`](plc_program.md) (architecture, UDT, faults), the homing build
sheet in [`plc_homing.md`](plc_homing.md), the ladder visuals in
[`plc_ladder.md`](plc_ladder.md), and switch placement in [`homing.md`](homing.md).

---

## 0. Order of operations (TL;DR)

1. **Wire** the ClearLink → EM806 (step/dir/enable), the home prox → a ClearLink
   input, EM806 ALM → a ClearLink input, hard limits + E-stop (§1).
2. **Configure the EM806** for 1 µs / 500 kHz step pulses (§1).
3. **Give the ClearLink a static IP** with a Rockwell/Molex tool (§2).
4. **Import the ClearLink EDS**, add the module as **"Step Dir"** (§3).
5. **Import Teknic's example `.L5K`** (reference) and **this repo's complete
   program** in order: the five `#_VisionRobot*.L5X` UDT files → `RobotTags.csv` →
   all the routines (`R00_Main` … `R60_Status`, `R_Move/Home*`), then set
   `R00_Main` as the Program's Main (§4).
6. **Set the Configuration assembly** (home sensor, homing enable, HLFB/Enable
   inversion, soft limits) (§5).
7. **Finish the `VisionRobot` glue** — alias `EM806_*_ALM` to the drive-alarm
   inputs, set `HOME_OFFSET_L/R`, and tie the routines into your task (§6).
8. **Commission** in the order in §7.

**Hardware:** CompactLogix (e.g. 1769-L16ER, EtherNet/IP scanner) · Teknic
ClearLink **CLNK-4-13** (EtherNet/IP adapter) · 2× Leadshine **EM806** stepper
drives · 2× NEMA 23 steppers · 2× inductive home prox · 4× hard-limit switches ·
E-stop. **Software:** Studio 5000 Logix Designer · the ClearLink EDS +
CompactLogix examples (the `ClearLink_Examples` pack) · a Rockwell BootP/DHCP or
EtherNet/IP Address Commissioning tool.

---

## 1. Wiring & drive config

Motion path: **CompactLogix →(EtherNet/IP)→ ClearLink →(step/dir/enable)→ EM806
→ NEMA 23.** Axis map: **M-0 = left shoulder, M-1 = right shoulder** (M-2/M-3
spare). Z is pneumatic, not a ClearLink axis.

Per shoulder, wire to the ClearLink:
- **M-0 / M-1 connector** → EM806 PUL (step), DIR, ENA (enable). Each motor
  connector has step + dir + enable outputs and one input.
- **Home prox** → a ClearLink digital input (`I/O-0…5` or `DI-6…8`). Note the pin
  number — it becomes the `Home Sensor` connector in §5.
- **EM806 ALM (alarm)** → another ClearLink digital input. The ClearLink's own
  "Motor Fault" is HLFB-based and will *not* see the EM806 alarm, so you read ALM
  yourself via the Discrete Input Point object.
- **Hard limits (−20° / +200°, per shoulder)** → into the drive **enable/fault
  chain** (so they act even if logic hangs); optionally also to ClearLink inputs
  wired as the `Positive/Negative Limit` connectors.
- **E-stop** → a hardware safety relay that drops drive power; mirror it to a PLC
  input.

**EM806 DIP/config:** set microstepping to give **3200 pulses/rev** (the
`STEPS_PER_DEG = 26.6667` assumes 3200 × 3:1 / 360). The ClearLink emits a **fixed
1 µs step pulse at up to 500 kHz** — set the EM806 to accept that pulse width.

---

## 2. Give the ClearLink a static IP

The ClearLink ships in DHCP mode with no fixed address (ClearLink EtherNet/IP
Object Reference, §Network Configuration). Assign a static IP once:

1. Put the ClearLink on a network **with a DHCP server** (it won't take an address
   over a direct/isolated link). Power it with 24 VDC.
2. Use a Rockwell tool — **BootP/DHCP EtherNet/IP Commissioning Tool** *or*
   **EtherNet/IP Address Commissioning Tool** (ClearLink MACs start `24:15:10:B`).
   Assign the desired IP, then **Disable BOOTP/DHCP** / set **Static Mode** so it
   persists.
3. Note the IP (e.g. `192.168.1.10`) — you'll use it for the module and for the
   vision PC's pycomm3 path (`IP/slot`, e.g. `192.168.1.10/0`).

> If you lose the address: double-press the ClearLink reset button and power-cycle
> to return it to DHCP.

---

## 3. Add the ClearLink to the I/O tree

1. In Studio 5000: **Tools → EDS Hardware Installation Tool** → register
   `clearlink_2.92.eds` (from the CompactLogix example pack).
2. Right-click the EtherNet/IP scanner → **New Module** → pick ClearLink → set its
   **IP** (from §2) and name the module **`ClearLink`** (the examples and this
   repo's docs assume that name → tags come out as `ClearLink:I1.*` etc.).
3. In **Module Definition**, choose the **"Step Dir"** connection type (module
   revision **2.091**). *Not* M-Connector — that's for ClearPath-MC servos.
4. Download. You now have three auto-created assembly tags:
   `ClearLink:I1` (input/feedback), `ClearLink:O1` (output/commands),
   `ClearLink:C` (configuration). **You do not create these — the AOP does.**

---

## 4. Import Teknic's example projects (and what each is for)

Teknic ships working CompactLogix examples — **build from these instead of typing
ladder from scratch.** Import the `.L5K` (File → Open, pick the `.L5K`), then copy
the routine into your project and retarget its tags to your `ClearLink` module.

| Example `.L5K` | Use it for | Notes |
|---|---|---|
| **`SD_Homing`** | the homing routines (`R_HomeMotor0/1`, sequenced by `R30_Homing`) | one motor each; the repo ships both — see the `.L5X` table below |
| **`SD_Position_Move`** | moving a shoulder to an **absolute angle** (`R_MoveMotor0/1`) | the example moves *incrementally* — the repo routines set `Abs_Flag` for absolute (§`plc_program.md` §3) |
| **`SD_Jog`** | manual velocity jogging during bring-up | handy to confirm direction/wiring before homing |
| **`SD_Velocity_Move`** | reference for velocity moves | not needed for the pick/place cycle |

**Adapting to two axes:** each example drives **Motor 0**. For the robot,
duplicate the logic for **Motor 1** (swap `Motor0_*` → `Motor1_*`), assign
Motor 0 = left / Motor 1 = right, and drive both from the coordinator/dispatcher
in `plc_program.md`. The corrected per-axis logic and ladder visuals (matched to
these examples) are in `plc_homing.md` and `plc_ladder.md`.

### Importable files in this repo (`docs/l5x/`)

The repo ships the whole motion side ready to import so you don't re-type it.
**Import in this order** — each group needs what the previous one creates (the
`VisionRobot` tag needs its UDT; the routines need both):

**1 — the UDT, five files, in numeric order** (`Assets → Data Types →
right-click → Import…`). Import each separately — Logix does **not** reliably
create the nested types from one combined file, so they're split, leaves first:

| `docs/l5x/…` (import in order) | Creates data type |
|---|---|
| `1_VisionRobot_Cmd.L5X` | `VisionRobot_Cmd` |
| `2_VisionRobot_Target.L5X` | `VisionRobot_Target` |
| `3_VisionRobot_Manual.L5X` | `VisionRobot_Manual` |
| `4_VisionRobot_Status.L5X` | `VisionRobot_Status` |
| `5_VisionRobot.L5X` | `VisionRobot` (references the four above — import it **last**) |

**2 — the tags, as a CSV** (`Tools → Import → Tags and Logic Comments…`, **after**
the UDT exists — the file is a Rockwell tag CSV, not L5X). These are all
**controller-scope** so both the vision PC and every program routine can reach them.

> ✅ **The file matches a real Logix v34 tag export byte-for-byte** — the
> `remark` preamble, the `0.3` format-version line before the header (Logix
> imports nothing without it), an **empty** `SCOPE` column for controller tags,
> quoted `DESCRIPTION`/`DATATYPE`/`SPECIFIER`/`ATTRIBUTES`, and **CRLF** line
> endings (marked `-text` in `.gitattributes` so git won't strip them). If you
> hand-edit it, keep Windows (CRLF) endings and that exact structure, or the
> import silently does nothing.

| `docs/l5x/…` | Creates (controller scope) |
|---|---|
| `RobotTags.csv` | **every internal tag the whole program uses** — not just the motion glue. The `VisionRobot` tag; per-axis move/home glue (`Move*/Home*/Ax*`, `EM806_*_ALM`); homing coordinator (`HomeStep`, `HR_ons`, `SoftLimitsEnable`); safety (`EStop_*`, `Guard_Closed`, `Ax*_Limit*`, `SafetyOK`, `EnableReq`); manual (`AutoMode`, `WithinLimits`, `MoveActive`, `MTT_prev`); the auto pick/place state machine (`State`, `VacTmr`/`BlowTmr`, `Cmd*`/`At*`, poses `PickL/R`, `DropL/R`, pneumatics `CylinderDown`/`VacuumOn`/`Blowoff`, Z sensors `PickDown/Up`, `DropDown/Up`, `VacuumSensor`); and the tuning values |

> ⚠️ **Two things the CSV can't do for you:**
> 1. **Values.** CSV import creates *definitions* only — every tag comes in at
>    0/false. The full list of values to set by hand (velocities, accels, homing
>    timeout, offsets, timer presets, poses) is in
>    [`plc_tag_values.md`](plc_tag_values.md); each tag's CSV description also
>    repeats its starting value.
> 2. **Physical I/O mapping.** The E-stop/guard/limit/Z-sensor inputs and the
>    solenoid outputs (`CylinderDown`, `VacuumOn`, `Blowoff`) and `EM806_*_ALM`
>    import as **base BOOLs** — after import, alias/map each to its real module
>    point.

**3 — the routines — the complete program** (`right-click a Program → Import
Routine…`, after the tags exist). Import all of them into one Program (e.g.
`Robot`), then **set `R00_Main` as the Program's Main Routine** — it `JSR`s the
rest each scan:

| `docs/l5x/…` | Routine | Role |
|---|---|---|
| `R00_Main.L5X` | scan dispatcher | `JSR`s everything in order; picks Manual vs Auto on the `AutoMode` tag. **Set as Main.** |
| `R10_Safety.L5X` | safety | E-stop / guard / limits / drive alarm → `Status.Faulted` + `FaultCode`; `SafetyOK`; reset |
| `R20_Drives.L5X` | drives | **owns the axis Enable outputs**; publishes `Status.Enabled` |
| `R30_Homing.L5X` | homing coordinator | `JSR`s `R_HomeMotor0/1`; offset-aware angle publish |
| `R40_Manual.L5X` | manual jog/home | absolute jog on `Manual.MoveToTarget`, gated by enabled/homed/limits |
| `R50_Auto.L5X` | **automatic pick/place** | the §11 state machine — camera-clear → pick → vacuum → drop → blowoff |
| `R60_Status.L5X` | status | publishes `Ready`/`Busy`/`VacuumOK` |
| `R_MoveMotor0.L5X` / `R_MoveMotor1.L5X` | move engine, per axis | mirrors `SD_Position_Move`, `Abs_Flag` set |
| `R_HomeMotor0.L5X` / `R_HomeMotor1.L5X` | ClearLink homing, per axis | mirrors `SD_Homing`; `JSR`'d by `R30` |

Manual vs Auto: with `AutoMode = 0` the operator jogs/homes via `R40`; set
`AutoMode = 1` (after enabling + homing) to hand the machine to `R50_Auto`, which
runs the pick/place cycle against the `VisionRobot.Cmd`/`Target` handshake.

> ⚠️ **`R50_Auto`, `R10_Safety` and `R20_Drives` command real motion, the Z
> cylinder and vacuum — REVIEW every rung before running.** The hardware E-stop
> safety relay is primary; these PLC bits only mirror it. These were generated
> without Studio 5000, so they're schema-conformant but not import-verified — if
> Logix flags anything, send me the message and I'll fix the generator.

> **These are plain Routines, not Add-On Instructions.** Teknic ships no motion
> AOI, and a real AOI can't touch the `ClearLink:O1/:I1` module tags directly — so
> the per-axis engines implement the design's `AOI_AxisMove`/`AOI_HomeAxis`
> (`plc_program.md` §5) as one routine per motor. `R20_Drives` is the sole owner of
> the axis Enable output (the move/home engines don't set it) so E-stop can drop it.

---

## 5. Set the Configuration assembly (`ClearLink:C`)

Sent once when the connection is established. Per motor (`Motor0Config`,
`Motor1Config`):

| Setting | Value | Why |
|---|---|---|
| `Home Sensor` connector | the prox input pin (0–12) from §1 | ClearLink reads the prox during a homing move; −1 = hard-stop homing |
| `Config Register` **Homing Enable** (bit 0) | 1 | enables the homing move + `Has Homed`/`Ready To Home` |
| `Config Register` **Home Sensor Active Level** (bit 1) | match the prox | which prox state means "at home" |
| `Config Register` **HLFB Inversion** (bit 3) | **1** | the EM806 has no HLFB — without this, `Enabled`/`At Target Position` never assert and `Motor In Fault` latches |
| `Config Register` **Enable Inversion** (bit 2) | as needed | if the EM806 enables on the opposite electrical sense |
| `Config Register` **Soft Limit Enable** (bit 5) + `Soft Limit 1/2` | after homing | −20° / +200° soft limits (steps = deg × `STEPS_PER_DEG`) |

> Changes to `ClearLink:C` take effect only when the EtherNet/IP connection is
> re-established (cycle the connection or the controller), or send them live with
> an explicit `Set_Attribute` message.

---

## 6. The `VisionRobot` contract — imported, then finished by hand

Three tag groups exist. **You do not create the ClearLink assembly tags** (§3 —
the AOP makes them). The **`VisionRobot` UDT + tag** and the **glue tags** you
**import** from the five `#_VisionRobot*.L5X` UDT files + `RobotTags.csv` (§4)
rather than hand-type.
`VisionRobot` is the surface the vision PC reads/writes over EtherNet/IP with
pycomm3 — the single source of truth: the app's **PLC tab** lists every tag and
the driver reads/writes exactly these.

After importing, the controller tag `VisionRobot` (UDT with
`Cmd`/`Target`/`Manual`/`Status` members — full definition in
[`plc_program.md`](plc_program.md#2-udt)) exists. The essentials:

**Python writes (PC → PLC):**
- `VisionRobot.Manual.Enable / HomeRequest / MoveToTarget / Abort`,
  `Manual.TargetLeftDeg / TargetRightDeg`, `Manual.CommandID` — the manual
  jog/home surface (Robot Test tab).
- `VisionRobot.Cmd.RequestPickPlace / Abort / Reset / CommandID`,
  `VisionRobot.Target.Pick_LeftDeg / …/ Drop_RightDeg / HoleIndex / CoverID` —
  the automatic pick/place job.

**Python reads (PLC → PC):**
- `VisionRobot.Status.Enabled / Homed / InPosition / Moving`,
  `Status.ActualLeftDeg / ActualRightDeg`, `Status.Faulted / FaultCode`,
  `Status.Ready / Busy / Done`, `Status.ActiveCommandID / CompleteCommandID /
  FailedCommandID`, `Status.VacuumOK / CameraClear / ReadyForVision`.

**Glue that comes in with `RobotTags.csv`:** the state-machine tags (`HomeStep`,
per-axis `Home*_State/Req`, `Ax*_HomeDone/Fault`, `Move*_*`, `HR_ons`,
`SoftLimitsEnable`) and the tuning-value tags (`STEPS_PER_DEG` const,
`HOME_VEL_0/1`, `HOME_ACC`, `MOVE_VEL`, `MOVE_ACC`, `HOME_OFFSET_L/R`).

**What you still finish by hand after importing** (CSV imports names/types, not
values — so all of these start at 0/false):
- **Set the tuning values:** `STEPS_PER_DEG := 26.66667` and the starting
  velocities/accels from each tag's CSV description (§4 lists them).
- **Alias `EM806_0_ALM` / `EM806_1_ALM`** onto the ClearLink digital inputs the
  two drive-alarm outputs are wired to (they import as plain BOOLs).
- **Set `HOME_OFFSET_L/R`** once you've measured them (below).
- **Tune** `HOME_VEL_0/1` sign+magnitude, `MOVE_VEL/ACC`, `HOME_ACC` at
  commissioning.

**The bridge = your program.** The imported routines (§4) read/write the
`ClearLink:*` tags and translate that to/from `VisionRobot.*`:
- `Manual.Enable` → drive `Motor*_Output_Reg_Enable`; publish `Status.Enabled`.
- `Manual.HomeRequest` → run `R30_Homing`; publish `Status.Homed` +
  `Status.ActualLeft/RightDeg` (**with the home offset**, §`plc_homing.md`).
- `Manual.MoveToTarget` + `Target*Deg` → `R_MoveMotor0/1` per axis (deg × steps);
  publish `Status.InPosition` + `Status.CompleteCommandID`.

> **Home offset:** the ClearLink zeroes position **at the prox trip point**, not
> at 135.85°/44.15°. Set `HOME_OFFSET_L/R` so
> `ActualDeg = (Motor*_CommandedPosn + HOME_OFFSET) / STEPS_PER_DEG` reads the
> true shoulder angle (`plc_program.md` §5, `plc_homing.md`).

---

## 7. Commission (in this order)

1. **Comms:** from the app's **PLC tab → Connect PLC** at `IP/slot`. Confirm the
   `VisionRobot` tags read/write.
2. **Jog (SD_Jog):** enable one axis, jog slowly; confirm direction and that
   `STEPS_PER_DEG` is right (command 90°, measure the shoulder). Fix `Enable
   Inversion` / step wiring if needed.
3. **Homing:** confirm each prox toggles (read its DIP) as the L1 flag passes;
   run **Robot Test → Home (find ref)**; confirm `Has Homed`, tune `HOME_VEL_0/1`
   (sign = approach direction), and set `HOME_OFFSET_L/R` so
   `ActualLeftDeg ≈ 135.85`, `ActualRightDeg ≈ 44.15`.
   Verify the sequential sweep can't collide the two arms.
4. **Absolute moves:** jog via **Robot Test** (Cartesian/joint); confirm
   `CompleteCommandID` tracks each move and `InPosition`/`At_Target_Posn` gates it.
5. **Soft limits:** enable them; confirm a move past −20°/+200° is refused.
6. **Faults:** trip E-stop (drops enable → State 900), an EM806 ALM, and a hard
   limit; confirm each faults and `Cmd.Reset` recovers.
7. **Only then** bring up the automatic pick/place sequence (`plc_program.md` §7).

Everything above can be dry-run first against the app's simulated PLC
(`--sim-plc`) so the handshake logic is proven before hardware.
