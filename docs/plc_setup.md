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
5. **Import Teknic's example `.L5K`** and lift the routines you need (§4).
6. **Set the Configuration assembly** (home sensor, homing enable, HLFB/Enable
   inversion, soft limits) (§5).
7. **Create the `VisionRobot` UDT + tags** and the glue that maps the ClearLink
   motion to them (§6).
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
| **`SD_Homing`** | the homing routine (`R30_Homing` / `AOI_HomeAxis`) | one motor; duplicate for M-1 and sequence both (§`plc_homing.md`) |
| **`SD_Position_Move`** | moving a shoulder to an **absolute angle** (`AOI_AxisMove`) | the example moves *incrementally* — set `Abs_Flag` for absolute (§`plc_program.md` §3) |
| **`SD_Jog`** | manual velocity jogging during bring-up | handy to confirm direction/wiring before homing |
| **`SD_Velocity_Move`** | reference for velocity moves | not needed for the pick/place cycle |

**Adapting to two axes:** each example drives **Motor 0**. For the robot,
duplicate the logic for **Motor 1** (swap `Motor0_*` → `Motor1_*`), assign
Motor 0 = left / Motor 1 = right, and drive both from the coordinator/dispatcher
in `plc_program.md`. The corrected per-axis logic and ladder visuals (matched to
these examples) are in `plc_homing.md` and `plc_ladder.md`.

### Importable routines in this repo (`docs/l5x/`)

We also ship the corrected ladder as **importable Studio 5000 routine `.L5X`** —
the same neutral rung text as Teknic's examples, so you can import instead of
re-typing. **Right-click a Program → Import Routine…** and pick the file:

| `docs/l5x/…` | Routine | Notes |
|---|---|---|
| `AOI_AxisMove.L5X` | absolute move, Motor 0 | copy for Motor 1 (`Motor0_`→`Motor1_`) |
| `AOI_HomeAxis.L5X` | ClearLink homing move, Motor 0 | copy for Motor 1 |
| `R30_Homing.L5X` | 2-axis homing coordinator | `JSR`s the two homing routines; offset-aware publish |

Each routine's **rung-0 comment lists the tags/constants to create** (locals like
`Home0_State`, aliases like `EM806_0_ALM`, constants like `STEPS_PER_DEG`,
`HOME_VEL`, `HOME_OFFSET_L/R`). The `ClearLink:O1/:I1/:C` tags come from the
module (§3); the `VisionRobot.*` tags from §6. Undefined tags flag on import —
that's expected; resolve them against what you created.

> ⚠️ **Import-test these first.** They were generated without Studio 5000, so
> they're schema-conformant but not import-verified. Import one, and if Logix
> reports a schema/format error, send it to me and I'll fix the generator
> (`scripts/render_plc_l5x.py`). For anything safety- or motion-critical, the
> Teknic `.L5K` examples remain the ground truth to cross-check against.

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

## 6. Tags you must create — the `VisionRobot` contract

Two tag groups exist. **You do not create the ClearLink assembly tags** (§3 — the
AOP makes them). You **do** create the `VisionRobot` UDT — the surface the vision
PC reads/writes over EtherNet/IP with pycomm3. It is the single source of truth:
the app's **PLC tab** lists every tag and the driver reads/writes exactly these.

Build one controller tag `VisionRobot` (UDT with `Cmd`/`Target`/`Manual`/`Status`
members — full definition in [`plc_program.md`](plc_program.md#2-udt)). The
essentials:

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

**Glue you also create:** state-machine locals (`HomeStep`, per-axis
`Ax*_HomeReq/Done/Fault`, `CURRENT_STATE`, …), the constants
(`STEPS_PER_DEG := 26.66667`, `HOME_VEL`, `HOME_ACC`, `MOVE_VEL`, `MOVE_ACC`, and
`HOME_OFFSET_L/R`), and the `AxisIF` aliases onto `ClearLink:O1/:I1` (member list
in `plc_homing.md` §1).

**The bridge = your program.** The imported example logic (§4) reads/writes the
`ClearLink:*` tags; your routines translate that to/from `VisionRobot.*`:
- `Manual.Enable` → drive `Motor*_Output_Reg_Enable`; publish `Status.Enabled`.
- `Manual.HomeRequest` → run `R30_Homing`; publish `Status.Homed` +
  `Status.ActualLeft/RightDeg` (**with the home offset**, §`plc_homing.md`).
- `Manual.MoveToTarget` + `Target*Deg` → `AOI_AxisMove` per axis (deg × steps);
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
   run **Robot Test → Home (find ref)**; confirm `Has Homed`, tune `HOME_VEL`,
   and set `HOME_OFFSET_L/R` so `ActualLeftDeg ≈ 135.85`, `ActualRightDeg ≈ 44.15`.
   Verify the sequential sweep can't collide the two arms.
4. **Absolute moves:** jog via **Robot Test** (Cartesian/joint); confirm
   `CompleteCommandID` tracks each move and `InPosition`/`At_Target_Posn` gates it.
5. **Soft limits:** enable them; confirm a move past −20°/+200° is refused.
6. **Faults:** trip E-stop (drops enable → State 900), an EM806 ALM, and a hard
   limit; confirm each faults and `Cmd.Reset` recovers.
7. **Only then** bring up the automatic pick/place sequence (`plc_program.md` §7).

Everything above can be dry-run first against the app's simulated PLC
(`--sim-plc`) so the handshake logic is proven before hardware.
