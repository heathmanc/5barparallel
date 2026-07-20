# EtherCAT bring-up — StepperOnline A6-EC + PC master

> **The real-hardware master is IgH EtherLab, not pysoem.** The AS715N is
> **DC-SYNC0-only**, and pysoem cannot generate a SYNC0 the drive detects (it
> faults `Er74.1` "No sync signal"). The IgH master does — **verified on the
> bench**: the drive holds OP with DC, `err=0x0000`, encoder live. Production runs
> on a small C RT daemon (`igh/ec_master_daemon`, using IgH's `libethercat`) that
> owns the DC loop and exposes the process data over shared memory, with
> `IgHMaster` (`src/bung_cover_robot/ethercat/igh_master.py`) as the Python side
> behind the same `EtherCatMaster` interface. **See [`../igh/README.md`](../igh/README.md)**
> for install/build/run. `PysoemMaster` is kept for reference / non-DC drives but
> is not used for this hardware.

The PC is the motion controller. It streams Cyclic-Synchronous-Position (CSP)
setpoints to the A6-EC servo drives over a single EtherCAT segment. The sections
below (PDO map, homing, safety, first-motion) apply regardless of master; the
pysoem-specific mechanics are historical.

> Everything above this layer (CiA 402 sequencing, the Cartesian CSP trajectory
> planner, the `EtherCatRobotDriver`, the whole pick/place cycle) is already
> tested in software against `SimulatedEtherCatMaster` — run it with
> `python -m bung_cover_robot.gui --sim-ec`.

## 1. Control PC / OS

Full platform build (RT kernel, NIC, core isolation, cyclictest acceptance) is in
[`control_pc_setup.md`](control_pc_setup.md). In short:

- Linux with a **PREEMPT_RT** kernel (`uname -v` shows `PREEMPT_RT`). Without it,
  CSP following error will grow under load.
- A **dedicated Intel NIC** for EtherCAT, wired point-to-point to drive 1's IN
  port, drive 1 OUT → drive 2 IN. The interface name (reference machine: the
  I219-V `e1000e` port, MAC-pinned to `ecat0` via a systemd `.link` file — see
  [`control_pc_setup.md`](control_pc_setup.md) §2) is the `ifname` for
  `PysoemMaster` and the Drives tab `ethercat_ifname` field. Keep IP traffic on a
  separate interface (WiFi).
- Isolate CPU cores for the RT thread (reference machine, 8-core no-HT:
  `isolcpus=6,7 nohz_full=6,7 rcu_nocbs=6,7`), then pin the process; grant
  `CAP_SYS_NICE` (or run the master as root) so `SCHED_FIFO` + `mlockall` take
  (`set_realtime()` logs a warning if they don't).
- `pip install pysoem` on the control PC (in the project venv).

## 2. Drive parameters (set once per drive, in the A6 tool or over SDO)

| Setting | Value | Why |
|---|---|---|
| EtherCAT station alias | 1 (left), 2 (right) | stable slave identity |
| Control mode | CSP (Cyclic Sync Position, mode 8) | streamed position |
| Electronic gear / encoder | 17-bit = 131072 counts/rev | feeds deg↔counts |
| Homing method (0x6098) | switch + index per your fixtures | see §4 |
| Following-error window | tight enough to trip on a stall | safety |
| STO | wired to the safety relay (see §5) | E-stop |

**Update `config/robot_config.yaml`** so the software deg↔counts matches the
drive: set the drivetrain to the servo (`pulses_per_rev: 131072`, belt ratio as
built). The planner and driver share `pulses_per_degree`, so this must be right
or moves land at the wrong angle.

### Object dictionary is the source of truth — not the panel Cxx.NN

`esi/STEPPERONLINE_A6_Servo_V0.02.xml` is the drive's EtherCAT Slave Info: the
real object dictionary (indices, subindices, ranges, sizes). The panel's
`Cxx.NN` numbering does **not** track the SDO subindex in group `0x2000` — the
subindex is the entry's *position* in the ESI object, not `NN+1`. Always read
the address from the ESI. The Drives-tab tuning table is preloaded from it:

| Tuning object | CoE | Default | Range | Notes |
|---|---|---|---|---|
| Load inertia ratio | `0x2000:07` | 100 | 0–12000 % | set first |
| Auto-tuning mode | `0x2000:05` | 1 | 0=Manual/1=Standard/2=Positioning | 0 to hand-tune gains |
| Stiffness level | `0x2000:06` | 12 | 1–31 | main dial in Standard mode |
| 1st position loop gain | `0x2001:01` | 400 | 0–20000 (0.1 rad/s) | |
| 1st speed loop gain | `0x2001:02` | 250 | 1–20000 (0.1 Hz) | raise before position gain |
| 1st speed loop integral time | `0x2001:03` | 3184 | 1–51200 (0.01 ms) | lower = stronger integral |
| 1st torque ref filter | `0x2001:04` | 200 | 5–16000 Hz (cutoff) | lower = more damping |

All U16, modifiable during operation, effective immediately. In Standard mode
(auto-tune = 1) the loop gains are derived from the stiffness level, so stiffness
is the primary knob; set auto-tune = 0 to write the loop gains directly.

## 3. PDO map (0x1C12 / 0x1C13)

**Verified on the bench** (`scripts/ec_inspect.py`): the ANCTL AS715N (the
StepperOnline A6-EC) ships a fixed native map that `master.py` now matches
exactly — little-endian, packed:

```
RxPDO 0x1701 (PC → drive, 12 B):
  Controlword 0x6040 (U16) | TargetPosition 0x607A (S32)
  | TouchProbe 0x60B8 (U16) | DigitalOutputs 0x60FE:1 (U32)
TxPDO 0x1B01 (drive → PC, 28 B):
  ErrorCode 0x603F (U16) | Statusword 0x6041 (U16) | PositionActual 0x6064 (S32)
  | TorqueActual 0x6077 (S16) | FollowingError 0x60F4 (S32)
  | TouchProbeStatus 0x60B9 (U16) | TouchProbe1 0x60BA (S32)
  | TouchProbe2 0x60BC (S32) | DigitalInputs 0x60FD (U32)
```

Notes:

- **Mode of operation (0x6060) is not cyclic** in this map, so `_configure_slave()`
  writes CSP (8) once over SDO. The drive already powers up in CSP.
- The map hands us **following error** and **digital inputs (0x60FD)** for free —
  the Drives page shows real switch states and following error — plus a **digital
  output word (0x60FE:1)** that drives the **pick head**: `EtherCatRobotDriver`
  asserts bit 0 for the vacuum solenoid and bit 1 for the air-cylinder plunger on
  drive 0 (the `vacuum_do_bit` / `plunger_do_bit` / `tooling_drive` constructor
  args repoint this to a dedicated EtherCAT I/O slice later). The daemon rewrites
  the whole DO word every DC cycle, so a set bit stays asserted until cleared.
- The touch-probe words are carried but unused (packed/ignored as 0).
- If a future drive ships a *different* default PDO, re-verify with
  `scripts/ec_inspect.py` and adjust `_RX_FMT`/`_TX_FMT`.
  `tests/test_ethercat_master.py` pins the pack/unpack layout.

## 4. Homing

This machine homes to **hard mechanical stops — no home switches.** Procedure:
drive each axis to its hard stop (jog, or push by hand while disabled), then
press **Set Home** on the Drives tab. `set_home()` captures the current encoder
counts as the datum and declares that pose to be the configured `home_angles`
(from `robot_config.yaml`). All subsequent Cartesian moves are referenced to it.

> The configured `home_angles` **must match the physical hard-stop pose**, or
> every Cartesian move is offset by the difference. Set `home_angles` to the
> actual shoulder angles at the hard stops.

(`home()` still implements CiA 402 homing mode 6 for a switch-equipped drive —
puts both drives in Homing mode, runs the drive's homing method to the switch —
but it is not used on this machine.)

**Encoder type matters (drive param C00.07).** Set it to match the motor:

- **Single-turn absolute** (current bench motors): the datum survives a *disable*
  (drive stays powered), so no re-home on a mid-session enable. But a full
  **power-off** loses the multi-turn count — with the 3:1 reduction the drive
  then only knows the shoulder angle modulo 120°, so **re-home after every power
  cycle**.
- **Multi-turn absolute** (battery-backed): the revolution count survives
  power-off too, so the datum is truly persistent — home once, ever.

Either way the software requires a home on each program launch (`is_referenced`
starts `False`), so start-up is safe; the difference is only whether a routine
power cycle forces an operator re-home. If C00.07 is left on **incremental**, the
drive ignores the absolute datum entirely and homes from wherever it powered up.

### 4d. Drive drops to SWITCH ON DISABLED with no fault

If a drive falls out of Operation Enabled *without* a fault, the power-stage
enable chain blipped — the drive was told (or forced) to drop torque, it did
not trip. Usual culprits, most likely first:

1. **STO / E-stop chain chatter** — a marginal contact or connector on the
   safety relay/contactor path opens for a few ms (vibration at higher
   speeds makes this worse). The refusal message now flags this when the
   statusword's voltage bit is down.
2. **24 V logic dip** — shared supply sagging under load.
3. **Drive sync-loss reaction** configured as "servo off" (warning class)
   rather than a latching fault.

The "cannot move: drives are disabled" error now names the drive, its decoded
CiA 402 state, and the raw statusword/error code. The bench demo auto
re-enables ONCE per job after an unexpected disable (never after a fault) and
tags the run result with how many times it had to — repeated auto re-enables
mean fix the chain, not the software.

**Prove or rule out the cable/EMI (the #1 cause) — built into the Drives
tab.** The status table's **"Link errors (CRC)"** row shows each drive's ESC
error counters live (the daemon reads registers 0x0300..0x0313 ~1 Hz and
publishes them over shared memory — no CLI, no sudo, no second process).
Press **Zero CRC ctrs**, run a speed trial, and watch the row: it stays
"0 (clean)" on a good link and goes red with a breakdown (`75  rx37 inv37
lost1`) on a bad one. Requires daemon ABI 4 — after pulling, rebuild once:
`make -C igh ETHERLAB=/opt/etherlab` (the app refuses an old daemon with a
clear message). `scripts/ec_crc.py` remains as a standalone fallback. RX-error / invalid-frame climbing on a port = the cable
segment feeding that port; lost-link = an intermittent connector; forwarded
errors = the fault is upstream (nearer the PC). A real drive fault never moves
these — anything that climbs under load is physical-layer, so fix the cabling
(shielded S/FTP, away from the motor power leads) before anything else.

### 4c. "Move did not settle" — tolerance and settle window

The end-of-move check is governed by two **Motion parameters** (Drives tab,
persisted, applied on Connect and on Apply):

- `position_tol_counts` — default **500** (~0.46° at the joint with the 17-bit
  encoder × 3:1; ≈1.5–2 mm at the TCP). The old default of 5 counts (0.005°)
  was only achievable in the simulator.
- `settle_timeout_s` — default **2.0 s**. The servo's integrator needs real
  time to pull in the last fraction of a degree after the CSP stream ends; a
  longer window often lets you run a *tighter* tolerance.

If moves still time out, the error now reports the per-drive shortfall in
counts and degrees. A persistent shortfall of hundreds of counts is a tuning
problem (raise stiffness, or auto-tune=0 and raise position loop gain / lower
the speed-loop integral time) or belt wind-up — the tolerance only decides
when to complain about it.

### 4b. Spurious "drives are faulted" aborts (fixed by debounce)

A demo/move can only be refused for a fault if the CiA 402 fault bit survives
**three consecutive fresh PDO cycles** (`_confirmed_fault()`), and a CSP stream
only aborts on **two consecutive** faulted samples. A real A6 fault LATCHES
until reset, so the debounce can never hide one — but a single torn
shared-memory read racing the RT daemon (or a one-cycle bus hiccup) no longer
kills a run with a phantom "cannot move: drives are faulted". When a fault IS
real, the error now carries the actual per-drive statusword and error code
(e.g. `drive 0: sw=0x0218 err=0x7500`) instead of just a drive number; ignored
transients are logged as warnings so they stay visible.

### 4a. Sample pick & place (vision bypass)

The Drives tab has a **Simulate pick && place** button (in the jog box) to
exercise the arm and pick head before vision calibration exists. It needs the
drives **enabled** and **Set Home** done, then runs a canned cycle: pick from a
**fixed supply nest** (same spot every time), drop into a **six-hole battery**
laid out just like the real thing — six covers in a straight line at **35 mm
pitch** — that is placed at a random position/tilt in the reachable envelope each
pass (so a range of poses gets exercised while the battery geometry stays
realistic). Every point is workspace-validated, and the vacuum/cylinder actuate
at each end just like a real pick. Tick **loop** to keep it running (re-placing
the battery each pass) until you press **Stop demo**. The **cycles/min** readout
shows a rolling average of completed placements per minute (trailing 60 s).

This is `run_demo_cycle` + `demo_pick_and_place_targets` (both in
`app/cycle_manager.py`), the same `DirectJobRunner` the vision cycle uses — it
just supplies fixed targets instead of camera detections.

**Speed.** The demo travels at the **jog speed** set in the jog box (a gentle
default), *not* the full `speed_mm_s` motion limit — a big point-to-point move at
the tuned maximum can outrun an under-tuned servo and trip the drive's
**excessive-position-deviation** alarm (StepperOnline A6 **Er.47** / CiA 402
error `0x8611`, the ESI's "Excessive position deviation threshold"). If you still
see Er.47, lower the jog speed further, drop `accel_mm_s2`, or raise the drive's
deviation threshold / retune the loop gains — the arm is being commanded faster
than it can follow.

### Tuning assistant (Drives tab → "Tuning…")

The **root cause** of speed-proportional Er.47 is that we stream **position
only** (CSP) and, out of the box, the A6's **speed feedforward is off**
(`C01.13 Speed feedforward selection = 0`, ESI object `0x2001:20`). Without it a
position loop lags in direct proportion to velocity, so following error climbs
with speed until it trips. The assistant fixes and measures this without the
guesswork:

1. **Load inertia (C00.06)** — estimated from the arm *geometry* (`robot/inertia.py`),
   **not** the drive's native inertia auto-tune (`F30.10`). That routine free-spins
   one axis several turns; on the assembled 5-bar the shoulders are coupled through
   the linkage, so it would fight the mechanism and exceed the joint window. Enter
   your motor's rotor inertia (Jm) for a trustworthy ratio.
2. **Speed feedforward** — sets `C01.13 = 1` (internal reference: FF derived from
   the position-command slope) and `C01.14` to the chosen gain (`0x2001:21`,
   0.1 % units; start 50 %, climb toward 100 %). Written to both drives and saved.
3. **Characterize** — runs a *validated* out-and-back TCP move (never a free
   spin) and reports the **peak following error** in counts, degrees, and as a
   percentage of the `0x6065` fault window. Raise FF/stiffness and re-run to watch
   the peak fall; only growth past the window trips Er.47.

Peak following error is captured inside the CSP streaming loop (read-only on the
real master, exact in the simulator), so the number reflects the whole move, not
a slow GUI sample.

**Start-of-move chirp / accel-phase spike.** A hard trapezoidal profile steps
acceleration on instantly (infinite jerk), which impulsively excites a mechanical
resonance — an audible chirp plus a following-error spike at the *start* of a
move. The planner is **jerk-limited (S-curve)**: `jerk_mm_s3` (motion parameter,
default 80000 mm/s³) eases acceleration in over ~`accel/jerk` seconds, removing
the impulse with almost no loss of cruise speed. Set it to 0 for the old hard
trapezoid; lower it for a gentler/quieter start at the cost of a little move
time. Torque feedforward (`C01.16`/`C01.17`) complements it for the accel phase.

**Velocity feedforward streaming (0x60B1) — the chirp-free FF.** With speed-FF
source = internal reference (`C01.13 = 1`), the drive derives its feedforward by
*differentiating the position command we stream*. In CSP that command is a
per-cycle staircase, so the derivative is spiky — it turns into an audible chirp
and a following-error ripple (worst as speed rises; jerk limiting doesn't help
because it's the command granularity, not the ramp). The fix is to feed the
drive OUR trajectory's velocity directly: the daemon maps `0x60B1` (velocity
offset) into the flexible RxPDO `0x1600` and streams the per-cycle velocity
(counts/s) alongside the position, and the drive is set to speed-FF source =
Communication (`C01.13 = 5`) so it uses it. Clean feedforward, no differentiation
→ tight tracking *and* no chirp, independent of load inertia.

To enable it:
1. Rebuild the daemon (ABI bumped to 5): `make -C igh ETHERLAB=/opt/etherlab`,
   then `sudo pkill ec_master_daemon` and reconnect.
2. Set `C01.13 = 5` (`speed_ff_source`) on both drives.
3. `velocity_ff_scale` (motion parameter, default 1.0) trims the streamed value
   for the drive's velocity-offset units — if FF over/under-compensates, adjust
   it until the characterize FE minimises. 0 disables streaming.

## 5. Safety (hardware, not software)

- **E-stop → hardwired torque removal, independent of the PC.** Two cases:
  - *Drive has STO:* E-stop → safety relay → both drives' STO. STO removes torque
    directly. Preferred.
  - *Drive has no STO* (the bench AS715N): E-stop → safety relay → **contactor
    that drops the drives' motor/bus power**. Slower than STO (the axis brakes /
    coasts as the bus collapses) but still a real stop that works even if the PC
    or drive firmware hangs. **Do not enable or jog a drive until this exists and
    is tested**, and keep the motor mechanically secured on the bench.
  Either way, this hardwired stop is primary; the software `stop()` (hold
  position) is secondary.
- Home/limit switches → the drives' digital inputs (used by the homing method and
  as hard travel limits).
- Vacuum + air cylinder → drive 0's DO word (0x60FE:1) today (vacuum = bit 0,
  plunger = bit 1); repointable to a small EtherCAT I/O slice later. The
  pick/place cycle (`DirectJobRunner`) sequences them: travel → plunge → vacuum
  ON → dwell → lift → travel → plunge → vacuum OFF → dwell → lift, with the dwells
  set by `PickSequence`. Wire the vacuum confirm switch back to a DI so the cycle
  can verify a cover is held.

## 5b. Single-axis bench bring-up

For bench work on **one drive** before the robot is assembled, set **Drives on
bus = 1** in the Drives-tab Connection box (persisted as `ethercat_num_drives`).
The master then expects a single slave, the Drives page shows that axis live
(state, encoder counts, following error, I/O, parameter table) and marks the
absent second panel "not on bus". **Connecting produces no torque** — the drive
stays in `switch_on_disabled` until you deliberately enable it, so viewing is
safe with no power-stage interlock in place. Enabling/jogging is *not* safe until
the stop interlock below exists.

## 5c. The AS715N is DC-SYNC0-only (the Er741 story)

Read the drive's own sync config with `scripts/ec_inspect.py`: the AS715N's sync
managers (`0x1C32`/`0x1C33:04`) advertise **only DC-SYNC0** — no free-run, no
SM-sync. **DC is therefore mandatory.** If the drive enters OP without a programmed
SYNC0 pulse, it faults with a **synchronization-signal error — `Er741`**. (Trying
to run free-run, or picking Profile Position to dodge sync, does *not* help: the
sync-manager requirement is below the CiA 402 mode.)

So `open()` (with `use_dc=True`, the default) **enables DC and programs SYNC0** at
the cycle time:

```
m.config_dc()
for s in slaves:
    s.dc_sync(True, cycle_dt_s * 1e9)   # SYNC0 period in ns
```

The drive's ESC then generates SYNC0 from its own DC-synchronized clock, so the RT
loop only has to keep frames flowing each cycle (which it does). On the PREEMPT_RT
control PC (isolated core, ~32 µs jitter) a 2 ms cycle holds this comfortably.

Mode of operation is independent of this: bench single-axis still uses Profile
Position (mode 1) for simple jogging, the 2-drive robot uses CSP (8) — **both over
DC-SYNC0**.

**Fault codes (from the A6-EC troubleshooting chapter):**

| Fault | Name | 0x603F |
|---|---|---|
| **Er74.0** | EtherCAT synchronization cycle setting error | 0x6320 |
| **Er74.1** | **No sync signal** | **0x8700** |
| Er74.2 | Chip synchronization process uncompleted in OP | 0x8700 |
| ErC1.0 | Excessive EtherCAT synchronization period error | 0x8700 |

**`Er74.1` (0x8700) = "No sync signal"** — the drive sees **zero SYNC0 pulses**.
This is a signal-*generation* fault, not a timing one: if `slave.dc_sync(...)`
were producing SYNC0 but mistimed, you'd get `Er74.0`/`ErC1.0` instead. Getting
`Er74.1` means the master isn't generating a SYNC0 the drive detects — check that
`pysoem`'s DC activation (`config_dc()` + `slave.dc_sync()`) actually fires SYNC0
(pysoem version, DC reference-clock selection), not the SYNC0 cycle/shift values.

## 6. First-motion checklist (do this once, slowly)

1. `--sim-ec` full cycle passes in software. ✅ (already tested)
2. Master reaches OP: all slaves to OP_STATE, `receive_processdata` WKC == #slaves.
3. Arm one drive to Operation Enabled; confirm the statusword decodes as expected.
4. **Jog test at low speed** (`TrajectoryLimits(speed_mm_s=20)`): a short move,
   watch following error stay small.
5. Home to the switches; verify `read_angles()` matches the taught home pose.
6. Run a single pick/place with vision bypass and a low speed cap before full rate.
7. Only then raise `TrajectoryLimits` speed/accel toward the inertia-matched limits.
