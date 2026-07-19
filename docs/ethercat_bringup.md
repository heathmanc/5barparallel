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
| Load inertia ratio | `0x2000:05` | 100 | 0–12000 % | set first |
| Auto-tuning mode | `0x2000:03` | 1 | 0=Manual/1=Standard/2=Positioning | 0 to hand-tune gains |
| Stiffness level | `0x2000:04` | 12 | 1–31 | main dial in Standard mode |
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
  output (0x60FE:1)** reserved for the vacuum later.
- The touch-probe words are carried but unused (packed/ignored as 0).
- If a future drive ships a *different* default PDO, re-verify with
  `scripts/ec_inspect.py` and adjust `_RX_FMT`/`_TX_FMT`.
  `tests/test_ethercat_master.py` pins the pack/unpack layout.

## 4. Homing

CiA 402 homing mode (mode 6): `home()` puts both drives in Homing mode and sets
the start bit; the drive runs its own homing method to the switch and zeros its
position there. The recorded `home_angles` (from `robot_config.yaml`) map that
zero to the shoulder angle.

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
- Vacuum + blow-off → a drive DO or a small EtherCAT I/O slice; wire the vacuum
  confirwith switch back to a DI so the cycle can verify a cover is held.

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
