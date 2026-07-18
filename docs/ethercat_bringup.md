# EtherCAT bring-up — StepperOnline A6-EC + PC master (Stage 4)

The PC is the motion controller. It runs a `pysoem` EtherCAT master that streams
Cyclic-Synchronous-Position (CSP) setpoints to two A6-EC servo drives over a
single EtherCAT segment. This document is the hardware checklist for
`PysoemMaster` (`src/bung_cover_robot/ethercat/master.py`), which is written but
**bench-untested** — it needs the real drives and a real-time kernel to validate.

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
zero to the shoulder angle. Because the A6 encoder is **absolute**, the datum
survives a disable — no re-home after every enable (unlike the old steppers).

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

## 5c. Distributed clocks vs free-run (why Er741 happens)

`PysoemMaster` runs **free-run (SM-synchronous)** by default (`use_dc=False`):
the drive acts on each PDO as it arrives. A `time.sleep`-paced Python RT loop
**cannot** phase-lock to the DC **SYNC0** hardware pulse, so if DC is enabled the
drive faults on a sync error the instant it enters OP — on the AS715N this shows
as **`Er741`**. Free-run avoids that and is correct for bench work and CSP jogging
(slightly more jitter, which a Python master has regardless).

Proper **DC-synced CSP** (`use_dc=True`) is a *future production* step: it needs
the RT loop to program SYNC0 (`slave.dc_sync(...)`) and phase-align its sends to
the DC. Don't enable it until that loop exists — free-run is the right default now.

> If you *do* see `Er741` (or any sync fault): power-cycle the drive to clear it,
> confirm the master is free-run, and reconnect. Look the exact code up in the A6
> servo software's **Fault Dictionary** (or the A6-EC manual) to confirm.

## 6. First-motion checklist (do this once, slowly)

1. `--sim-ec` full cycle passes in software. ✅ (already tested)
2. Master reaches OP: all slaves to OP_STATE, `receive_processdata` WKC == #slaves.
3. Arm one drive to Operation Enabled; confirm the statusword decodes as expected.
4. **Jog test at low speed** (`TrajectoryLimits(speed_mm_s=20)`): a short move,
   watch following error stay small.
5. Home to the switches; verify `read_angles()` matches the taught home pose.
6. Run a single pick/place with vision bypass and a low speed cap before full rate.
7. Only then raise `TrajectoryLimits` speed/accel toward the inertia-matched limits.
