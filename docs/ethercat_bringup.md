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
  port, drive 1 OUT → drive 2 IN. The interface name (reference machine:
  `enp0s31f6`, `e1000e`) is the `ifname` for `PysoemMaster` and the Drives tab
  `ethercat_ifname` field. Keep IP traffic on a separate interface (WiFi).
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

`PysoemMaster` packs/unpacks these fixed images (little-endian, packed):

```
RxPDO 0x1600 (PC → drive):  Controlword 0x6040 (U16) | Modes 0x6060 (S8) | TargetPosition 0x607A (S32)
TxPDO 0x1A00 (drive → PC):  Statusword  0x6041 (U16) | ModesDisp 0x6061 (S8) | PositionActual 0x6064 (S32)
```

Confirm the drive's actual assignment matches this order and width. If your A6
firmware ships a different default PDO, either re-map it in `_configure_slave()`
(SDO writes to 0x1C12/0x1C13 + the 0x1600/0x1A00 entries) or adjust `_RX_FMT` /
`_TX_FMT`. `tests/test_ethercat_master.py` checks the pack/unpack is self-consistent.

## 4. Homing

CiA 402 homing mode (mode 6): `home()` puts both drives in Homing mode and sets
the start bit; the drive runs its own homing method to the switch and zeros its
position there. The recorded `home_angles` (from `robot_config.yaml`) map that
zero to the shoulder angle. Because the A6 encoder is **absolute**, the datum
survives a disable — no re-home after every enable (unlike the old steppers).

## 5. Safety (hardware, not software)

- **E-stop → safety relay → both drives' STO.** STO removes torque independently
  of the master, so an E-stop stops motion even if the PC hangs. This is the
  primary stop; the software `stop()` (hold position) is secondary.
- Home/limit switches → the drives' digital inputs (used by the homing method and
  as hard travel limits).
- Vacuum + blow-off → a drive DO or a small EtherCAT I/O slice; wire the vacuum
  confirwith switch back to a DI so the cycle can verify a cover is held.

## 6. First-motion checklist (do this once, slowly)

1. `--sim-ec` full cycle passes in software. ✅ (already tested)
2. Master reaches OP: all slaves to OP_STATE, `receive_processdata` WKC == #slaves.
3. Arm one drive to Operation Enabled; confirm the statusword decodes as expected.
4. **Jog test at low speed** (`TrajectoryLimits(speed_mm_s=20)`): a short move,
   watch following error stay small.
5. Home to the switches; verify `read_angles()` matches the taught home pose.
6. Run a single pick/place with vision bypass and a low speed cap before full rate.
7. Only then raise `TrajectoryLimits` speed/accel toward the inertia-matched limits.
