# PLC values to set after import

Studio 5000's tag CSV/L5X import creates tag **definitions only** — every
tag comes in at `0`/`0.0`. After importing `docs/l5x/RobotTags.csv`, set
these values by hand (Controller Tags → Monitor/Edit). Most are starting
points you refine at commissioning; `STEPS_PER_DEG` is fixed and
`HOME_OFFSET_L/R` you measure. Source of truth: `scripts/render_plc_l5x.py`.

## Motion — absolute moves (R_MoveMotor0/1)

| Tag | Type | Value to set | Unit | Notes |
|---|---|---|---|---|
| `STEPS_PER_DEG` | REAL | `26.66667` (fixed) | steps/deg | 3200 pulses/rev * 3:1 / 360. Set value to 26.66667 after import. |
| `MOVE_VEL` | DINT | `20000` | steps/s | Move speed, steps/s (max 500000). Set ~20000. |
| `MOVE_ACC` | DINT | `100000` | steps/s^2 | Move accel, steps/s^2. Set ~100000. |
| `MOVE_DEC` | DINT | `0` | steps/s^2 | Move decel, steps/s^2. 0 => use accel. |
| `MOVE_SETTLE_MS` | DINT | `100` | ms | Post-load move settle, ms. A move can't read 'in position' until this elapses, so the ClearLink At_Target_Posn has cleared from the previous move before it's checked - stops a chained auto move from completing on a stale at-target and being cancelled before it arrives. |

## Homing (R_HomeMotor0/1)

| Tag | Type | Value to set | Unit | Notes |
|---|---|---|---|---|
| `HOME_VEL_0` | DINT | `-2000` | steps/s | Motor 0 homing speed, steps/s, signed toward the prox. Tune. |
| `HOME_VEL_1` | DINT | `2000` | steps/s | Motor 1 homing speed, steps/s, signed toward the prox. Tune. |
| `HOME_ACC` | DINT | `50000` | steps/s^2 | Homing accel, steps/s^2. Set ~50000. |
| `HOME_TMO_MS` | DINT | `15000` | ms | Homing timeout, ms (loaded into Home*_Tmr.PRE). Homing beyond this faults. |

## Home offsets — measure at commissioning (R30_Homing)

| Tag | Type | Value to set | Unit | Notes |
|---|---|---|---|---|
| `HOME_OFFSET_L` | DINT | `3748` | steps | Left switch angle * STEPS_PER_DEG (nominal 3748 -> ActualLeftDeg ~140.54). Refine at commissioning. |
| `HOME_OFFSET_R` | DINT | `1052` | steps | Right switch angle * STEPS_PER_DEG (nominal 1052 -> ActualRightDeg ~39.46). Refine at commissioning. |

## Auto pick/place process timers (R50_Auto)

| Tag | Type | Value to set | Unit | Notes |
|---|---|---|---|---|
| `VAC_SETTLE` | DINT | `300` | ms | Vacuum settle time, ms (VacTmr preset). Tune. |
| `BLOWOFF_TIME` | DINT | `200` | ms | Blowoff time, ms (BlowTmr preset). Tune. |
| `BENCH_DWELL_MS` | DINT | `600` | ms | BENCH: with Bypass_Vision, dwell this long at each cylinder down/up step so the pick/place is visible on the bench (no real Z/vacuum). 0 = no dwell. Ignored in production (real sensors gate). |

## Poses — set to safe positions (R50_Auto)

| Tag | Type | Value to set | Unit | Notes |
|---|---|---|---|---|
| `CAMERA_CLEAR_L` | REAL | `140.5406` | deg | Camera-clear pose, left shoulder deg. Defaults to home (a safe, small move); set to a real out-of-view pose. NEVER leave 0: a 0 target forces a full-speed swing to 0 deg that open-loop steppers skip on. |
| `CAMERA_CLEAR_R` | REAL | `39.4594` | deg | Camera-clear pose, right shoulder deg. Defaults to home; see CAMERA_CLEAR_L. |

## Heartbeat watchdog + drop-out debounce (R10/R20)

| Tag | Type | Value to set | Unit | Notes |
|---|---|---|---|---|
| `HB_TIMEOUT_MS` | DINT | `1000` | ms | PC heartbeat must change within this many ms or the PLC declares comms loss (code 10) and drops the drives. Set > 4x the PC heartbeat period. |
| `EN_DROP_TMO_MS` | DINT | `1000` | ms | If the drive is commanded on but Status.Enabled stays false this long (ms), treat it as a drop-out (power-cycle) and drop Manual.Enable so it can't auto-re-enable. > drive enable time, < a real power-off. |

## Timer presets (`.PRE`)

Timers import with `.PRE = 0`, which would make `.DN` true immediately.
The homing timer is loaded by ladder each scan; the auto timers are loaded
by `R50_Auto` when you build it. Set the **source constant** above and the
preset follows.

| Timer | `.PRE` loaded from | Value | Loaded by |
|---|---|---|---|
| `Home0_Tmr` / `Home1_Tmr` | `HOME_TMO_MS` | `15000` ms | `R_HomeMotor0/1` (MOV each scan) |
| `VacTmr` | `VAC_SETTLE` | `300` ms | `R50_Auto` |
| `BlowTmr` | `BLOWOFF_TIME` | `200` ms | `R50_Auto` |

> Physical-I/O tags (E-stop, guards, limits, Z reed switches, and the
> `CylinderDown`/`VacuumOn`/`Blowoff` outputs and `EM806_*_ALM`) take no
> value — alias/map each to its real module point instead (see
> `plc_setup.md` §6).
