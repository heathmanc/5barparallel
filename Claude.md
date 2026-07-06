# Claude.md — Vision-Guided 5-Bar Bung-Cover Robot

> Context file for Claude Code. It describes the concept, the **verified**
> mechanical/kinematic design, the software boundaries, the code that already
> exists, and what to build next. Read §2 and §3 before changing anything
> geometric.

---

## 1. What this project is

A Python + OpenCV application driving a **vision-guided 5-bar parallel-SCARA
pick-and-place robot** that installs small plastic battery bung covers into the
vent holes of a **Group 31 (G31) battery** on an indexing conveyor.

One overhead camera sees the work area. The robot moves to a camera-clear pose,
Python detects the battery holes and the available loose covers, selects one
cover, solves the 5-bar inverse kinematics, and hands a single pick/place job to
a **CompactLogix PLC**, which executes the motion through a **Teknic ClearLink**
and **Leadshine stepper drives**. Z is pneumatic.

**Throughput target:** 6 covers in 30 s = 12 cycles/min = 5 s/cover. The battery
is **indexed** (registers against a hard stop), so the robot only has to reach
the hole pattern at one repeatable position, not a moving target.

---

## 2. Current status (READ FIRST)

| Area | Status |
|---|---|
| Architecture & hardware selection | **Locked** (see §6, §7) |
| Mechanical geometry (arm lengths, spacing, mount) | **Locked & verified** (see §3, §5) |
| `robot/fivebar_kinematics.py` | **Done, tested** |
| `robot/workspace.py` (singularity/reach guard) | **Done, tested** |
| `config/robot_config.yaml` | **Done** (verified values) |
| `tests/test_kinematics.py` | **Done** (encodes the design verification) |
| `vision/camera.py` (Basler/pypylon + mock) | **Done, tested** |
| `robot/driver.py`, `plc/{tags,compactlogix_client,plc_robot_driver}.py` | **Done, tested** (manual jog/home) |
| `app/robot_test_controller.py`, `gui/*` (Vision + Camera + Calibration + Robot Test + Settings + PLC tabs) | **Done, tested** |
| `vision/{calibration,detect_holes,detect_covers,detection}.py` | **Done, tested** |
| Interactive calibration (click correspondences → fit → save **per recipe**; feeds Vision reachability) | **Done, tested** |
| `app/recipes.py` + `config/recipes.yaml` (per-recipe calibration + hole count; Vision changeover) | **Done, tested** |
| `plc/handshake.py`, `robot/planner.py`, `app/cycle_manager.py` (full auto cycle, wired to Vision Start/Stop) | **Done, tested** |
| `main.py`, `app/diagnostics.py` | **To build** (see §14) |

Nothing hardware has been purchased yet, so geometry can still be trimmed if the
one open input (exact hole span, §17) turns out smaller — but the current design
is a safe **superset** and can be built to as-is.

---

## 3. Verified design decisions — do not silently revert

These came out of a full kinematic design review and **supersede the original
spec draft**. If a change seems to require undoing one of these, stop and flag
it.

1. **Mounting is INLINE, not across.** The two shoulders sit on a line
   **parallel to conveyor travel**, with the robot beside the belt reaching
   across it. Reason: a 5-bar's clean workspace is wide along the base line and
   shallow in the reach direction. The battery's long (13 in) axis must lie
   **along the base line**; only the ±2 in cross-conveyor spread lies in the
   reach direction. Mounting across the conveyor forces the 13 in into the reach
   direction and needs arms *longer* than the original 305 mm and ~3× the
   inertia. (Original draft said "motors across the conveyor" — that is wrong
   for this battery.)

2. **Arm links are L1 = 220 mm (proximal), L2 = 230 mm (distal).**
   Not 305 + 305. Shorter links because inline mounting only needs ~350 mm of
   reach. This drops arm inertia to **~40 % of the 305 + 305 design**, which is
   the main lever for hitting 5 s/cover, and it stiffens the arm.

3. **Base spacing is 101.6 mm (4 in), symmetric**: left base (−50.8, 0), right
   base (+50.8, 0) in robot frame. Not 14 in, not the asymmetric
   (−127, 228.6) from the draft. Narrower spacing gives larger singularity
   margin here.

4. **Assembly branch: left elbow "up", right elbow "down".** This is the
   best-conditioned branch — the symmetric branches (both-up / both-down) give a
   smaller, more lopsided clean region. Keep it.

5. **Mounting standoff ≈ 250 mm** (base line to hole row, across the belt). Puts
   the robot ~4 in outside the near belt edge. (250 rather than 260 keeps the
   ±2 in tolerance corners under the 85 % stiffness cap.)

6. **Workspace guard thresholds:** parallel-singularity margin ≥ 20°, serial
   (full-extension) margin ≥ 15°, TCP reach ≤ 85 % of full extension. Python
   must never send a target that fails these.

**Verified coverage:** the six holes + the cap pick point + the ±2 in
cross-conveyor tolerance all pass all five checks, worst-case ~31° parallel
margin, ~53° serial margin, ≤ 84 % reach. Largest fully-clean rectangle is
~**12 × 8 in**.

---

## 4. Coordinate frames — read before touching geometry

Two frames. Mixing them up is the most likely source of bugs.

**World / conveyor frame** (physical layout, for mounting & understanding):
- `X_world` = **along conveyor** (travel direction).
- `Y_world` = **across conveyor** (belt width). Belt is 12 in (304.8 mm) wide,
  centerline at `Y_world = 0`, edges at ±152.4.
- Battery (G31, 330 × 173 mm) registers its leading edge against the **stop at
  `X_world = 0`** and its body extends to **+X** (to `X_world = 330`).
- 6 bung holes: along the battery centerline (`Y_world ≈ 0`, ±2 in tolerance),
  spread in `X_world` across the battery.
- **Cap pick point: `X_world = −50, Y_world = 0`** — 50 mm "left" of the stop, on
  the conveyor centerline. (Off the battery's leading end.)
- Robot shoulder-line center: `X_world = 125, Y_world = −250` (beside the belt).

**Robot frame** (what the kinematics module uses):
- Shoulder bases lie on the **X axis** (`Y = 0`); mechanism reaches into **+Y**.
- Angles are CCW from +X, in degrees.

**Transform (nominal, inline mount → pure translation, no rotation):**
```
X_robot = X_world - 125
Y_robot = Y_world + 250
```
So every nominal target sits at `Y_robot = 250`; `X_robot` spans −175 (pick) to
+175 (far hole). **At runtime the real transform is the vision calibration**
(pixel → robot XY), established empirically per §13; the numbers above are the
design nominal.

---

## 5. Mechanical geometry (final)

| Parameter | Value |
|---|---|
| Proximal link L1 | 220 mm |
| Distal link L2 | 230 mm |
| Max reach per arm (L1+L2) | 450 mm |
| Base spacing | 101.6 mm (4 in), symmetric |
| Left base / right base (robot frame) | (−50.8, 0) / (+50.8, 0) |
| Assembly branch | left up, right down |
| Belt reduction | 3:1 GT3 (20T motor → 60T shoulder) |
| Drive pulses / motor rev | 3200 |
| **pulses_per_degree** | 3200 × 3 / 360 = **26.6667** |
| Joint limits (both shoulders) | −20° … +200° |
| Mount style / standoff | inline / ~250 mm |

Payload (one plastic cover) is negligible; arm mass is the design driver, hence
the short links.

---

## 6. Hardware stack

```
Overhead camera
  -> Python / OpenCV vision PC (detection, calibration, cover selection, 5-bar IK, workspace guard)
  -> CompactLogix 1769-L30ER PLC        (sequence, safety, pneumatics, motion commands)
  -> Teknic ClearLink CLNK-4-13         (EtherNet/IP from PLC; STEP/DIR/ENABLE out)
  -> Leadshine EM806 drives x2          (Axis0 = left shoulder, Axis1 = right shoulder)
  -> Leadshine 57HS22-07 NEMA 23 x2     (bipolar parallel)
  -> 3:1 GT3 belt reduction
  -> 5-bar linkage (L1=220 / L2=230 mm) mounted inline beside the conveyor
  -> centered pneumatic Z cylinder + vacuum cup (+ optional blowoff)
```

- The PLC owns the ClearLink interface (EDS/AOP/AOI). **Python never talks to the
  ClearLink directly.**
- ClearLink axis map: Axis 0 = left shoulder, Axis 1 = right shoulder,
  Axis 2/3 = spare. No motorized Z (pneumatic).
- Power: 48 VDC motor bus (fused per drive), 24 VDC control, kept separate.
- **Headroom note:** with arm inertia at ~40 % of the original design, the
  NEMA 23 / 48 V / 3:1 stack is now generously sized for 5 s/cover. Keep the
  headroom; it's cheap insurance for the cycle-time target.

---

## 7. Software architecture & responsibility split

**Python owns** (this repo):
- Vision: hole detection, cover detection (OpenCV first).
- Calibration: pixel → robot-frame XY (per-Z-plane homography).
- Cover selection (pick a safe, reachable, non-crowded cover).
- 5-bar **inverse kinematics** → shoulder angles (degrees).
- **Workspace/singularity validation** — the go/no-go guard before the PLC.
- PLC handshake orchestration (write targets, CommandID, request, wait).

**PLC owns** (separate Studio 5000 project, not in this repo):
- Robot sequence **state machine** (§11).
- ClearLink EtherNet/IP command/status; drive enable/fault.
- Home / limit / fault handling.
- Pneumatic cylinder extend/retract; vacuum + blowoff.
- E-stop / safety, operator start/stop/reset.

**Interface:** Python computes joint **angles**; the PLC + ClearLink execute the
moves. The PLC does not know 5-bar kinematics.

---

## 8. Repository layout & module status

```
bung_cover_5bar_robot/
  Claude.md                       # this file
  README.md
  requirements.txt
  pyproject.toml
  config/
    robot_config.yaml             # DONE (verified geometry + homing block)
    camera_config.yaml            # DONE (controls + intrinsics block)
    recipes.yaml                  # DONE (per-recipe: hole count, cover size)
  calibration/                    # per-recipe .npy homographies (git-ignored)
  src/bung_cover_robot/
    __init__.py
    main.py                       # TODO (CLI, --dry-run)
    app/
      robot_test_controller.py    # DONE, tested (headless jog/home logic)
      cycle_manager.py            # DONE, tested (auto cycle + job runners)
      recipes.py                  # DONE, tested (Recipe, RecipeStore)
      diagnostics.py              # TODO
    plc/
      tags.py                     # DONE, tested (single-source tag registry)
      compactlogix_client.py      # DONE, tested (pycomm3 wrapper + simulated)
      plc_robot_driver.py         # DONE, tested (manual jog/home driver)
      handshake.py                # DONE, tested (auto pick/place + timeout/recovery)
    robot/
      fivebar_kinematics.py       # DONE, tested
      workspace.py                # DONE, tested
      driver.py                   # DONE, tested (ABC + dry-run + homing)
      planner.py                  # DONE, tested (PickPlaceJob, make_job, sort_holes)
    vision/
      camera.py                   # DONE, tested (Basler/pypylon + mock)
      calibration.py              # DONE, tested (HomographyTransform, CalibrationManager)
      detect_holes.py             # DONE, tested (OpenCV blob + collinearity)
      detect_covers.py            # DONE, tested (OpenCV blob + reachability)
      detection.py                # DONE, tested (shared blob/ROI/annotate)
    gui/                          # DONE, tested (dark HMI: Vision, Camera,
      main_window.py              #   Calibration, Robot Test, Settings, PLC tabs;
      calibration_tab.py          #   click correspondences -> fit -> save)
      cycle_worker.py             #   threaded auto-cycle runner (Start/Stop)
  tests/
    test_kinematics.py            # DONE (+ camera, driver, plc, detection,
    ...                           #   calibration, controller, gui smoke)
```

---

## 9. Existing code: `robot/fivebar_kinematics.py` & `robot/workspace.py`

**`fivebar_kinematics.py`**
- `FiveBarConfig` — dataclass of geometry, joint limits, drivetrain;
  `from_yaml()` loads `config/robot_config.yaml`; `pulses_per_degree`,
  `max_reach_mm`, `base_spacing_mm`, etc. as properties. Defaults **are** the
  verified design point.
- `JointTarget` — result of a solve (`left_deg`, `right_deg`, pulse counts, TCP,
  elbow positions).
- `KinematicsError` — raised when a TCP is outside the reach envelope.
- `FiveBarKinematics`:
  - `inverse(x, y) -> JointTarget` — the workhorse. Solves each arm as a
    circle–circle intersection; picks the elbow branch from config; **normalizes
    shoulder angles into the joint's 360° window** (so a real 185° pose isn't
    mis-read by `atan2` as −175° and wrongly rejected).
  - `forward(left_deg, right_deg) -> (x, y)` — for round-trip checks/diagnostics.
  - `is_reachable(x, y) -> bool` — *geometric envelope only* (no limits/
    singularity — use the validator for the real go/no-go).
  - `degrees_to_pulses`, `within_joint_limits`.

**`workspace.py`**
- `SingularityLimits` — parallel ≥ 20°, serial ≥ 15°, reach ≤ 0.85 (defaults).
- `WorkspaceValidator`:
  - `parallel_margin_deg` — degrees from the two distal links being collinear
    (the dangerous **direct-kinematic** singularity that cuts through a 5-bar's
    workspace).
  - `serial_margin_deg` — degrees from an arm being straight/folded (workspace
    edge).
  - `reach_fraction` — TCP distance / full reach (stiffness proxy).
  - `validate(x, y) -> ValidationResult` — **the guard.** Never raises; returns
    `ok` + human-readable `reason` + metrics. Checks, in order: reachable →
    joint limits → parallel singularity → serial singularity → stiffness cap.
  - `is_safe(x, y) -> bool`; `scan(...)` (offline map generation only).

**Rule for all downstream code:** call `WorkspaceValidator.validate()` before a
target is ever written to the PLC. If it isn't `ok`, do not send it.

---

## 10. Kinematics & singularity model (the "why")

The 5-bar is two 2-link arms (L1 then L2) meeting at a shared TCP. Two failure
modes matter:

- **Parallel (direct-kinematic) singularity** — the two **distal** links become
  collinear. The mechanism loses the ability to resist/control force along that
  line; the FK Jacobian blows up and the arm can flip assembly modes. This forms
  a *band that arcs through the middle of the workspace* — it is the reason the
  work zone must be placed deliberately, and why inline mounting + these link
  lengths were chosen (they keep the whole battery + pick zone clear of it with
  ~31°+ margin).
- **Serial (inverse-kinematic) singularity** — an arm reaches full extension or
  full fold (proximal & distal collinear). This is the outer workspace boundary;
  near it the arm is compliant and placement accuracy degrades, hence the 85 %
  reach cap.

`workspace.py` measures the margin to both and rejects targets that get close.

---

## 11. PLC interface

Python ↔ CompactLogix via `pycomm3` (or `pylogix`). **Keep all tag names in
`plc/tags.py`.**

Recommended tags:
```
VisionRobot.Cmd.RequestPickPlace   BOOL
VisionRobot.Cmd.Abort              BOOL
VisionRobot.Cmd.Reset              BOOL
VisionRobot.Cmd.CommandID          DINT
VisionRobot.Target.Pick_LeftDeg    REAL
VisionRobot.Target.Pick_RightDeg   REAL
VisionRobot.Target.Drop_LeftDeg    REAL
VisionRobot.Target.Drop_RightDeg   REAL
VisionRobot.Target.HoleIndex       DINT
VisionRobot.Target.CoverID         DINT
VisionRobot.Status.Ready           BOOL
VisionRobot.Status.Busy            BOOL
VisionRobot.Status.Done            BOOL
VisionRobot.Status.Faulted         BOOL
VisionRobot.Status.FaultCode       DINT
VisionRobot.Status.ActiveCommandID     DINT
VisionRobot.Status.CompleteCommandID   DINT
VisionRobot.Status.FailedCommandID     DINT
VisionRobot.Status.VacuumOK        BOOL
VisionRobot.Status.CameraClear     BOOL
VisionRobot.Status.ReadyForVision  BOOL
```

**Handshake (Python side, `plc/handshake.py`):**
1. Wait for `ReadyForVision` / `Ready`.
2. Validate both pick & drop targets with `WorkspaceValidator` (abort the job if
   either fails — never send a bad target).
3. Write target tags, then increment & write `CommandID`, then set
   `RequestPickPlace`.
4. Wait for `CompleteCommandID == CommandID`.
5. **Timeout/recovery (important):** if `command_timeout_s` elapses with neither
   `Complete` nor a clean `Faulted`, treat it as its own recoverable error
   state — do not hang. (This was an explicit gap in the original draft.)
6. On `Faulted`, stop and report `FaultCode`.

`CommandID` exists to reject stale/duplicated commands.

**Manual jog/home surface (`VisionRobot.Manual.*`, for the Robot Test tab):**
Separate from the automatic pick/place handshake above. Jog is
**absolute-incremental** — Python computes a *validated* absolute angle target
and the PLC does one coordinated move to it (never a continuous velocity jog, so
every step is singularity-checked before motion).
```
VisionRobot.Manual.Enable         BOOL   VisionRobot.Status.Enabled          BOOL
VisionRobot.Manual.HomeRequest    BOOL   VisionRobot.Status.Homed            BOOL
VisionRobot.Manual.MoveToTarget   BOOL   VisionRobot.Status.InPosition       BOOL
VisionRobot.Manual.Abort          BOOL   VisionRobot.Status.Moving           BOOL
VisionRobot.Manual.TargetLeftDeg  REAL   VisionRobot.Status.ActualLeftDeg    REAL
VisionRobot.Manual.TargetRightDeg REAL   VisionRobot.Status.ActualRightDeg   REAL
VisionRobot.Manual.CommandID      DINT   VisionRobot.Status.CompleteCommandID DINT
```
- `enable`: write `Manual.Enable`, wait `Status.Enabled`.
- `home`: pulse `Manual.HomeRequest`, wait `Status.Homed`; PLC runs the homing
  routine and reports the reference angles in `ActualLeft/RightDeg`.
- `move_to_angles`: write `TargetLeft/RightDeg`, bump `Manual.CommandID`, pulse
  `MoveToTarget`, wait `CompleteCommandID == CommandID` **and** `InPosition`.
Python side is `plc/plc_robot_driver.py`; `plc/compactlogix_client.py` includes a
`SimulatedPlcClient` that emulates this ladder for `--sim-plc` and tests. The
Studio 5000 implementation outline (UDT, AOIs, manual + auto state machines,
homing, faults) is in `docs/plc_program.md`.

**Homing & limit switches (open-loop steppers → homing is mandatory each
power-up).** Because of the 3:1 reduction, the home flag must sense the
**shoulder** rotation (60T pulley / L1 root), **not** the motor shaft (a
motor-side flag is 3× ambiguous). Per shoulder:
- **1 home/reference sensor** (inductive prox on the stationary base, flag on the
  rotating shoulder hub) — mandatory. Approach one direction, back off, re-approach
  slow for repeatability, then set the known home angle.
- **2 hard overtravel limits** near the −20°/+200° ends, wired into the drive
  ENABLE/fault chain (acts even if PLC logic hangs). Minimum viable = home switch
  + PLC soft limits.
Choose a **home pose with the arms spread (left-up/right-down splay), clear of the
parallel-singularity band**, so both arms reach home without colliding. Homing is
PLC-owned (§7); the Python `home()` just triggers it and waits for `Homed`.

**PLC state machine (Studio 5000, for reference):**
`0 IDLE → 10 MOVE_CAMERA_CLEAR → 20 WAIT_CAMERA_CLEAR → 30 WAIT_FOR_VISION_COMMAND
→ 40 LOAD_TARGETS → 50 MOVE_ABOVE_PICK → 60 WAIT_ABOVE_PICK → 70 CYLINDER_DOWN_PICK
→ 80 WAIT_PICK_DOWN → 90 VACUUM_ON → 100 VERIFY_VACUUM → 110 CYLINDER_UP_PICK
→ 120 WAIT_PICK_UP → 130 MOVE_ABOVE_DROP → 140 WAIT_ABOVE_DROP → 150 CYLINDER_DOWN_DROP
→ 160 WAIT_DROP_DOWN → 170 VACUUM_OFF_BLOWOFF → 180 CYLINDER_UP_DROP → 190 WAIT_DROP_UP
→ 200 COMPLETE_JOB → (900 FAULT)`
Advance on real status bits, not timing guesses. Timers only for vacuum settle,
blowoff, and debounce.

---

## 12. Vision strategy

- **OpenCV first** (round holes, round covers, fixed overhead camera, controlled
  lighting, robot clears the frame before capture). Hold **YOLO in reserve** for
  if OpenCV proves unreliable.
- **Re-image before each pick** — loose covers shift when neighbors are removed,
  so do not blindly queue all cover locations from one image.
- Cycle: robot → camera-clear pose → capture → detect holes (once per battery is
  fine) → detect covers (re-validate each cycle) → pick one safe cover → IK →
  validate → one job to PLC → repeat for 6 holes.
- **Hole detection** (`detect_holes.py`): ROI crop → gray → blur → threshold/
  edge → contours → circularity + diameter filter → line-fit the 6 centers
  (holes are collinear along the battery) as a sanity check.
- **Cover detection** (`detect_covers.py`): ROI crop → gray → threshold →
  blob/contour → circularity + area filter → reject covers that are touching,
  partially hidden, near a tray edge, or outside the reachable workspace
  (test with `WorkspaceValidator`).

---

## 13. Calibration & coordinate flow

**The vent holes and the loose covers sit on the same plane.** A battery-type
**changeover** shifts that plane (and the hole pattern), so calibration is
**one homography per battery recipe** — the *same* transform maps both the holes
and the covers for the loaded battery type. Add a one-time **lens-undistortion**
step (`cv2.undistortPoints` with the camera intrinsics) **before** the homography
— without it, a 2592×1944 sensor's corners can be several pixels off, which can
exceed the placement tolerance.

```
pixel point
  -> undistort -> homography (active recipe) -> ROBOT-frame XY
  -> WorkspaceValidator.validate()           (reject if not ok)
  -> FiveBarKinematics.inverse()             -> left/right shoulder degrees
  -> PLC target tags
```

Each **recipe owns its own calibration** — `calibration/<recipe_key>.npy`
(git-ignored). `CalibrationManager` is keyed by recipe: `has(key)` / `get(key)` /
`save(key, t)` / `keys()`. Recipes themselves live in `config/recipes.yaml`
(`app/recipes.py`: `Recipe`, `RecipeStore`), each carrying its vent-hole count
and nominal cover size. At **changeover** the Vision tab loads the selected
recipe's calibration + hole count; the `CycleManager` takes that one active
``calibration`` and applies it to both holes and covers.

**Building one interactively:** the **Calibration tab** (`gui/calibration_tab.py`)
is the operator workflow — pick the recipe (or add a new one), capture a frame,
click each known point and type its robot-frame XY (mm), **fit** the homography
(≥4 non-collinear points; it reports the RMS residual in mm), then **save** it to
that recipe. A saved calibration is broadcast (recipe key + transform) to the
Vision tab, which adopts it live when it's the active recipe. Intrinsics for the
pre-homography undistortion are read from `config/camera_config.yaml`.

---

## 14. What to build next (priority order)

Done so far: `plc/{tags,compactlogix_client,plc_robot_driver,handshake}.py`,
`robot/{driver,planner}.py`, `vision/{camera,calibration,detect_holes,detect_covers,detection}.py`,
`app/{robot_test_controller,cycle_manager,recipes}.py`, `config/recipes.yaml`,
and the full `gui/*` HMI (incl. per-recipe interactive calibration + the
Start/Stop-wired automatic cycle, which runs on a worker thread —
`gui/cycle_worker.py` — so a multi-second PLC handshake never
blocks the HMI; Stop halts after the current pick). The loop is closed:
**detect → calibrate (pixel→robot) → validate → plan → PLC pick/place handshake
→ re-image**, running in dry-run, `--sim-plc`, or on a real PLC. Remaining:

1. **`app/diagnostics.py`** — save annotated images on any detection/validation
   failure.
2. **`main.py`** — CLI entry, `--config`, `--dry-run` / `--sim-plc`.

(`config/recipes.yaml` + `app/recipes.py` are done — per-recipe calibration,
hole count, and changeover are wired into the Calibration and Vision tabs.)

Keep growing `tests/` alongside (planner logic, calibration round-trips, a
dry-run cycle-manager smoke test).

---

## 15. Coding standards

- Type hints everywhere.
- Hardware I/O behind wrapper classes; support **`--dry-run`** with no PLC.
- **Never send an unreachable / near-singular / over-extended target to the PLC**
  — always gate on `WorkspaceValidator.validate()`.
- **Never use fixed sleeps to assume motion is complete** — advance on PLC status
  bits.
- Keep PLC tag names in one module (`plc/tags.py`).
- Keep calibration data out of source (`calibration/*.npy`, git-ignored).
- Save diagnostic images when detection/validation fails.
- Clear, specific exceptions for camera, PLC, IK, and detection failures.

---

## 16. Commands

```bash
# setup
python -m venv .venv
# Windows: .venv\Scripts\activate   |   *nix: source .venv/bin/activate
pip install -e .              # installs the package (makes imports + pytest work)
pip install -r requirements.txt

# test
pytest -q                     # kinematics + workspace are covered today

# run
python -m bung_cover_robot.main --config config/robot_config.yaml --dry-run
python -m bung_cover_robot.main --config config/robot_config.yaml        # live
```

---

## 17. Open inputs & how they affect the design

Only **one** geometric input is still soft:

- **Exact cap-to-cap hole span on the G31.** The current design was sized to the
  full 13 in battery length (worst case), so it **already covers** any shorter
  real span — building to it is safe. If the six vent caps actually span less
  (the original draft implied ~7.5 in at 38 mm pitch, which conflicts with the
  13 in length), the arms could shrink further and margins grow. Confirm the
  real hole positions during commissioning and, if desired, re-run the sizing
  (the workspace scan in `workspace.py` reproduces the design maps).

Everything else (mounting style, arm lengths, spacing, standoff, assembly
branch, pick point at −50 mm, workspace thresholds) is fixed and verified.

---

## Appendix A — Design-review findings (why the geometry is what it is)

- A 5-bar has a **parallel-singularity band** arcing through its workspace; the
  original draft's geometry put the hole line and pickup **inside** it. Fixed by
  placing the work zone in the clean outer region and by the choices below.
- **Base spacing is a weak lever** (±100 mm of spacing moves the band only
  ~20 mm, and *wider* is worse). **Arm length is the strong lever.**
- **Inline vs across mounting** for the 13 in battery: across needs ~300+345 mm
  arms at ~120 % of the original inertia; inline needs ~220+230 mm at ~40 %.
  Inline wins on reach, inertia, and margin. Angling the mount off inline only
  hurts.
- **Arm size scales with the along-conveyor reach.** Because the battery is
  indexed, that reach is the hole-pattern span, not a 12 in travel window — which
  is what makes the short, light arm possible.
- **Assembly branch** left-up/right-down beats both symmetric branches for
  usable clean area.
- Net: inline, 220+230 mm, 4 in spacing, ~250 mm standoff → the whole battery +
  pick zone is singularity-free with ~31°+ margin, largest clean rectangle
  ~12 × 8 in, arm inertia ~40 % of the original 305+305 spec (the headroom that
  makes 5 s/cover realistic).

## Appendix B — Glossary

- **TCP** — tool center point (the vacuum-cup center; the shared distal joint).
- **Proximal / distal link** — shoulder→elbow (L1) / elbow→TCP (L2).
- **Assembly branch (elbow up/down)** — which circle-intersection solution an arm
  uses; a 5-bar has independent branches per arm.
- **Parallel singularity** — distal links collinear (direct-kinematic; mid-
  workspace band).
- **Serial singularity** — an arm straight or folded (inverse-kinematic;
  workspace edge).
- **Reach fraction** — TCP distance / (L1+L2); stiffness proxy, capped at 0.85.
- **Indexed** — the battery stops at a repeatable position against a hard stop.
```