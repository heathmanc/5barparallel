# Vision-Guided 5-Bar Bung-Cover Robot

Python + OpenCV control for a vision-guided **5-bar parallel-SCARA** pick-and-place
robot that installs plastic battery bung covers into the vent holes of a Group 31
battery on an indexing conveyor. An overhead camera detects holes and loose covers;
Python solves the 5-bar inverse kinematics, validates the target against the
workspace/singularity guard, and hands one pick/place job to a CompactLogix PLC.

See [`Claude.md`](Claude.md) for the full design, the verified geometry, and the
hardware/software responsibility split.

## Status

Only the **verified kinematics foundation** is implemented so far:

| Module | Status |
|---|---|
| `config/robot_config.yaml` | done — verified geometry |
| `src/bung_cover_robot/robot/fivebar_kinematics.py` | done, tested |
| `src/bung_cover_robot/robot/workspace.py` | done, tested |
| `config/camera_config.yaml` | done |
| `src/bung_cover_robot/vision/camera.py` | done, tested — Basler (pypylon) + mock |
| `src/bung_cover_robot/robot/driver.py` | done, tested — dry-run + PLC-backed drivers |
| `src/bung_cover_robot/plc/{tags,compactlogix_client,plc_robot_driver}.py` | done, tested — jog/home over EtherNet/IP + simulator |
| `src/bung_cover_robot/app/robot_test_controller.py` | done, tested — reference + home + jog logic |
| `src/bung_cover_robot/gui/` | done — PySide6 HMI, Robot Test + Settings tabs |
| `tests/test_*.py` | done (kinematics, camera, driver, controller, PLC, GUI smoke) |
| `plc/handshake.py`, `vision/{calibration,detect_*}.py`, `robot/planner.py`, `app/cycle_manager.py`, `main.py` | to build (Claude.md §14) |

## Setup

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -e .[dev]              # installs the package + pytest
```

## Test

```bash
pytest -q
```

The tests encode the design verification from `Claude.md` §3: the whole work zone
(six holes + cap pick point + ±2 in cross-conveyor tolerance) clears every
singularity and reach check.

## Quick use

```python
from bung_cover_robot.robot import FiveBarKinematics, WorkspaceValidator

kin = FiveBarKinematics()             # verified default geometry
validator = WorkspaceValidator(kin)

x, y = 0.0, 250.0                     # robot-frame TCP target (mm)
result = validator.validate(x, y)     # ALWAYS gate on this before the PLC
if result.ok:
    target = kin.inverse(x, y)
    print(target.left_deg, target.right_deg)
else:
    print("rejected:", result.reason)
```

**Rule:** never send a target to the PLC without `WorkspaceValidator.validate()`
returning `ok` (Claude.md §9, §15).

## Camera (Basler)

Native Basler controls are reached through Basler's `pypylon` SDK; frames come
back as OpenCV-native BGR `numpy` arrays. A `MockCamera` provides synthetic
frames so the pipeline runs with no hardware (`--dry-run`).

```python
from bung_cover_robot.vision import (
    open_camera, CameraConfig, CameraControls,
)

config = CameraConfig.from_yaml("config/camera_config.yaml")
controls = CameraControls.from_yaml("config/camera_config.yaml")

# mock=True for dry-run; drop it (or pass mock=False) for a real Basler.
with open_camera(config, controls, mock=True) as cam:
    frame = cam.grab()                       # OpenCV BGR ndarray

    # Exposed controls — by logical name, resolved to the right GenICam node:
    cam.set_control("exposure_time_us", 6000.0)
    cam.set_control("brightness", 0.2)
    cam.set_control("contrast", 1.1)
    cam.set_control("gain", 3.0)
    print(cam.get_control("exposure_time_us"))
```

Logical control names (`brightness`, `contrast`, `exposure_time_us`, `gain`,
`gamma`, `black_level`, `saturation`, `sharpness`, ROI, orientation, …) are
mapped to per-model GenICam nodes in `CONTROL_REGISTRY`; you can also pass a raw
GenICam node name or extra nodes via `CameraControls(extra={...})`. On a real
camera, `BaslerCamera.list_devices()` enumerates connected cameras and
`control_range(name)` returns a control's `(min, max)` for building sliders.

## GUI (robot HMI)

A PySide6 tabbed HMI. The **Robot Test** tab establishes home and jogs the
robot; every move is gated by `WorkspaceValidator` before it reaches the driver.

```bash
pip install -e .[gui]
python -m bung_cover_robot.gui                       # in-process dry-run sim
python -m bung_cover_robot.gui --sim-plc             # PLC driver + simulated PLC
python -m bung_cover_robot.gui --plc 192.168.1.10/0  # PLC driver + real CompactLogix
```

**Robot Test tab:**
- **Drives** — Enable / STOP. Motion is refused while disabled.
- **Home** — *Home (find ref)* runs the hardware homing routine (find the home
  switches) and adopts the reference pose; *Set Home (teach)* captures the
  current pose as the software home; *Go Home* drives back to it. Jogging
  requires the robot to be **referenced** first.
- **Jog** — per-shoulder joint jog (L±, R±) and Cartesian TCP jog (X±, Y± in the
  robot frame), with independent joint-step (deg) and Cartesian-step (mm) sizes.
  *Absolute-incremental*: each press computes a new absolute target, validates
  it, and commands one coordinated move.
- **Position / workspace** — live TCP, shoulder angles, drive pulses, and the
  parallel/serial singularity margins + reach fraction. A jog that would leave
  the clean workspace is rejected with the reason; the robot doesn't move.

**Settings tab** — view/edit mechanical geometry (L1, L2, spacing, joint limits,
branch, drivetrain) and the workspace guard thresholds. *Validate & Apply*
re-runs the full work-zone validation and **refuses** geometry that can't clear
every singularity/reach check (Claude.md §3); *Save to YAML* persists only
validated geometry to `config/robot_config.yaml`.

The GUI is a thin view over the headless `RobotTestController`, which drives a
swappable `RobotDriver`:
- `DryRunRobotDriver` — in-process simulation.
- `PlcRobotDriver` — manual jog/home over EtherNet/IP (pycomm3), using the
  `VisionRobot.Manual.*` tag surface (`plc/tags.py`). A `SimulatedPlcClient`
  emulates the ladder so `--sim-plc` runs the real handshake with no hardware.
  To drive a real robot, the Studio 5000 program must implement those tags +
  the homing routine (see the PLC contract note below).
