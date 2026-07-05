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
| `tests/test_kinematics.py`, `tests/test_camera.py` | done |
| `plc/*`, `vision/{calibration,detect_*}.py`, `robot/planner.py`, `app/*`, `main.py` | to build (Claude.md §14) |

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
