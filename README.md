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
| `tests/test_kinematics.py` | done |
| `plc/*`, `vision/*`, `robot/planner.py`, `app/*`, `main.py` | to build (Claude.md §14) |

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
