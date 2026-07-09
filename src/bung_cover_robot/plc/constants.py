"""Commissioning constants — the tuning/reference values an operator sets once
on the PLC and would lose if the controller is reloaded or its memory cleared.

These are the ``set_by_hand`` controller-scope tags emitted by
``scripts/render_plc_l5x.py`` (excluding the read-only ``Constant := true``
STEPS_PER_DEG). Studio 5000 does not restore them from the program download, so
this module lets the app snapshot the live values (disaster-recovery backup) and
push a saved set back over EtherNet/IP in one shot.

The registry here is the runtime mirror of the generator's ``_glue_tags()``;
``tests/test_plc_constants.py`` asserts the two never drift.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

from .compactlogix_client import PlcClient, PlcError

try:
    import yaml
except ImportError:  # pragma: no cover - persistence needs PyYAML
    yaml = None  # type: ignore[assignment]


@dataclass(frozen=True)
class PlcConstant:
    """One writable commissioning tag (controller-scope, addressed by bare name)."""

    name: str
    dtype: str          # "DINT" or "REAL"
    default: float
    unit: str
    desc: str

    def coerce(self, value) -> object:
        """Cast a Python value to the tag's PLC type (DINT -> int, REAL -> float)."""
        if self.dtype == "DINT":
            return int(round(float(value)))
        return float(value)


# The set-by-hand scalars from render_plc_l5x._glue_tags(), in a sensible
# commissioning order. Defaults mirror the generator exactly (drift-guarded).
COMMISSIONING_CONSTANTS: List[PlcConstant] = [
    PlcConstant("MOVE_VEL", "DINT", 20000, "steps/s", "Move speed (max 500000)."),
    PlcConstant("MOVE_ACC", "DINT", 100000, "steps/s^2", "Move accel."),
    PlcConstant("MOVE_DEC", "DINT", 0, "steps/s^2", "Move decel (0 => use accel)."),
    PlcConstant("HOME_VEL_0", "DINT", -2000, "steps/s", "Motor 0 homing speed (signed toward prox)."),
    PlcConstant("HOME_VEL_1", "DINT", 2000, "steps/s", "Motor 1 homing speed (signed toward prox)."),
    PlcConstant("HOME_ACC", "DINT", 50000, "steps/s^2", "Homing accel."),
    PlcConstant("HOME_TMO_MS", "DINT", 15000, "ms", "Homing timeout (Home*_Tmr.PRE)."),
    PlcConstant("HOME_OFFSET_L", "DINT", 3748, "steps", "Left switch angle * STEPS_PER_DEG (nominal; refine at commissioning)."),
    PlcConstant("HOME_OFFSET_R", "DINT", 1052, "steps", "Right switch angle * STEPS_PER_DEG (nominal; refine at commissioning)."),
    PlcConstant("HOME_ANGLE_L", "REAL", 140.5406, "deg", "Left home angle published on a bypass home."),
    PlcConstant("HOME_ANGLE_R", "REAL", 39.4594, "deg", "Right home angle published on a bypass home."),
    PlcConstant("VAC_SETTLE", "DINT", 300, "ms", "Vacuum settle time (VacTmr preset)."),
    PlcConstant("BLOWOFF_TIME", "DINT", 200, "ms", "Blowoff time (BlowTmr preset)."),
    PlcConstant("CAMERA_CLEAR_L", "REAL", 0.0, "deg", "Camera-clear pose, left shoulder deg."),
    PlcConstant("CAMERA_CLEAR_R", "REAL", 0.0, "deg", "Camera-clear pose, right shoulder deg."),
    PlcConstant("HB_TIMEOUT_MS", "DINT", 1000, "ms", "PC-heartbeat watchdog timeout."),
    PlcConstant("EN_DROP_TMO_MS", "DINT", 1000, "ms", "Enable drop-out (power-cycle) debounce."),
]

_BY_NAME: Dict[str, PlcConstant] = {c.name: c for c in COMMISSIONING_CONSTANTS}


def default_values() -> Dict[str, float]:
    """The generator defaults, keyed by tag name."""
    return {c.name: c.default for c in COMMISSIONING_CONSTANTS}


@dataclass
class PushResult:
    """Outcome of writing one constant to the PLC."""

    name: str
    ok: bool
    error: str = ""


def push_constants(client: PlcClient, values: Dict[str, float]) -> List[PushResult]:
    """Write every known constant to the PLC (missing keys fall back to default).

    Never raises for a per-tag failure — each tag's outcome is reported so a
    partial restore is visible. Coerces to the tag's PLC type first.
    """
    results: List[PushResult] = []
    for c in COMMISSIONING_CONSTANTS:
        raw = values.get(c.name, c.default)
        try:
            client.write(c.name, c.coerce(raw))
            results.append(PushResult(c.name, True))
        except (PlcError, Exception) as exc:  # noqa: BLE001 - report, don't abort
            results.append(PushResult(c.name, False, str(exc)))
    return results


def read_constants(client: PlcClient) -> Dict[str, float]:
    """Snapshot the live PLC value of every constant (skips tags that error)."""
    out: Dict[str, float] = {}
    for c in COMMISSIONING_CONSTANTS:
        try:
            out[c.name] = c.coerce(client.read(c.name))
        except (PlcError, Exception):  # noqa: BLE001 - a missing tag just isn't snapshotted
            continue
    return out


class PlcConstantStore:
    """Persisted commissioning values, backed by ``config/plc_constants.yaml``.

    Any unknown/absent tag reads back as its generator default, so a partial or
    missing file is still a complete, pushable set.
    """

    def __init__(self, values: Optional[Dict[str, float]] = None,
                 path: Optional[str | Path] = None) -> None:
        self.path: Optional[Path] = Path(path) if path else None
        merged = default_values()
        if values:
            merged.update({k: v for k, v in values.items() if k in _BY_NAME})
        self._values = merged

    @classmethod
    def load(cls, path: str | Path) -> "PlcConstantStore":
        p = Path(path)
        if not p.exists():
            return cls(path=p)
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required to load PLC constants")
        data = yaml.safe_load(p.read_text()) or {}
        return cls(data.get("values", data), path=p)

    def as_dict(self) -> Dict[str, float]:
        return dict(self._values)

    def get(self, name: str) -> float:
        return self._values[name]

    def set(self, name: str, value: float) -> None:
        if name not in _BY_NAME:
            raise KeyError(f"unknown constant {name!r}")
        self._values[name] = _BY_NAME[name].coerce(value)

    def update(self, values: Dict[str, float]) -> None:
        for name, value in values.items():
            if name in _BY_NAME:
                self.set(name, value)

    def save(self) -> Path:
        if self.path is None:
            raise RuntimeError("PlcConstantStore has no path to save to")
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required to save PLC constants")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(yaml.safe_dump({"values": self._values}, sort_keys=True))
        return self.path
