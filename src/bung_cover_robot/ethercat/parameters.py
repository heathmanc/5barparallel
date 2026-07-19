"""Drive/motion setup parameters — the settable table behind the Drives tab.

Two kinds of parameter share one registry (the runtime analog of the old PLC
commissioning-constants table):

  * ``scope="motion"`` — PC-side motion planning values (TrajectoryLimits and
    friends). Applied by rebuilding the live driver's limits.
  * ``scope="drive"``  — CiA 402 object-dictionary entries written to BOTH
    drives over SDO (index/sub recorded here). Against the simulator the write
    is a no-op; the real pysoem master pushes them at Apply / on connect
    (Stage 4 bench).

Values persist in ``config/drive_parameters.yaml``; a missing file means "all
defaults", so the table is always complete and pushable.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]


@dataclass(frozen=True)
class DriveParameter:
    name: str
    scope: str            # "motion" (PC-side) | "drive" (CiA 402 SDO)
    dtype: str            # "int" | "float"
    default: float
    unit: str
    desc: str
    sdo: Optional[Tuple[int, int]] = None   # (index, subindex) for scope="drive"

    def coerce(self, value) -> float:
        return int(round(float(value))) if self.dtype == "int" else float(value)


PARAMETERS: List[DriveParameter] = [
    # --- PC-side motion planning (TrajectoryLimits) --------------------------
    DriveParameter("speed_mm_s", "motion", "float", 200.0, "mm/s",
                   "Cartesian cruise speed for planned moves."),
    DriveParameter("accel_mm_s2", "motion", "float", 2000.0, "mm/s^2",
                   "Cartesian accel/decel for the trapezoid profile."),
    DriveParameter("cycle_dt_s", "motion", "float", 0.002, "s",
                   "EtherCAT DC cycle time; must match the master."),
    DriveParameter("max_joint_step_deg", "motion", "float", 0.0, "deg",
                   "Per-cycle shoulder step cap (0 = off) - singularity guard."),
    DriveParameter("position_tol_counts", "motion", "int", 5, "counts",
                   "End-of-move following-error tolerance."),
    # --- CiA 402 drive objects (written to both drives) ----------------------
    DriveParameter("homing_method", "drive", "int", 24, "-",
                   "0x6098 homing method (switch + index pulse).", sdo=(0x6098, 0)),
    DriveParameter("homing_speed_fast", "drive", "int", 20000, "counts/s",
                   "0x6099:1 speed while searching the switch.", sdo=(0x6099, 1)),
    DriveParameter("homing_speed_slow", "drive", "int", 2000, "counts/s",
                   "0x6099:2 speed while searching the index.", sdo=(0x6099, 2)),
    DriveParameter("following_error_window", "drive", "int", 4000, "counts",
                   "0x6065 max following error before the drive faults.", sdo=(0x6065, 0)),
    DriveParameter("quick_stop_decel", "drive", "int", 200000, "counts/s^2",
                   "0x6085 deceleration on quick stop / abort.", sdo=(0x6085, 0)),
]

_BY_NAME: Dict[str, DriveParameter] = {p.name: p for p in PARAMETERS}


def _hexint(t: str) -> int:
    t = t.strip()
    return int(t, 0) if t.lower().startswith("0x") else int(t, 16)


def parse_drive_address(text: str) -> Tuple[int, int]:
    """(index, subindex) from a drive-parameter address. Accepts the drive's
    friendly ``Cxx.NN`` form — mapped to CoE object ``0x20xx : NN+1`` (the rule
    derived from the A6-EC parameter list, e.g. C0A.08 -> 0x200A:09) — or a raw
    ``INDEX:SUB`` hex CoE address (``0x`` optional)."""
    s = text.strip().replace(" ", "")
    if s[:1] in "Cc" and "." in s:
        grp, nn = s[1:].split(".", 1)
        return 0x2000 + int(grp, 16), int(nn, 16) + 1
    if ":" in s:
        i, sub = s.split(":", 1)
        return _hexint(i), _hexint(sub)
    raise ValueError(f"cannot parse drive address {text!r} — use Cxx.NN or 0xINDEX:SUB")


@dataclass(frozen=True)
class CustomParameter:
    """A user-added drive object (e.g. a gain), written over SDO on Apply."""

    name: str
    index: int
    sub: int
    value: float
    dtype: str = "int"

    def coerce(self, v) -> float:
        return int(round(float(v))) if self.dtype == "int" else float(v)

    @property
    def address(self) -> str:
        return f"0x{self.index:04X}:{self.sub}"


def default_values() -> Dict[str, float]:
    return {p.name: p.default for p in PARAMETERS}


class ParameterStore:
    """Persisted parameter values; unknown/absent keys read as defaults."""

    def __init__(self, values: Optional[Dict[str, float]] = None,
                 path: Optional[str | Path] = None,
                 custom: Optional[List[CustomParameter]] = None) -> None:
        self.path: Optional[Path] = Path(path) if path else None
        merged = default_values()
        if values:
            merged.update({k: _BY_NAME[k].coerce(v) for k, v in values.items()
                           if k in _BY_NAME})
        self._values = merged
        self._custom: List[CustomParameter] = list(custom or [])

    @classmethod
    def load(cls, path: str | Path) -> "ParameterStore":
        p = Path(path)
        if not p.exists():
            return cls(path=p)
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required to load drive parameters")
        data = yaml.safe_load(p.read_text()) or {}
        custom = [CustomParameter(name=str(c["name"]), index=int(c["index"]),
                                  sub=int(c["sub"]), value=c.get("value", 0),
                                  dtype=str(c.get("dtype", "int")))
                  for c in data.get("custom", []) if "name" in c]
        return cls(data.get("values", data), path=p, custom=custom)

    def as_dict(self) -> Dict[str, float]:
        return dict(self._values)

    def get(self, name: str) -> float:
        return self._values[name]

    def set(self, name: str, value) -> None:
        if name not in _BY_NAME:
            raise KeyError(f"unknown parameter {name!r}")
        self._values[name] = _BY_NAME[name].coerce(value)

    # --- custom (user-added) drive parameters ------------------------------- #
    def custom_parameters(self) -> List[CustomParameter]:
        return list(self._custom)

    def add_custom(self, name: str, address: str, value=0, dtype: str = "int") -> CustomParameter:
        """Add/replace a custom drive object addressed by Cxx.NN or INDEX:SUB."""
        index, sub = parse_drive_address(address)
        val = int(round(float(value))) if dtype == "int" else float(value)
        cp = CustomParameter(name=name.strip(), index=index, sub=sub, value=val, dtype=dtype)
        if not cp.name:
            raise ValueError("parameter name is required")
        self._custom = [c for c in self._custom if c.name != cp.name] + [cp]
        return cp

    def set_custom_value(self, name: str, value) -> None:
        for i, c in enumerate(self._custom):
            if c.name == name:
                self._custom[i] = CustomParameter(c.name, c.index, c.sub,
                                                  c.coerce(value), c.dtype)
                return
        raise KeyError(f"unknown custom parameter {name!r}")

    def remove_custom(self, name: str) -> None:
        self._custom = [c for c in self._custom if c.name != name]

    def save(self) -> Path:
        if self.path is None:
            raise RuntimeError("ParameterStore has no path to save to")
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required to save drive parameters")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "values": self._values,
            "custom": [{"name": c.name, "index": c.index, "sub": c.sub,
                        "dtype": c.dtype, "value": c.value} for c in self._custom],
        }
        self.path.write_text(yaml.safe_dump(payload, sort_keys=True))
        return self.path

    # --- application --------------------------------------------------------
    def trajectory_limits(self):
        """Build TrajectoryLimits from the motion-scope values."""
        from .trajectory import TrajectoryLimits

        cap = self.get("max_joint_step_deg")
        return TrajectoryLimits(
            speed_mm_s=self.get("speed_mm_s"),
            accel_mm_s2=self.get("accel_mm_s2"),
            cycle_dt_s=self.get("cycle_dt_s"),
            max_joint_step_deg=None if cap <= 0 else cap,
        )

    def apply(self, driver) -> List[str]:
        """Apply to a live EtherCatRobotDriver: motion limits rebuild in place;
        drive-scope SDOs are written when the master supports it (the simulator
        doesn't - those report as 'stored (sim)'). Returns per-item messages."""
        notes: List[str] = []
        driver.limits = self.trajectory_limits()
        driver.position_tol_counts = int(self.get("position_tol_counts"))
        notes.append("motion limits applied to driver")
        sdo_write = getattr(driver.master, "sdo_write", None)
        for p in PARAMETERS:
            if p.scope != "drive":
                continue
            if callable(sdo_write):  # pragma: no cover - real master (Stage 4)
                try:
                    sdo_write(p.sdo[0], p.sdo[1], int(self.get(p.name)))
                    notes.append(f"{p.name}: written 0x{p.sdo[0]:04X}:{p.sdo[1]}")
                except Exception as exc:  # noqa: BLE001
                    notes.append(f"{p.name}: WRITE FAILED - {exc}")
            else:
                notes.append(f"{p.name}: stored (sim master has no SDO channel)")
        for c in self._custom:
            if callable(sdo_write):  # pragma: no cover - real master
                try:
                    sdo_write(c.index, c.sub, int(c.value), size=(1 if c.dtype == "int8"
                              else 2 if c.dtype == "int16" else 4))
                    notes.append(f"{c.name}: written {c.address}")
                except Exception as exc:  # noqa: BLE001
                    notes.append(f"{c.name}: WRITE FAILED - {exc}")
            else:
                notes.append(f"{c.name}: stored (sim master has no SDO channel)")
        return notes
