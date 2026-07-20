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
    size: int = 4                           # CoE object byte length (1/2/4)

    def coerce(self, value) -> float:
        return int(round(float(value))) if self.dtype == "int" else float(value)


PARAMETERS: List[DriveParameter] = [
    # --- PC-side motion planning (TrajectoryLimits) --------------------------
    DriveParameter("speed_mm_s", "motion", "float", 200.0, "mm/s",
                   "Cartesian cruise speed for planned moves."),
    DriveParameter("accel_mm_s2", "motion", "float", 2000.0, "mm/s^2",
                   "Cartesian accel/decel for the trapezoid profile."),
    DriveParameter("jerk_mm_s3", "motion", "float", 80000.0, "mm/s^3",
                   "S-curve jerk limit: eases acceleration in over ~accel/jerk s "
                   "to kill the start-of-move chirp + following-error spike. "
                   "0 = off (hard trapezoid). Lower = gentler/quieter but adds a "
                   "little move time."),
    DriveParameter("cycle_dt_s", "motion", "float", 0.002, "s",
                   "EtherCAT DC cycle time; must match the master."),
    DriveParameter("max_joint_step_deg", "motion", "float", 0.0, "deg",
                   "Per-cycle shoulder step cap (0 = off) - singularity guard."),
    DriveParameter("position_tol_counts", "motion", "int", 500, "counts",
                   "End-of-move tolerance. 500 counts ~ 0.46 deg at the joint "
                   "(17-bit encoder, 3:1) - a realistic servo settling window; "
                   "tighten as the gain tuning improves."),
    DriveParameter("settle_timeout_s", "motion", "float", 2.0, "s",
                   "How long a move may take to settle into the tolerance "
                   "after the CSP stream ends (integral action needs time - "
                   "a longer wait often allows a TIGHTER tolerance)."),
    DriveParameter("velocity_ff_scale", "motion", "float", 1.0, "-",
                   "Scales the streamed 0x60B1 velocity feedforward. The drive "
                   "uses it only with speed-FF source = Communication "
                   "(C01.13 / speed_ff_source = 5); this gives feedforward "
                   "without the drive differentiating the position steps (no "
                   "chirp). 1.0 = counts/s; trim on the bench if the drive's "
                   "velocity units differ (FE minimises at the right scale). "
                   "0 = don't stream velocity FF."),
    # --- CiA 402 drive objects (written to both drives) ----------------------
    DriveParameter("homing_method", "drive", "int", 24, "-",
                   "0x6098 homing method (switch + index pulse).", sdo=(0x6098, 0), size=1),
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


def _dtype_size(dtype: str) -> int:
    return 1 if dtype == "int8" else 2 if dtype == "int16" else 4


# CoE SDO abort codes that mean the value's byte length didn't match the object:
#   0x06070012 = data type mismatch, length too high
#   0x06070013 = data type mismatch, length too low
_SIZE_ABORTS = ("06070012", "06070013")


def _sdo_write_adaptive(sdo_write, index: int, sub: int, value: int,
                        size: int, drive: int) -> int:
    """Write an SDO, retrying the other byte widths if the drive rejects it for
    length. Returns the size that succeeded. Non-length aborts (bad value, no
    such object) raise immediately — we only auto-recover from a wrong width."""
    order = [size] + [s for s in (2, 4, 1) if s != size]
    last: Exception | None = None
    for sz in order:
        try:
            sdo_write(index, sub, value, size=sz, drive=drive)
            return sz
        except Exception as exc:  # noqa: BLE001
            if any(a in str(exc) for a in _SIZE_ABORTS):
                last = exc
                continue
            raise
    raise last if last is not None else RuntimeError("SDO write failed")


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
    """A drive object (e.g. a gain), written over SDO on Apply. Preloaded tuning
    objects and user-added ones share this type."""

    name: str
    index: int
    sub: int
    value: float
    dtype: str = "int"
    desc: str = ""

    def coerce(self, v) -> float:
        return int(round(float(v))) if self.dtype == "int" else float(v)

    @property
    def address(self) -> str:
        return f"0x{self.index:04X}:{self.sub}"


# Preloaded tuning objects for the AS715N (A6-EC). Addresses are the EXPLICIT
# subindices from the ESI DataType definitions (DT2000/DT2001) in
# STEPPERONLINE_A6_Servo — the subindices are SPARSE (object 0x2000 has entries
# at :01,:02,:05,:06,:07,:08,:11...; :03 and :04 don't exist, which is why a
# write to 0x2000:04 aborts). All UINT (16-bit), rw, effective immediately.
#   (name, CoE address, default value, dtype, description)
DEFAULT_TUNING: List[Tuple[str, str, float, str, str]] = [
    ("load_inertia_ratio", "0x2000:07", 100,  "int16", "Load inertia ratio (%, 0-12000) — set this FIRST; gains scale off it"),
    ("auto_tuning_mode",   "0x2000:05", 1,     "int16", "Gain auto-tuning mode: 0=Manual, 1=Standard (by stiffness), 2=Positioning. Set 0 to hand-tune the gains"),
    ("stiffness_level",    "0x2000:06", 12,    "int16", "Stiffness level (1-31) — the main 'make it stiffer' dial in Standard mode; too high oscillates"),
    ("pos_loop_gain",      "0x2001:01", 400,   "int16", "1st position loop gain (0.1 rad/s, 0-20000) — raise to cut following error (Manual mode)"),
    ("speed_loop_gain",    "0x2001:02", 250,   "int16", "1st speed loop gain (0.1 Hz, 1-20000) — raise this before position gain"),
    ("speed_integ_time",   "0x2001:03", 3184,  "int16", "1st speed loop integral time (0.01 ms, 1-51200) — lower kills steady-state error"),
    ("torque_filter",      "0x2001:04", 200,   "int16", "1st torque ref filter time constant (Hz, 5-16000) — LOWER to damp high-freq buzz (more delay)"),
    # FF objects live in 0x2001 at DECIMAL subindices 20/21/23/24 (ESI DT2001);
    # the friendly Cxx.NN form maps there correctly (C01.13 -> 0x2001:20), unlike
    # a bare "0x2001:20" which the address parser would read as hex sub 0x20.
    ("speed_ff_source",    "C01.13",    1,     "int16", "Speed feedforward SELECT: 0=off, 1=internal ref, 2=model, 5=comms. 1 derives FF from the position-command slope — the CSP fix for velocity-proportional following error (Er.47/0x8611). Set with the drive stopped"),
    ("speed_ff_gain",      "C01.14",    500,   "int16", "Speed feedforward gain (0.1%, 0-2000) — 500 = 50%. Cancels following error at speed; climb toward 1000 (100%) watching for overshoot at move ends"),
    ("torque_ff_source",   "C01.16",    0,     "int16", "Torque feedforward SELECT: 0=off, 1=internal ref, 2=model. Enable (1) only if the ACCEL ramp still lags after speed FF"),
    ("torque_ff_gain",     "C01.17",    0,     "int16", "Torque feedforward gain (0.1%, 0-2000) — raise with care, too high overshoots"),
]

# Bump when DEFAULT_TUNING addresses/values change so a saved config re-seeds.
TUNING_SEED_VERSION = 5
# Names used by earlier (wrong-address) seed sets, dropped on migration.
_LEGACY_TUNING_NAMES = {"inertia_ratio", "machine_stiffness", "realtime_autotune",
                        "pos_loop_gain", "vel_loop_gain", "vel_integ_time", "torque_filter"}


def default_values() -> Dict[str, float]:
    return {p.name: p.default for p in PARAMETERS}


class ParameterStore:
    """Persisted parameter values; unknown/absent keys read as defaults."""

    def __init__(self, values: Optional[Dict[str, float]] = None,
                 path: Optional[str | Path] = None,
                 custom: Optional[List[CustomParameter]] = None,
                 seeded: bool = False) -> None:
        self.path: Optional[Path] = Path(path) if path else None
        merged = default_values()
        if values:
            merged.update({k: _BY_NAME[k].coerce(v) for k, v in values.items()
                           if k in _BY_NAME})
        self._values = merged
        self._custom: List[CustomParameter] = list(custom or [])
        self._seeded = seeded
        self._seed_version = 0
        # Names edited since the last successful write. Apply pushes ONLY these
        # to the drive — never the whole (partly-guessed) preloaded set.
        self._dirty: set = set()

    def _seed_default_tuning(self) -> None:
        """Preload the tuning objects (stiffness, inertia ratio, loop gains) so
        the tuning section isn't empty on a fresh install. Once seeded, the flag
        persists so removals aren't undone on the next load."""
        for name, addr, val, dtype, desc in DEFAULT_TUNING:
            if any(c.name == name for c in self._custom):
                continue
            self.add_custom(name, addr, val, dtype, desc)
        self._seeded = True
        self._seed_version = TUNING_SEED_VERSION
        self._dirty.clear()          # preloaded defaults are not "edited by the user"

    def _migrate_tuning(self) -> None:
        """Replace an outdated preloaded tuning set (wrong addresses/values from
        an earlier version) with the current one, preserving user-added params."""
        managed = _LEGACY_TUNING_NAMES | {n for n, *_ in DEFAULT_TUNING}
        self._custom = [c for c in self._custom if c.name not in managed]
        for name, addr, val, dtype, desc in DEFAULT_TUNING:
            self.add_custom(name, addr, val, dtype, desc)
        self._seeded = True
        self._seed_version = TUNING_SEED_VERSION
        self._dirty.clear()

    def dirty(self) -> List[str]:
        return sorted(self._dirty)

    def touch(self, name: str) -> None:
        """Force ``name`` to be treated as edited so the next ``apply`` writes it
        even when its value already equals the seeded default — e.g. re-asserting
        speed feedforward source = 1, whose default is already 1."""
        self._dirty.add(name)

    @classmethod
    def load(cls, path: str | Path) -> "ParameterStore":
        p = Path(path)
        if not p.exists():
            s = cls(path=p)
            s._seed_default_tuning()          # fresh install -> preload tuning objects
            return s
        if yaml is None:  # pragma: no cover
            raise RuntimeError("PyYAML is required to load drive parameters")
        data = yaml.safe_load(p.read_text()) or {}
        custom = [CustomParameter(name=str(c["name"]), index=int(c["index"]),
                                  sub=int(c["sub"]), value=c.get("value", 0),
                                  dtype=str(c.get("dtype", "int")),
                                  desc=str(c.get("desc", "")))
                  for c in data.get("custom", []) if "name" in c]
        seeded = bool(data.get("tuning_seeded", False)) if isinstance(data, dict) else False
        seed_ver = int(data.get("tuning_seed_version", 1 if seeded else 0)) if isinstance(data, dict) else 0
        s = cls(data.get("values", data), path=p, custom=custom, seeded=seeded)
        s._seed_version = seed_ver
        if not s._seeded:                     # legacy file / empty tuning -> seed once
            s._seed_default_tuning()
        elif seed_ver < TUNING_SEED_VERSION:  # outdated preloaded set -> correct it
            s._migrate_tuning()
        s._dirty.clear()                      # a freshly loaded store has no pending edits
        return s

    def as_dict(self) -> Dict[str, float]:
        return dict(self._values)

    def get(self, name: str) -> float:
        return self._values[name]

    def set(self, name: str, value) -> None:
        if name not in _BY_NAME:
            raise KeyError(f"unknown parameter {name!r}")
        new = _BY_NAME[name].coerce(value)
        if new != self._values.get(name):
            self._dirty.add(name)
        self._values[name] = new

    # --- custom (user-added) drive parameters ------------------------------- #
    def custom_parameters(self) -> List[CustomParameter]:
        return list(self._custom)

    def add_custom(self, name: str, address: str, value=0, dtype: str = "int",
                   desc: str = "") -> CustomParameter:
        """Add/replace a custom drive object addressed by Cxx.NN or INDEX:SUB."""
        index, sub = parse_drive_address(address)
        val = int(round(float(value))) if dtype == "int" else float(value)
        cp = CustomParameter(name=name.strip(), index=index, sub=sub, value=val,
                             dtype=dtype, desc=desc)
        if not cp.name:
            raise ValueError("parameter name is required")
        self._custom = [c for c in self._custom if c.name != cp.name] + [cp]
        self._dirty.add(cp.name)          # a user-added object is meant to be written
        return cp

    def set_custom_value(self, name: str, value) -> None:
        for i, c in enumerate(self._custom):
            if c.name == name:
                new = c.coerce(value)
                if new != c.value:
                    self._dirty.add(name)
                self._custom[i] = CustomParameter(c.name, c.index, c.sub,
                                                  new, c.dtype, c.desc)
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
            "tuning_seeded": self._seeded,
            "tuning_seed_version": self._seed_version,
            "values": self._values,
            "custom": [{"name": c.name, "index": c.index, "sub": c.sub,
                        "dtype": c.dtype, "value": c.value, "desc": c.desc}
                       for c in self._custom],
        }
        self.path.write_text(yaml.safe_dump(payload, sort_keys=True))
        return self.path

    # --- application --------------------------------------------------------
    def trajectory_limits(self):
        """Build TrajectoryLimits from the motion-scope values."""
        from .trajectory import TrajectoryLimits

        cap = self.get("max_joint_step_deg")
        jerk = self.get("jerk_mm_s3")
        return TrajectoryLimits(
            speed_mm_s=self.get("speed_mm_s"),
            accel_mm_s2=self.get("accel_mm_s2"),
            cycle_dt_s=self.get("cycle_dt_s"),
            jerk_mm_s3=None if jerk <= 0 else jerk,
            max_joint_step_deg=None if cap <= 0 else cap,
        )

    def _drive_items(self):
        """(name, index, sub, value, size) for every drive-scope object — the
        curated drive params and the custom/tuning ones."""
        items = [(p.name, p.sdo[0], p.sdo[1], int(self.get(p.name)), p.size)
                 for p in PARAMETERS if p.scope == "drive"]
        items += [(c.name, c.index, c.sub, int(c.value), _dtype_size(c.dtype))
                  for c in self._custom]
        return items

    def apply(self, driver, only_dirty: bool = True) -> List[str]:
        """Apply to a live EtherCatRobotDriver: motion limits rebuild in place;
        each EDITED drive-scope SDO is written to every drive (never the whole
        preloaded set — editing stiffness writes only stiffness). Writes are
        size-adaptive and verified; a verified write clears its dirty flag.
        Pass ``only_dirty=False`` to force-push every drive-scope object.
        Returns per-item messages plus a written/unchanged summary."""
        notes: List[str] = []
        driver.limits = self.trajectory_limits()
        driver.position_tol_counts = int(self.get("position_tol_counts"))
        driver.settle_timeout_s = float(self.get("settle_timeout_s"))
        driver.velocity_ff_scale = float(self.get("velocity_ff_scale"))
        notes.append("motion limits applied")
        sdo_write = getattr(driver.master, "sdo_write", None)
        sdo_read = getattr(driver.master, "sdo_read", None)
        drives = list(range(len(getattr(driver.master, "drives", []) or [])))
        if not callable(sdo_write) or not drives:
            notes.append("drive params: master has no SDO channel")
            return notes
        items = [it for it in self._drive_items()
                 if not only_dirty or it[0] in self._dirty]
        if not items:
            notes.append("no edited drive parameters to write")
            return notes
        written = unchanged = ignored = failed = 0
        for name, idx, sub, val, sz in items:
            ok = []
            landed = True
            for d in drives:
                current = None
                if callable(sdo_read):
                    try:
                        current = int(sdo_read(idx, sub, size=sz, drive=d))
                    except Exception:  # noqa: BLE001 - unreadable -> treat as changed
                        current = None
                if current == val:
                    unchanged += 1
                    continue
                try:
                    _sdo_write_adaptive(sdo_write, idx, sub, val, sz, d)
                except Exception as exc:  # noqa: BLE001
                    failed += 1
                    landed = False
                    notes.append(f"{name} drive {d} 0x{idx:04X}:{sub}: WRITE ABORTED - {exc}")
                    continue
                # Verify: the write returned OK, but did the value actually change?
                # A read-only monitor object or a state-gated / auto-tune-overridden
                # gain accepts the write and keeps its old value.
                after = None
                if callable(sdo_read):
                    try:
                        after = int(sdo_read(idx, sub, size=sz, drive=d))
                    except Exception:  # noqa: BLE001
                        after = None
                if after is not None and after != val:
                    ignored += 1
                    landed = False
                    notes.append(f"{name} drive {d} 0x{idx:04X}:{sub}: wrote {val} but "
                                 f"drive kept {after} (read-only or state-gated?)")
                else:
                    ok.append(str(d))
                    written += 1
            if ok:
                notes.append(f"{name} -> 0x{idx:04X}:{sub} = {val} (drives {', '.join(ok)})")
            if landed:
                self._dirty.discard(name)     # verified everywhere -> no longer pending
        summary = f"{written} written, {unchanged} unchanged"
        if ignored:
            summary += f", {ignored} ignored by drive"
        if failed:
            summary += f", {failed} aborted"
        notes.append(summary)
        return notes

    def read_custom_from_drives(self, driver) -> Dict[str, List[Optional[int]]]:
        """Read each custom/tuning object back from every drive over SDO. Returns
        {name: [drive0_value, drive1_value, ...]} with None where a read failed or
        the master can't read (e.g. the simulator before anything was written)."""
        sdo_read = getattr(driver.master, "sdo_read", None)
        n = len(getattr(driver.master, "drives", []) or [])
        out: Dict[str, List[Optional[int]]] = {}
        for c in self._custom:
            vals: List[Optional[int]] = []
            for d in range(n):
                if callable(sdo_read):
                    try:
                        vals.append(int(sdo_read(c.index, c.sub,
                                                 size=_dtype_size(c.dtype), drive=d)))
                    except Exception:  # noqa: BLE001
                        vals.append(None)
                else:
                    vals.append(None)
            out[c.name] = vals
        return out
