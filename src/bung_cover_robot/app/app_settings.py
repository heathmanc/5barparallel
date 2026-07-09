"""Small operator/site preference store (git-ignored).

Holds values that are specific to a machine/site and shouldn't live in the
tracked config — today the PLC IP/slot and the last Basler serial — so the HMI
can restore them across launches. A flat key/value YAML at
``config/app_settings.yaml``; missing file just means "no saved prefs yet".
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional


class AppSettings:
    def __init__(
        self, data: Optional[Dict[str, Any]] = None, path: Optional[str | Path] = None
    ) -> None:
        self._data: Dict[str, Any] = dict(data or {})
        self.path: Optional[Path] = Path(path) if path else None

    @classmethod
    def load(cls, path: str | Path) -> "AppSettings":
        """Load from YAML, remembering ``path`` so a later ``set`` persists. A
        missing/empty/malformed file yields an empty (but writable) store."""
        import yaml

        p = Path(path)
        if not p.exists():
            return cls(path=p)
        try:
            data = yaml.safe_load(p.read_text()) or {}
        except (OSError, yaml.YAMLError):
            data = {}
        return cls(data if isinstance(data, dict) else {}, path=p)

    def get(self, key: str, default: Any = None) -> Any:
        value = self._data.get(key, default)
        return value if value is not None else default

    def set(self, key: str, value: Any) -> None:
        """Set one key and persist immediately (best-effort)."""
        if value is None:
            self._data.pop(key, None)
        else:
            self._data[key] = value
        self.save()

    def save(self) -> Optional[Path]:
        if self.path is None:
            return None
        import yaml

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(yaml.safe_dump(self._data, sort_keys=True))
        return self.path
