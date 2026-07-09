"""Battery recipes (Claude.md §13).

A *recipe* is a battery type. Because the vent holes and the loose covers share
one plane that a changeover shifts, each recipe owns its own pixel->robot
calibration — stored per-recipe at ``calibration/<key>.npy`` by
``vision.calibration.CalibrationManager``. A recipe also carries the few
process values that vary by battery type (vent-hole count, cover diameter).

Selecting a recipe at changeover loads that recipe's calibration and detection
parameters for the whole cycle.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

_KEY_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")


class RecipeError(Exception):
    """Missing recipe or invalid recipe key."""


def slugify_key(text: str) -> str:
    """Turn free text into a filename-safe recipe key (calibration filename)."""
    slug = re.sub(r"[^A-Za-z0-9_-]+", "-", text.strip()).strip("-")
    return slug.lower()


@dataclass(frozen=True)
class Recipe:
    key: str                       # filename-safe id; also the calibration key
    name: str                      # human-readable label
    hole_count: int = 6            # expected vent holes (feeds the hole detector)
    cover_diameter_mm: float = 0.0  # nominal cover (bung) size (0 = size gate off)
    # A shouldered bung cover is wider than the hole it seats in, so the pick target
    # (cover) and the drop target (hole) are gated on separate diameters.
    hole_diameter_mm: float = 0.0   # nominal drop-hole size (0 = size gate off)
    # Physical-size gate half-width: a detected cover/hole is accepted only if its
    # real diameter is nominal * (1 +/- diameter_tolerance). 0.2 = +/-20%.
    diameter_tolerance: float = 0.2
    # Live detection tuning (the Vision-tab sliders) — saved per recipe so a
    # changeover restores the pixel-Ø windows + Hough knobs dialed in for that
    # battery. Covers use Hough (min/max Ø px, edge sensitivity, votes); drop
    # holes have their own (smaller) Ø window.
    cover_min_px: float = 250.0
    cover_max_px: float = 400.0
    hough_edge: int = 70            # 0..100 edge sensitivity (maps to Hough param1)
    hough_votes: int = 30           # Hough accumulator threshold (param2)
    hole_min_px: float = 30.0
    hole_max_px: float = 220.0

    def __post_init__(self) -> None:
        if not _KEY_RE.match(self.key):
            raise RecipeError(
                f"invalid recipe key {self.key!r} (use letters, digits, - and _)"
            )
        if self.hole_count < 1:
            raise RecipeError("hole_count must be >= 1")
        if self.cover_diameter_mm < 0:
            raise RecipeError("cover_diameter_mm must be >= 0")
        if self.hole_diameter_mm < 0:
            raise RecipeError("hole_diameter_mm must be >= 0")
        if not 0.0 < self.diameter_tolerance <= 1.0:
            raise RecipeError("diameter_tolerance must be in (0, 1]")
        for field_name in ("cover_min_px", "cover_max_px", "hole_min_px", "hole_max_px"):
            if getattr(self, field_name) <= 0:
                raise RecipeError(f"{field_name} must be > 0")

    @classmethod
    def from_dict(cls, data: dict) -> "Recipe":
        return cls(
            key=str(data["key"]),
            name=str(data.get("name", data["key"])),
            hole_count=int(data.get("hole_count", 6)),
            cover_diameter_mm=float(data.get("cover_diameter_mm", 0.0)),
            hole_diameter_mm=float(data.get("hole_diameter_mm", 0.0)),
            diameter_tolerance=float(data.get("diameter_tolerance", 0.2)),
            cover_min_px=float(data.get("cover_min_px", 250.0)),
            cover_max_px=float(data.get("cover_max_px", 400.0)),
            hough_edge=int(data.get("hough_edge", 70)),
            hough_votes=int(data.get("hough_votes", 30)),
            hole_min_px=float(data.get("hole_min_px", 30.0)),
            hole_max_px=float(data.get("hole_max_px", 220.0)),
        )

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "name": self.name,
            "hole_count": self.hole_count,
            "cover_diameter_mm": self.cover_diameter_mm,
            "hole_diameter_mm": self.hole_diameter_mm,
            "diameter_tolerance": self.diameter_tolerance,
            "cover_min_px": self.cover_min_px,
            "cover_max_px": self.cover_max_px,
            "hough_edge": self.hough_edge,
            "hough_votes": self.hough_votes,
            "hole_min_px": self.hole_min_px,
            "hole_max_px": self.hole_max_px,
        }


_DEFAULT_RECIPES = (
    Recipe("g31-6", "Group 31 (6-vent)", hole_count=6, cover_diameter_mm=18.0),
    Recipe("g24-6", "Group 24 (6-vent)", hole_count=6, cover_diameter_mm=18.0),
)


class RecipeStore:
    """The set of known recipes, backed by ``config/recipes.yaml``."""

    def __init__(
        self, recipes: Optional[List[Recipe]] = None, path: Optional[str | Path] = None
    ) -> None:
        self.path: Optional[Path] = Path(path) if path else None
        source = recipes if recipes is not None else list(_DEFAULT_RECIPES)
        self._recipes: Dict[str, Recipe] = {r.key: r for r in source}

    # --- loading ------------------------------------------------------------
    @classmethod
    def load(cls, path: str | Path) -> "RecipeStore":
        """Load from YAML; falls back to the built-in defaults if the file is
        missing or empty (still remembering ``path`` so new recipes persist)."""
        import yaml

        p = Path(path)
        if not p.exists():
            return cls(path=p)
        data = yaml.safe_load(p.read_text()) or {}
        recipes = [Recipe.from_dict(d) for d in data.get("recipes", [])]
        return cls(recipes or None, path=p)

    def save(self) -> Optional[Path]:
        if self.path is None:
            return None
        import yaml

        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(
            yaml.safe_dump(
                {"recipes": [r.to_dict() for r in self.list()]}, sort_keys=False
            )
        )
        return self.path

    # --- access -------------------------------------------------------------
    def list(self) -> List[Recipe]:
        return list(self._recipes.values())

    def keys(self) -> List[str]:
        return list(self._recipes.keys())

    def has(self, key: str) -> bool:
        return key in self._recipes

    def get(self, key: str) -> Recipe:
        if key not in self._recipes:
            raise RecipeError(f"no recipe {key!r}")
        return self._recipes[key]

    def add(self, recipe: Recipe) -> Recipe:
        """Add (or replace) a recipe and persist the store."""
        self._recipes[recipe.key] = recipe
        self.save()
        return recipe
