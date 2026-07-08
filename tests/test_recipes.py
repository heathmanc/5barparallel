"""Recipe model + store (per battery type; each owns its calibration)."""

import pytest

from bung_cover_robot.app.recipes import (
    Recipe,
    RecipeError,
    RecipeStore,
    slugify_key,
)


def test_recipe_validates_key():
    with pytest.raises(RecipeError):
        Recipe(key="bad key!", name="x")
    with pytest.raises(RecipeError):
        Recipe(key="ok", name="x", hole_count=0)
    r = Recipe(key="g31-6", name="Group 31")
    assert r.hole_count == 6


def test_recipe_diameter_tolerance_default_and_validation():
    r = Recipe(key="g31-6", name="Group 31")
    assert r.cover_diameter_mm == 0.0
    assert r.diameter_tolerance == pytest.approx(0.2)   # tighter, sane default
    with pytest.raises(RecipeError):                      # out of (0, 1]
        Recipe(key="ok", name="x", diameter_tolerance=0.0)
    with pytest.raises(RecipeError):
        Recipe(key="ok", name="x", diameter_tolerance=1.5)
    with pytest.raises(RecipeError):
        Recipe(key="ok", name="x", cover_diameter_mm=-1.0)


def test_recipe_params_roundtrip():
    r = Recipe(
        key="g65-8", name="Group 65", hole_count=8,
        cover_diameter_mm=22.5, diameter_tolerance=0.15,
    )
    d = r.to_dict()
    assert d["diameter_tolerance"] == pytest.approx(0.15)
    assert Recipe.from_dict(d) == r


def test_slugify_key():
    assert slugify_key("Group 65 8-vent") == "group-65-8-vent"
    assert slugify_key("  A/B  ") == "a-b"


def test_store_defaults_and_lookup():
    store = RecipeStore()
    assert {r.key for r in store.list()} == {"g31-6", "g24-6"}
    assert store.has("g31-6")
    assert store.get("g31-6").hole_count == 6
    with pytest.raises(RecipeError):
        store.get("nope")


def test_store_add_persists_and_reloads(tmp_path):
    path = tmp_path / "recipes.yaml"
    store = RecipeStore(path=path)
    store.add(Recipe(key="g65-8", name="Group 65", hole_count=8))
    assert path.exists()

    reloaded = RecipeStore.load(path)
    assert reloaded.has("g65-8")
    assert reloaded.get("g65-8").hole_count == 8
    # the seed recipes are preserved alongside the added one
    assert reloaded.has("g31-6")


def test_store_load_missing_file_uses_defaults(tmp_path):
    store = RecipeStore.load(tmp_path / "does_not_exist.yaml")
    assert store.has("g31-6")
    # path is remembered so a later add() persists
    store.add(Recipe(key="new-1", name="New"))
    assert (tmp_path / "does_not_exist.yaml").exists()
