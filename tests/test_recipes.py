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
