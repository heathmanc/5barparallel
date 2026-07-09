"""Commissioning-constant registry, push/read, persistence, and drift guard."""

import importlib.util
from pathlib import Path

import pytest

from bung_cover_robot.plc import (
    COMMISSIONING_CONSTANTS,
    PlcConstantStore,
    SimulatedPlcClient,
    default_values,
    push_constants,
    read_constants,
)
from bung_cover_robot.plc.compactlogix_client import PlcClient, PlcError

REPO = Path(__file__).resolve().parents[1]


# --------------------------------------------------------------------------- #
# registry
# --------------------------------------------------------------------------- #
def test_registry_is_well_formed():
    names = [c.name for c in COMMISSIONING_CONSTANTS]
    assert len(names) == len(set(names))            # unique
    assert all(c.dtype in ("DINT", "REAL") for c in COMMISSIONING_CONSTANTS)
    assert default_values()["HOME_ANGLE_L"] == pytest.approx(140.5406)


def test_coerce_matches_type():
    by = {c.name: c for c in COMMISSIONING_CONSTANTS}
    assert by["HOME_OFFSET_L"].coerce(3748.6) == 3749      # DINT rounds
    assert isinstance(by["HOME_OFFSET_L"].coerce(10.0), int)
    assert by["HOME_ANGLE_L"].coerce(140) == pytest.approx(140.0)
    assert isinstance(by["HOME_ANGLE_L"].coerce(140), float)


# --------------------------------------------------------------------------- #
# push / read against the simulator
# --------------------------------------------------------------------------- #
def test_push_then_read_round_trips():
    client = SimulatedPlcClient().connect()
    values = default_values()
    values["HOME_OFFSET_L"] = 3748
    values["HOME_OFFSET_R"] = 1052
    results = push_constants(client, values)
    assert all(r.ok for r in results)
    assert len(results) == len(COMMISSIONING_CONSTANTS)

    back = read_constants(client)
    assert back["HOME_OFFSET_L"] == 3748
    assert back["HOME_OFFSET_R"] == 1052
    assert back["HOME_ANGLE_L"] == pytest.approx(140.5406)


def test_push_fills_missing_with_defaults():
    client = SimulatedPlcClient().connect()
    # only supply one value; the rest must still be written from defaults
    push_constants(client, {"VAC_SETTLE": 500})
    back = read_constants(client)
    assert back["VAC_SETTLE"] == 500
    assert back["BLOWOFF_TIME"] == 200          # default pushed


class _FlakyClient(PlcClient):
    """Writes everything except one tag, which raises — exercises partial push."""

    def __init__(self, bad):
        self._bad = bad
        self._store = {}

    def connect(self):
        return self

    def close(self):
        pass

    @property
    def is_connected(self):
        return True

    def read(self, tag):
        return self._store.get(tag, 0)

    def write(self, tag, value):
        if tag == self._bad:
            raise PlcError("simulated write failure")
        self._store[tag] = value


def test_push_reports_per_tag_failure_without_aborting():
    client = _FlakyClient(bad="HOME_ACC")
    results = push_constants(client, default_values())
    by = {r.name: r for r in results}
    assert not by["HOME_ACC"].ok and "failure" in by["HOME_ACC"].error
    # every other tag still got written
    assert sum(1 for r in results if r.ok) == len(COMMISSIONING_CONSTANTS) - 1
    assert client._store["MOVE_VEL"] == 20000


def test_read_skips_disconnected_tags_gracefully():
    client = SimulatedPlcClient()  # not connected -> read raises
    assert read_constants(client) == {}


# --------------------------------------------------------------------------- #
# persistence
# --------------------------------------------------------------------------- #
def test_store_missing_file_uses_defaults(tmp_path):
    store = PlcConstantStore.load(tmp_path / "none.yaml")
    assert store.as_dict() == default_values()


def test_store_roundtrip(tmp_path):
    path = tmp_path / "plc_constants.yaml"
    store = PlcConstantStore.load(path)
    store.update({"HOME_OFFSET_L": 3748, "HOME_OFFSET_R": 1052})
    store.save()
    assert path.exists()

    reloaded = PlcConstantStore.load(path)
    assert reloaded.get("HOME_OFFSET_L") == 3748
    assert reloaded.get("HOME_OFFSET_R") == 1052
    # untouched values fall back to defaults
    assert reloaded.get("MOVE_VEL") == 20000


def test_store_rejects_unknown_tag(tmp_path):
    store = PlcConstantStore.load(tmp_path / "x.yaml")
    with pytest.raises(KeyError):
        store.set("NOT_A_TAG", 1)


# --------------------------------------------------------------------------- #
# drift guard — registry must mirror the L5X generator's set-by-hand tags
# --------------------------------------------------------------------------- #
def _load_generator():
    spec = importlib.util.spec_from_file_location(
        "_gen", REPO / "scripts" / "render_plc_l5x.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_registry_mirrors_generator_hand_tags():
    gen = _load_generator()
    hand = {
        t.name: t for t in gen._glue_tags()
        if t.set_by_hand and not t.constant and t.dtype in ("DINT", "REAL")
    }
    for c in COMMISSIONING_CONSTANTS:
        assert c.name in hand, f"{c.name} missing from generator"
        tag = hand[c.name]
        assert c.dtype == tag.dtype, f"{c.name} dtype drift"
        assert c.default == pytest.approx(float(tag.value)), f"{c.name} default drift"
    # and no set-by-hand writable scalar is silently absent from the registry
    assert set(hand) == {c.name for c in COMMISSIONING_CONSTANTS}
