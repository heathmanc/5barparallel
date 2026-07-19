"""Unit tests for the EtherCAT error-counter tool (scripts/ec_crc.py).

The bus I/O is subprocess-based (needs real hardware); these cover the pure
counter math + rendering and the no-CLI guard, which is what could silently
misreport a bad-cable diagnosis."""

import importlib.util
from pathlib import Path

import pytest

_SPEC = importlib.util.spec_from_file_location(
    "ec_crc", Path(__file__).resolve().parents[1] / "scripts" / "ec_crc.py")
ec_crc = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(ec_crc)


def _counters(rx0=0, inv0=0, fwd0=0, lost0=0, pu=0, pdi=0, rx1=0):
    return {"ports": [{"rx_error": rx0, "invalid_frame": inv0,
                       "forwarded": fwd0, "lost_link": lost0},
                      {"rx_error": rx1, "invalid_frame": 0,
                       "forwarded": 0, "lost_link": 0}],
            "pu_error": pu, "pdi_error": pdi}


def test_split_rx_high_low_bytes():
    # 0x0300 uint16: high byte = RX error, low byte = invalid frame.
    assert ec_crc.split_rx(0x1203) == {"rx_error": 0x12, "invalid_frame": 0x03}
    assert ec_crc.split_rx(0x0000) == {"rx_error": 0, "invalid_frame": 0}
    assert ec_crc.split_rx(0xFFFF) == {"rx_error": 255, "invalid_frame": 255}


def test_total_errors_sums_every_field():
    assert ec_crc.total_errors(_counters()) == 0
    assert ec_crc.total_errors(
        _counters(rx0=2, inv0=1, fwd0=3, lost0=4, pu=5, pdi=6, rx1=7)) == 28


def test_diff_counters_is_field_wise_delta():
    base = _counters(rx0=10, inv0=5, pu=1)
    now = _counters(rx0=13, inv0=5, pu=4)
    d = ec_crc.diff_counters(now, base)
    assert d["ports"][0]["rx_error"] == 3
    assert d["ports"][0]["invalid_frame"] == 0
    assert d["pu_error"] == 3
    assert ec_crc.total_errors(d) == 6


def test_render_clean_vs_errors():
    clean = ec_crc.render([_counters()])
    assert "CLEAN" in clean and "ERRORS" not in clean

    dirty = ec_crc.render([_counters(rx0=4)])
    assert "physical layer" in dirty
    assert "<-- ERRORS" in dirty          # the slave with errors is flagged


def test_render_delta_mode_labels_baseline():
    base = [_counters(rx0=100)]
    now = [_counters(rx0=112)]
    out = ec_crc.render(now, base)
    assert "delta since baseline" in out
    assert "12 error(s)" in out           # only the growth, not the 100 baseline


def test_no_cli_returns_guard_code(monkeypatch, capsys):
    monkeypatch.setattr(ec_crc.shutil, "which", lambda _name: None)
    assert ec_crc.main([]) == 2
    assert "ethercat` CLI is not on PATH" in capsys.readouterr().err
