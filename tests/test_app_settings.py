"""AppSettings: the git-ignored operator/site preference store."""

from bung_cover_robot.app.app_settings import AppSettings


def test_roundtrip_persists(tmp_path):
    path = tmp_path / "app_settings.yaml"
    s = AppSettings.load(path)
    assert s.get("plc_ip", "") == ""          # nothing saved yet
    s.set("plc_ip", "192.168.1.10/0")
    assert path.exists()
    reloaded = AppSettings.load(path)
    assert reloaded.get("plc_ip") == "192.168.1.10/0"


def test_missing_file_is_writable(tmp_path):
    s = AppSettings.load(tmp_path / "nope.yaml")
    assert s.get("x", 5) == 5
    s.set("x", 1)
    assert (tmp_path / "nope.yaml").exists()


def test_set_none_removes_key(tmp_path):
    s = AppSettings.load(tmp_path / "a.yaml")
    s.set("k", "v")
    s.set("k", None)
    assert s.get("k", "fallback") == "fallback"


def test_malformed_file_yields_empty_store(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(": : not : yaml :")
    s = AppSettings.load(p)
    assert s.get("anything") is None
