import json

from body.gate import new_stops, over_range


def _trace(tmp, name, events):
    (tmp / f"{name}.json").write_text(json.dumps({"events": events, "duration_ms": 100}))


def test_over_range(tmp_path):
    _trace(tmp_path, "bow", [[0, 1, 0]])                # R2 → 0
    assert over_range(tmp_path, {"R2": (17, 94)}) == {("bow", "R2", 0)}


def test_new_stops_only(tmp_path):
    _trace(tmp_path, "bow", [[0, 1, 0]])
    _trace(tmp_path, "point", [[0, 1, 100]])
    v1 = {"R2": (17, 94)}
    v2 = {"R2": (0, 93)}
    assert new_stops(tmp_path, v1, v2) == set()
    assert new_stops(tmp_path, {"R2": (0, 94)}, {"R2": (5, 93)}) == {("bow", "R2", 0)}
