import json

from conftest import FIRMWARE, FIXTURES
from twin.firmware_trace.trace import POSES, build_traces


def test_stand_is_exact(tmp_path):
    r = build_traces(FIRMWARE, tmp_path)
    assert r.traces["stand"] == [(0, 0, 135), (20, 1, 45), (40, 2, 45), (60, 3, 135),
                                 (80, 4, 0), (100, 5, 180), (120, 6, 0), (140, 7, 180)]


def test_wave_duration(tmp_path):
    # runWavePose: 29 setServoAngle (stand 8 + 4 + 1 + 4×2 + stand 8) × 20 мс
    # + delayWithFace 200 + 200 + 300 + 4×(300+300) = 580 + 3100
    build_traces(FIRMWARE, tmp_path)
    assert json.loads((tmp_path / "wave.json").read_text())["duration_ms"] == 3680


def test_all_poses_traced(tmp_path):
    r = build_traces(FIRMWARE, tmp_path)
    assert set(r.traces) | set(r.failed) == set(POSES)
    assert not r.failed
    assert len(POSES) == 19


def test_walk_cycles(tmp_path):
    r = build_traces(FIRMWARE, tmp_path, walk_cycles=3)
    first_step = [e for e in r.traces["walk"] if e[1] == 0 and e[2] == 170]
    assert len(first_step) == 3
    assert [e[1:] for e in r.traces["walk"][-8:]] == [(0, 135), (1, 45), (2, 45), (3, 135),
                                                       (4, 0), (5, 180), (6, 0), (7, 180)]


def test_bad_pose_does_not_stop_others(tmp_path):
    r = build_traces(FIXTURES / "bad_firmware", tmp_path)
    assert "wave" in r.failed and "runMoonwalk" in r.failed["wave"]
    assert "stand" in r.traces
