import json

import numpy as np

from twin import params as P
from twin.simulate import foot_slip_mm, load, run, save


def test_stand_holds():
    log = run({"events": [], "duration_ms": 5000})
    assert log.status == "ok"
    assert log.torso_tilt_deg.max() < 2
    assert foot_slip_mm(log) < 1


def test_targets_follow_trace():
    log = run({"events": [[0, 0, 90]], "duration_ms": 500})
    assert log.target_fw[-1][0] == 90
    assert abs(log.q_fw[-1][0] - 90) < 3


def test_speed_limit():
    log = run({"events": [[0, 5, 0]], "duration_ms": 1000})
    rate = np.abs(np.diff(log.target_fw[:, 5]) / np.diff(log.t))
    assert rate.max() <= P.SERVO_SPEED_DPS + 1e-6


def test_deadband_ignores_tiny_moves():
    log = run({"events": [[0, 0, 135 + 0.5]], "duration_ms": 200})
    assert log.target_fw[-1][0] == 135


def test_absurd_pose_is_flagged_not_crashing():
    ev = [[0, c, 0] for c in range(8)] + [[20, c, 180] for c in range(8)]
    log = run({"events": ev, "duration_ms": 2000})
    assert log.status in ("ok", "fell", "unstable")
    assert np.isfinite(log.com).all()


def test_save_load_roundtrip(tmp_path):
    log = run({"events": [], "duration_ms": 100})
    save(log, tmp_path / "x.pkl")
    again = load(tmp_path / "x.pkl")
    assert np.allclose(again.q_fw, log.q_fw) and again.status == log.status


def test_joint_speed_is_servo_limited():
    # MG90S: 60° за 0,1 с; момент падает со скоростью, поэтому сустав не разгоняется заметно быстрее
    log = run({"events": [[0, 5, 0]], "duration_ms": 800})
    speed = np.abs(np.diff(log.q_fw[:, 5]) / np.diff(log.t))
    assert speed.max() < P.SERVO_SPEED_DPS * 1.1


def test_blowup_detected():
    import mujoco
    from twin import model as M
    from twin.simulate import blew_up
    m, d = M.load()
    M.set_pose(m, d, P.STAND_FW)
    assert not blew_up(d)
    d.qvel[0] = np.nan
    mujoco.mj_step(m, d)
    assert blew_up(d)
