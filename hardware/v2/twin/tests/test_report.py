import numpy as np
import pytest

from twin.report import analyze, current_a, render_video, support_margin_mm, write
from twin.simulate import Log, run


def fake_log(n=500, dt=0.002, stall_channel=None, stall_s=0.0):
    torque = np.zeros((n, 8))
    if stall_channel is not None:
        torque[100:100 + int(round(stall_s / dt)), stall_channel] = 0.2
    sq = np.array([[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]])
    return Log(t=np.arange(n) * dt, target_fw=np.full((n, 8), 90.0), q_fw=np.full((n, 8), 90.0),
               torque=torque, com=np.zeros((n, 3)), torso_pos=np.zeros((n, 3)), torso_tilt_deg=np.zeros(n),
               foot_tips=np.zeros((n, 4, 3)), foot_contacts=[sq] * n, contacts=[[]] * n,
               qpos=np.zeros((n, 15)))


def test_current_model():
    assert np.allclose(current_a(np.array([0.0, 0.1, 0.2])), [0.1, 0.43, 0.76])


def test_margin_inside_square():
    assert support_margin_mm([0, 0], [[-10, -10], [10, -10], [10, 10], [-10, 10]]) == pytest.approx(10)


def test_margin_outside_square_is_negative():
    assert support_margin_mm([15, 0], [[-10, -10], [10, -10], [10, 10], [-10, 10]]) == pytest.approx(-5)


def test_margin_two_feet():
    assert support_margin_mm([0, 5], [[-10, 0], [10, 0]]) == pytest.approx(-5)


def test_stall_detected():
    s = analyze(fake_log(stall_channel=3, stall_s=0.3))
    assert s.servos[3].stall_s == pytest.approx(0.3, abs=0.01)
    assert s.servos[0].stall_s == 0


def test_idle_current():
    s = analyze(fake_log())
    assert s.current_peak_a == pytest.approx(0.8) and s.current_mean_a == pytest.approx(0.8)


def test_collisions_collapse_runs():
    log = fake_log(n=10)
    log.contacts = [[]] * 2 + [[("foot_R3", "torso", 1.0)]] * 5 + [[]] * 3
    s = analyze(log)
    assert [(c[0], c[1]) for c in s.collisions] == [("foot_R3", "torso")]
    assert s.collisions[0][2] == pytest.approx(0.004)


def test_report_lists_failed(tmp_path):
    out = write({}, {"wave": "use of undeclared identifier 'runMoonwalk'"}, tmp_path)
    assert "wave" in out.read_text() and "runMoonwalk" in out.read_text()


def test_video_written(tmp_path):
    log = run({"events": [[0, 0, 100]], "duration_ms": 300})
    render_video(log, tmp_path / "v.mp4")
    assert (tmp_path / "v.mp4").stat().st_size > 1000


def test_out_of_safe_range_listed():
    log = fake_log(n=10)
    log.target_fw[5:, 1] = 0           # R2 в 0°
    s = analyze(log, safe={"R2": (17, 94)})
    assert s.out_of_range == [("R2", 0, (17, 94))]


def test_margin_typical_and_share_outside():
    log = fake_log(n=10)
    log.com[:2, :2] = [0.05, 0]        # 2 кадра из 10 центр масс в 5 см снаружи опоры
    s = analyze(log)
    assert s.margin_min_mm == pytest.approx(-40)
    assert s.margin_typical_mm == pytest.approx(10)
    assert s.outside_share == pytest.approx(0.2)
