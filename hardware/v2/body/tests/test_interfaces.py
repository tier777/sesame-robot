import numpy as np
import pytest

from body.interfaces import load


@pytest.fixture(scope="module")
def v1():
    return load()


def test_deck_top(v1):
    assert v1.deck_top_z == pytest.approx(23.0, abs=0.1)


def test_four_cover_screws(v1):
    s = np.array(v1.cover_screws)
    assert len(s) == 4 and abs(s[:, 1].sum()) < 0.5
    assert {tuple(np.round(p, 1)) for p in s} == {(-36.3, -16.8), (-36.3, 16.8), (33.3, -15.9), (33.3, 15.9)}


def test_flange_holes_27_8_apart(v1):
    for hip, s in v1.servos.items():
        a, b = np.array(s.flange_holes)
        assert np.linalg.norm(a[:2] - b[:2]) == pytest.approx(27.8, abs=0.2)
        assert abs(abs(a[1]) - 25.4) < 0.2


def test_servo_body_below_deck(v1):
    for s in v1.servos.values():
        lo, hi = s.body_lo, s.body_hi
        assert hi[2] <= 23.2 and lo[2] == pytest.approx(7.2, abs=0.3)
        w, l = sorted(hi[:2] - lo[:2])
        assert 12.0 < w < 13.0 and 22.5 < l < 24.5      # корпус MG90S 12,4×22,8 + выход провода в модели автора


def test_usb_window_at_back(v1):
    x, y0, y1, z0, z1 = v1.cover_usb_window
    assert x < -35 and 9 < y1 - y0 < 14 and 5 < z1 - z0 < 8 and z0 == pytest.approx(23, abs=1)


def test_volumes(v1):
    assert v1.frame_v1_volume_cm3 + v1.bottom_v1_volume_cm3 == pytest.approx(53.3, abs=0.5)
