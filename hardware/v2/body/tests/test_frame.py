import numpy as np
import pytest

from body import frame_v2 as F
from body.check import fits, grid_inside, overhang_patches
from body.interfaces import load


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    return F.build(tmp_path_factory.mktemp("v2"), step=0.5)


def _box_points(lo, hi, step=0.5):
    ax = [np.arange(lo[i], hi[i] + 1e-9, step) for i in range(3)]
    return np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3)


def test_servo_fits_each_pocket():
    frame = F.frame_field()
    for s in load().servos.values():
        assert (frame(_box_points(s.body_lo + 0.05, s.body_hi - 0.05)) > 0).all()


def test_holder_fits_bay():
    lo = np.array([-29.0, -15.5, -2.4 + 0.05])
    pts = _box_points(lo, -lo * [1, 1, 0] + [0, 0, 13.55])
    assert (F.frame_field()(pts) > 0).all() and (F.lid_field()(pts) > 0).all()


def test_cover_fits():
    report = fits(F.frame_field(), F.lid_field())
    assert report == [], report


def test_cover_screw_holes_open():
    frame = F.frame_field()
    for x, y in load().cover_screws:
        assert frame(np.array([[x, y, 22.0], [x, y, 21.2]])).min() > 0


def test_hip_sweep_is_empty():
    from body.sweep import keep_out
    pts = _box_points((-42, -36, -4), (42, 36, 23), 1.0)
    inside_sweep = keep_out()(pts) < -1.0
    assert (F.frame_field()(pts[inside_sweep]) > 0).all()


def test_board_too_big_is_reported():
    big = F.Params(charger=(40.0, 30.0, 8.0))
    msgs = fits(F.frame_field(big), F.lid_field(big), big)
    assert any("зарядка" in m and "мм" in m for m in msgs)


def test_speaker_grille_open():
    p = F.Params()
    c = np.array(p.speaker_center)
    line = np.array([c + [t, 0, 0] for t in np.arange(0, 6, 0.25)])
    assert (F.frame_field()(line) > 0).all()


def test_watertight_and_light(built):
    frame, lid = built["meshes"]
    assert frame.is_watertight and lid.is_watertight
    assert (frame.volume + lid.volume) / 1000 <= 53


def test_no_overhangs(built):
    frame, _ = built["meshes"]
    worst = overhang_patches(frame, bed_z=23.0)
    assert worst <= 20.0, f"нависание без опоры шириной {worst:.1f} мм"


def test_slim_keeps_shape():
    from body.sdf import box, mesh
    m = mesh(box((20, 20, 10), r=2), (-11, -11, -6), (11, 11, 6), 0.15)
    s = F.slim(m, 50000)                       # из 140 тыс.; плоский бокс сильнее не сжимается
    assert len(s.faces) <= 50000 and s.is_watertight
    assert abs(s.volume - m.volume) / m.volume < 0.005
