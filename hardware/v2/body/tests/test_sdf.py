import numpy as np
import pytest

from body.sdf import box, cylinder, difference, from_grid, mesh, slab_z, smooth_union, union


def test_box_volume():
    m = mesh(box((10, 20, 30)), (-6, -11, -16), (6, 11, 16), 0.25)
    assert m.is_watertight and abs(m.volume - 6000) < 60


def test_hole_diameter():
    plate = difference(box((10, 10, 2)), cylinder(1.0, 4))
    m = mesh(plate, (-6, -6, -2), (6, 6, 2), 0.1)
    assert m.volume == pytest.approx(200 - 2 * np.pi, abs=1.0)     # дыра ⌀2,0 ± 0,1 мм


def test_smooth_union_fills_corner():
    a, b = box((10, 2, 10), center=(0, -4, 0)), box((2, 10, 10), center=(-4, 0, 0))
    p = np.array([[-2.8, -2.8, 0.0]])          # в углу, чуть вне обеих плит
    assert union(a, b)(p)[0] > 0 and smooth_union(a, b, 1.5)(p)[0] < 0


def test_touching_edge_raises():
    with pytest.raises(ValueError, match="края"):
        mesh(box((10, 10, 10)), (-5, -5, -5), (5, 5, 5), 0.5)


def test_from_grid_matches_source():
    ball = lambda p: np.linalg.norm(p, axis=1) - 3.0
    step = 0.5
    ax = np.arange(-5, 5 + step / 2, step)
    g = np.stack(np.meshgrid(ax, ax, ax, indexing="ij"), -1).reshape(-1, 3)
    f = from_grid(ball(g).reshape(len(ax), len(ax), len(ax)), (-5, -5, -5), step)
    pts = np.random.default_rng(1).uniform(-4, 4, (500, 3))
    assert np.abs(f(pts) - ball(pts)).max() < step / 2


def test_slab_z():
    assert slab_z(1, 2)(np.array([[0, 0, 1.5]]))[0] < 0 and slab_z(1, 2)(np.array([[0, 0, 3]]))[0] > 0
