import numpy as np

from body.topopt import compliance, optimize, wall_density

# консоль 40×16: левый край заделан, сила вниз в середине правого края
LOADS = [[(40, 8, 0.0, -1.0)]]
SUPPORTS = [(0, iz) for iz in range(17)]


def test_cantilever_compliance_drops():
    d = optimize(40, 16, volfrac=0.4, loads=LOADS, supports=SUPPORTS, iters=40)
    uniform = np.full_like(d, 0.4)
    assert compliance(d, LOADS, SUPPORTS) < 0.7 * compliance(uniform, LOADS, SUPPORTS)


def test_volume_fraction_kept():
    d = optimize(40, 16, volfrac=0.4, loads=LOADS, supports=SUPPORTS, iters=20)
    assert abs(d.mean() - 0.4) < 0.02


def test_wall_symmetric():
    d = wall_density()
    assert d.shape[1] > d.shape[0]
    assert np.abs(d - d[:, ::-1]).max() < 0.05
