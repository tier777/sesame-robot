"""2D-оптимизация топологии (SIMP, плоское напряжённое состояние, элементы 1×1 мм) для боковой стенки отсека."""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve

OUT = Path(__file__).parent / "out"


def _ke(nu: float = 0.3) -> np.ndarray:
    k = np.array([1 / 2 - nu / 6, 1 / 8 + nu / 8, -1 / 4 - nu / 12, -1 / 8 + 3 * nu / 8,
                  -1 / 4 + nu / 12, -1 / 8 - nu / 8, nu / 6, 1 / 8 - 3 * nu / 8])
    i = [[0, 1, 2, 3, 4, 5, 6, 7], [1, 0, 7, 6, 5, 4, 3, 2], [2, 7, 0, 5, 6, 3, 4, 1], [3, 6, 5, 0, 7, 2, 1, 4],
         [4, 5, 6, 7, 0, 1, 2, 3], [5, 4, 3, 2, 1, 0, 7, 6], [6, 3, 4, 1, 2, 7, 0, 5], [7, 2, 1, 4, 3, 6, 5, 0]]
    return 1 / (1 - nu ** 2) * k[np.array(i)]


def _edof(nx: int, nz: int) -> np.ndarray:
    """Степени свободы элементов; узел (ix, iz) → n = ix·(nz+1) + iz; dof 2n — x, 2n+1 — z."""
    out = np.zeros((nx * nz, 8), int)
    for ex in range(nx):
        for ez in range(nz):
            n1, n2 = ex * (nz + 1) + ez, (ex + 1) * (nz + 1) + ez
            out[ex * nz + ez] = [2 * n1 + 2, 2 * n1 + 3, 2 * n2 + 2, 2 * n2 + 3, 2 * n2, 2 * n2 + 1, 2 * n1, 2 * n1 + 1]
    return out


def _solve(x: np.ndarray, loads, supports, penal: float):
    nz, nx = x.shape
    ndof = 2 * (nx + 1) * (nz + 1)
    ke, edof = _ke(), _edof(nx, nz)
    xe = x.T.ravel()                                       # порядок элементов ex·nz + ez
    e = 1e-9 + xe ** penal * (1 - 1e-9)
    rows, cols = np.kron(edof, np.ones((8, 1), int)).ravel(), np.kron(edof, np.ones((1, 8), int)).ravel()
    k = coo_matrix(((ke.ravel()[None] * e[:, None]).ravel(), (rows, cols)), shape=(ndof, ndof)).tocsc()
    fixed = np.array([d for ix, iz in supports for d in (2 * (ix * (nz + 1) + iz), 2 * (ix * (nz + 1) + iz) + 1)])
    free = np.setdiff1d(np.arange(ndof), fixed)
    us = []
    for case in loads:
        f = np.zeros(ndof)
        for ix, iz, fx, fz in case:
            n = ix * (nz + 1) + iz
            f[2 * n] += fx
            f[2 * n + 1] += fz
        u = np.zeros(ndof)
        u[free] = spsolve(k[free][:, free], f[free])
        us.append(u)
    return us, ke, edof, xe


def compliance(x: np.ndarray, loads, supports, penal: float = 3.0) -> float:
    us, ke, edof, xe = _solve(x, loads, supports, penal)
    return float(sum(((1e-9 + xe ** penal) * np.einsum("ij,jk,ik->i", u[edof], ke, u[edof])).sum() for u in us))


def optimize(nx: int, nz: int, volfrac: float = 0.4, penal: float = 3.0, rmin: float = 1.5,
             loads=(), supports=(), iters: int = 80) -> np.ndarray:
    """Плотности nz×nx (строка — высота снизу вверх, столбец — длина). Несколько случаев нагрузки — сумма податливостей."""
    x = np.full((nz, nx), volfrac)
    # фильтр чувствительности (веса по расстоянию до rmin)
    r = int(np.ceil(rmin))
    offsets = [(dz, dx, rmin - np.hypot(dz, dx)) for dz in range(-r, r + 1) for dx in range(-r, r + 1) if np.hypot(dz, dx) < rmin]
    for _ in range(iters):
        us, ke, edof, xe = _solve(x, loads, supports, penal)
        dc = np.zeros(nx * nz)
        for u in us:
            ce = np.einsum("ij,jk,ik->i", u[edof], ke, u[edof])
            dc -= penal * xe ** (penal - 1) * ce
        dc = dc.reshape(nx, nz).T
        num, den = np.zeros_like(x), np.zeros_like(x)
        for dz, dx, w in offsets:
            sx = np.roll(np.roll(x * dc, dz, 0), dx, 1)
            sw = np.roll(np.roll(x, dz, 0), dx, 1)
            valid = np.ones_like(x, bool)
            if dz > 0: valid[:dz] = False
            if dz < 0: valid[dz:] = False
            if dx > 0: valid[:, :dx] = False
            if dx < 0: valid[:, dx:] = False
            num += np.where(valid, w * sx, 0)
            den += np.where(valid, w, 0)
        dc = num / np.maximum(1e-3, x * den)
        lo, hi, move = 0.0, 1e9, 0.2
        while hi - lo > 1e-4 * (lo + hi + 1e-12):
            mid = (lo + hi) / 2
            xn = np.clip(x * np.sqrt(np.maximum(-dc, 0) / mid), np.maximum(0.001, x - move), np.minimum(1.0, x + move))
            lo, hi = (mid, hi) if xn.mean() > volfrac else (lo, mid)
        x = xn
    return x


@lru_cache(maxsize=1)
def wall_density(length: int = 62, height: int = 23, iters: int = 60) -> np.ndarray:
    """Боковая стенка отсека в виде сбоку: длина — отсек с торцами, высота — от крышки отсека до деки.
    Опоры — верхний край над фланцами передней и задней серв бёдер (x 5…33 мм от середины в обе стороны);
    случай 1 — 7,7 Н вниз (0,26 кг × 3 g) по низу, случай 2 — удар 5 Н вверх в середину низа."""
    mid = length / 2
    supports = [(ix, height) for ix in range(length + 1) if 5 <= abs(ix - mid) <= 33]
    down = [(ix, 0, 0.0, -7.7 / (length + 1)) for ix in range(length + 1)]
    up = [(int(mid) + d, 0, 0.0, 5.0 / 3) for d in (-1, 0, 1)]
    d = optimize(length, height, volfrac=0.45, loads=[down, up], supports=supports, iters=iters)
    d = (d + d[:, ::-1]) / 2
    OUT.mkdir(exist_ok=True)
    np.save(OUT / "topopt_wall.npy", d)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.imshow(d, origin="lower", cmap="Greys", vmin=0, vmax=1, extent=(-mid, mid, -2.4, 21))
    ax.set_title("Стенка отсека: чёрное — где работает материал")
    ax.set_xlabel("x, мм"); ax.set_ylabel("z, мм")
    fig.tight_layout(); fig.savefig(OUT / "topopt_wall.png", dpi=120); plt.close(fig)
    return d
