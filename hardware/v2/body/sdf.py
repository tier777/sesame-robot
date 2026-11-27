"""Маленькая SDF-библиотека: поле расстояний (минус — внутри) на numpy и сетка через marching cubes."""
from __future__ import annotations

from typing import Callable

import numpy as np
import trimesh
from scipy.ndimage import map_coordinates
from skimage.measure import marching_cubes

Field = Callable[[np.ndarray], np.ndarray]


def _v(x) -> np.ndarray:
    return np.asarray(x, float)


def box(size, center=(0, 0, 0), r: float = 0.0) -> Field:
    """Прямоугольник со скруглёнными рёбрами радиуса r (размер — внешний)."""
    half, c = _v(size) / 2 - r, _v(center)

    def f(p):
        q = np.abs(p - c) - half
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0) - r
    return f


def cylinder(r: float, h: float, center=(0, 0, 0), axis: str = "z") -> Field:
    i = "xyz".index(axis)
    others = [k for k in range(3) if k != i]
    c = _v(center)

    def f(p):
        d = p - c
        radial = np.linalg.norm(d[:, others], axis=1) - r
        along = np.abs(d[:, i]) - h / 2
        q = np.stack([radial, along], 1)
        return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
    return f


def capsule(a, b, r: float) -> Field:
    a, b = _v(a), _v(b)
    ab = b - a

    def f(p):
        t = np.clip((p - a) @ ab / (ab @ ab), 0, 1)
        return np.linalg.norm(p - (a + t[:, None] * ab), axis=1) - r
    return f


def slab_z(z0: float, z1: float) -> Field:
    return lambda p: np.abs(p[:, 2] - (z0 + z1) / 2) - (z1 - z0) / 2


def union(*fs: Field) -> Field:
    return lambda p: np.min([f(p) for f in fs], axis=0)


def intersection(*fs: Field) -> Field:
    return lambda p: np.max([f(p) for f in fs], axis=0)


def difference(a: Field, *bs: Field) -> Field:
    return lambda p: np.max([a(p)] + [-b(p) for b in bs], axis=0)


def smooth_union(a: Field, b: Field, k: float) -> Field:
    """Плавное объединение: в стыке появляется скругление радиуса порядка k."""
    def f(p):
        da, db = a(p), b(p)
        h = np.clip(0.5 + 0.5 * (db - da) / k, 0, 1)
        return da * h + db * (1 - h) - k * h * (1 - h)
    return f


def translate(f: Field, v) -> Field:
    v = _v(v)
    return lambda p: f(p - v)


def from_grid(values: np.ndarray, origin, step: float) -> Field:
    """Поле, заданное значениями на регулярной сетке; между узлами — трилинейно, снаружи — краевое значение."""
    origin = _v(origin)

    def f(p):
        idx = ((p - origin) / step).T
        return map_coordinates(values, idx, order=1, mode="nearest")
    return f


def mesh(f: Field, lo, hi, step: float, chunk: int = 64_000_000) -> trimesh.Trimesh:
    lo, hi = _v(lo), _v(hi)
    axes = [np.arange(lo[i], hi[i] + step / 2, step) for i in range(3)]
    nx, ny, nz = map(len, axes)
    vol = np.empty((nx, ny, nz), np.float32)
    gx, gy = np.meshgrid(axes[0], axes[1], indexing="ij")
    per = max(1, chunk // (nx * ny))
    for k0 in range(0, nz, per):
        zs = axes[2][k0:k0 + per]
        pts = np.stack([np.repeat(gx[..., None], len(zs), 2), np.repeat(gy[..., None], len(zs), 2),
                        np.broadcast_to(zs, (nx, ny, len(zs)))], -1).reshape(-1, 3)
        vol[:, :, k0:k0 + len(zs)] = f(pts).reshape(nx, ny, len(zs))
    border = np.concatenate([vol[0].ravel(), vol[-1].ravel(), vol[:, 0].ravel(), vol[:, -1].ravel(),
                             vol[:, :, 0].ravel(), vol[:, :, -1].ravel()])
    if (border <= 0).any():
        raise ValueError("тело касается края сетки: расширь границы lo/hi")
    # уровень чуть внутри: грани, совпавшие с узлами сетки, иначе дают неразделимые рёбра
    verts, faces, _, _ = marching_cubes(vol, level=-1e-4 * step, spacing=(step, step, step))
    m = trimesh.Trimesh(verts + lo, faces[:, ::-1], process=False)
    m.merge_vertices()
    if m.volume < 0:
        m.invert()
    return m
