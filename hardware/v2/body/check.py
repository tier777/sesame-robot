"""Проверки рамы v2: верхняя крышка надевается, платы входят в свои места, нависания для печати."""
from __future__ import annotations

from functools import lru_cache

import numpy as np
import trimesh
from scipy.ndimage import distance_transform_edt, label

from body import sdf as S
from body.interfaces import load

COVER_STEP = 0.5


@lru_cache(maxsize=1)
def cover_field() -> S.Field:
    """Поле крышки v117 по вокселам: минус — внутри пластика."""
    cover = load().cover_mesh
    vox = cover.voxelized(COVER_STEP).fill()
    lo = cover.bounds[0] - 2
    shape = np.ceil((cover.bounds[1] + 2 - lo) / COVER_STEP).astype(int) + 1
    occ = np.zeros(shape, bool)
    idx = np.round((vox.points - lo) / COVER_STEP).astype(int)
    occ[tuple(idx.T)] = True
    signed = distance_transform_edt(~occ) * COVER_STEP - distance_transform_edt(occ) * COVER_STEP
    return S.from_grid(signed, lo, COVER_STEP)


def grid_inside(lo, hi, step: float) -> np.ndarray:
    ax = [np.arange(lo[i], hi[i] + 1e-9, step) for i in range(3)]
    return np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3)


def _board(name: str, size, center, frame: S.Field, msgs: list) -> None:
    half = np.array(size) / 2
    pts = grid_inside(np.array(center) - half + 0.05, np.array(center) + half - 0.05, 0.5)
    depth = max(0.0, -min(frame(pts).min(), cover_field()(pts).min() + COVER_STEP))
    if depth > 0.1:
        msgs.append(f"{name} {size[0]:g}×{size[1]:g}×{size[2]:g} не входит в своё место: упирается на {depth:.1f} мм, "
                    f"нужно ещё {depth:.1f} мм или плата меньше")


def fits(frame: S.Field, lid: S.Field, p=None) -> list[str]:
    from body.frame_v2 import DEFAULT
    p = p or DEFAULT
    v1 = load()
    msgs = []
    # верхняя крышка: пластик крышки не должен заходить в раму
    cover_pts = v1.cover_mesh.voxelized(COVER_STEP).fill().points
    clash = (frame(cover_pts) < -0.3).sum() * COVER_STEP ** 3
    if clash > 50:
        msgs.append(f"верхняя крышка v117 врезается в раму на {clash / 1000:.2f} см³")
    # зарядка на деке у задней стенки, USB-C напротив окна крышки
    w = v1.cover_usb_window
    cx = w[0] + 2.0 + p.charger[0] / 2          # задняя стенка крышки 1,9 мм, плата вплотную, USB-C в окне
    _board("зарядка", p.charger, (cx, (w[1] + w[2]) / 2, p.deck_top + p.charger[2] / 2), frame, msgs)
    # плата защиты лежит на держателе в отсеке
    _board("плата защиты", p.bms, (0, 0, p.bay_floor_z + p.holder[2] + p.bms[2] / 2 + 0.3), frame, msgs)
    # на деке под крышкой: усилитель спереди, MPU-6500 под DevKit (DevKit стоит на штырях ~8,5 мм)
    _board("усилитель", p.amp, (20.0, 5.0, p.deck_top + p.amp[2] / 2 + 0.2), frame, msgs)
    _board("MPU-6500", p.mpu, (0.0, 0.0, p.deck_top + p.mpu[2] / 2 + 0.2), frame, msgs)
    if p.mpu[2] > 7.5:
        msgs.append(f"MPU-6500 высотой {p.mpu[2]:g} мм не пролезет под DevKit на штырях: нужно не выше 7,5 мм")
    # динамик и микрофон стоят торцом между отсеком и стенкой: диаметр ограничен высотой и шириной этого места
    room_h = 2 * min(p.speaker_z - p.bay_floor_z, p.deck_top - p.deck_t - p.speaker_z)
    room_w = 2 * p.bay_outer[1]
    for name, dia in (("динамик", p.speaker[0]), ("микрофон", p.mic[0])):
        need = dia + 0.6 - min(room_h, room_w)
        if need > 0:
            msgs.append(f"{name} ⌀{dia:g} не входит в своё место: нужно ещё {need:.1f} мм или поменьше")
    return msgs


def overhang_patches(mesh: trimesh.Trimesh, bed_z: float) -> float:
    """Печать декой вниз: грани, смотрящие в работе вверх (ниже стола), в печати смотрят вниз.
    Возвращает наибольшую ширину такого участка (меньшую сторону габарита), мм — это длина моста."""
    up = (mesh.face_normals[:, 2] > np.cos(np.radians(45))) & (mesh.triangles_center[:, 2] < bed_z - 0.1)
    if not up.any():
        return 0.0
    faces = np.nonzero(up)[0]
    adj = mesh.face_adjacency
    keep = np.isin(adj[:, 0], faces) & np.isin(adj[:, 1], faces)
    import scipy.sparse as sp
    n = len(mesh.faces)
    g = sp.coo_matrix((np.ones(keep.sum()), (adj[keep, 0], adj[keep, 1])), shape=(n, n))
    _, labels = sp.csgraph.connected_components(g, directed=False)
    worst = 0.0
    for lab in np.unique(labels[faces]):
        tri = mesh.triangles[faces[labels[faces] == lab]].reshape(-1, 3)
        ext = tri[:, :2].max(axis=0) - tri[:, :2].min(axis=0)
        worst = max(worst, float(ext.min()))
    return worst
