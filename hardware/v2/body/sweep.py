"""Зона хода ног: объём, который бёдра и лапы заметают во всех позах прошивки, — рама там должна быть пустой."""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from scipy.ndimage import distance_transform_edt

from body.sdf import Field, from_grid
from twin import params as P
from twin.calibrate import _link_meshes

TRACES = P.HERE / "traces"
REGION = (np.array([-46.0, -46.0, -8.0]), np.array([46.0, 46.0, 36.0]))   # где вообще может быть рама
SAMPLES = 4000
FOOT_OF = {v: k for k, v in P.PARENT_HIP.items()}


def _states(traces_dir: Path):
    """Все наборы из 8 углов, которые даёт прошивка: после каждого события каждой позы, начиная со stand."""
    files = sorted(Path(traces_dir).glob("*.json"))
    if not files:
        raise FileNotFoundError(f"нет записей поз в {traces_dir}: запусти bash hardware/v2/tools/twin.sh")
    for f in files:
        state = list(P.STAND_FW)
        yield list(state)
        for _, ch, angle in json.loads(f.read_text())["events"]:
            state[ch] = angle
            yield list(state)


def commanded_ranges(traces_dir: Path = TRACES) -> dict[str, tuple[int, int]]:
    lo, hi = {j: 180 for j in P.HIPS}, {j: 0 for j in P.HIPS}
    for s in _states(traces_dir):
        for j in P.HIPS:
            a = s[P.CHANNEL[j]]
            lo[j], hi[j] = min(lo[j], a), max(hi[j], a)
    return {j: (lo[j], hi[j]) for j in P.HIPS}


@lru_cache(maxsize=None)
def leg_points(hip: str) -> tuple[np.ndarray, np.ndarray]:
    """Точки поверхности бедра и лапы этой ноги в позе сборки (stand), мм."""
    out = []
    for link in (f"femur_{hip}", f"foot_{FOOT_OF[hip]}"):
        mesh = trimesh.util.concatenate(_link_meshes(link))
        pts, _ = trimesh.sample.sample_surface_even(mesh, SAMPLES, seed=2)
        out.append(np.vstack([pts, mesh.vertices[:: max(1, len(mesh.vertices) // SAMPLES)]]))
    return out[0], out[1]


def _rot(pts: np.ndarray, anchor, axis, ang: float) -> np.ndarray:
    a, k = np.asarray(anchor), np.asarray(axis, float)
    p = pts - a
    c, s = np.cos(ang), np.sin(ang)
    return p * c + np.cross(k, p) * s + np.outer(p @ k, k) * (1 - c) + a


def _pairs(hip: str, traces_dir: Path, max_step: float = 4.0) -> set[tuple[int, int]]:
    """Пары (угол бедра, угол лапы) по всем позам, переходы между соседними состояниями — с шагом не больше 4°."""
    foot = FOOT_OF[hip]
    pairs, prev = set(), None
    for s in _states(traces_dir):
        cur = (s[P.CHANNEL[hip]], s[P.CHANNEL[foot]])
        if prev is not None and prev != cur:
            n = int(np.ceil(max(abs(cur[0] - prev[0]), abs(cur[1] - prev[1])) / max_step))
            for t in np.linspace(0, 1, n + 1):
                pairs.add((round(prev[0] + t * (cur[0] - prev[0])), round(prev[1] + t * (cur[1] - prev[1]))))
        pairs.add(cur)
        prev = cur
    return pairs


@lru_cache(maxsize=4)
def keep_out(step: float = 0.5, margin: float = 1.5, traces_dir: Path = TRACES) -> Field:
    """Поле: минус — внутри зоны хода ног (с запасом margin), плюс — свободно для рамы."""
    g = P.geometry()["joints"]
    lo, hi = REGION
    shape = np.ceil((hi - lo) / step).astype(int) + 1
    occ = np.zeros(shape, bool)
    for hip in P.HIPS:
        foot = FOOT_OF[hip]
        femur, foot_pts = leg_points(hip)
        for h, f in _pairs(hip, traces_dir):
            fp = _rot(foot_pts, g[foot]["anchor_mm"], g[foot]["axis"], P.fw_to_rad(foot, f))
            pts = _rot(np.vstack([femur, fp]), g[hip]["anchor_mm"], g[hip]["axis"], P.fw_to_rad(hip, h))
            idx = np.round((pts - lo) / step).astype(int)
            ok = ((idx >= 0) & (idx < shape)).all(axis=1)
            occ[tuple(idx[ok].T)] = True
    dist = distance_transform_edt(~occ) * step
    return from_grid(dist - margin, lo, step)
