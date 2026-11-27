"""Что рама v2 обязана сохранить от v1: высота деки, саморезы верхней крышки, сервы бёдер, окно USB в крышке."""
from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from twin import geometry as G
from twin import params as P

OUT = Path(__file__).parent / "out"
FRAME, COVER, BOTTOM = "Internal-Frame:1", "=>[0:1:1:108]", "Bottom-Cover:1"
HIP_SERVO = {"R1": "v4:2", "R2": "v4:3", "L1": "v4:1", "L2": "v4:4"}


@dataclass
class Servo:
    body_lo: np.ndarray        # габарит корпуса сервы ниже деки (без фланцев), мм
    body_hi: np.ndarray
    flange_holes: list         # две точки (x, y, z) осей отверстий фланца
    case: trimesh.Trimesh      # вся серва (для проверок посадки)


@dataclass
class V1:
    deck_top_z: float
    cover_screws: list
    cover_usb_window: tuple    # (x, y0, y1, z0, z1)
    servos: dict
    frame_v1_volume_cm3: float
    bottom_v1_volume_cm3: float
    cover_mesh: trimesh.Trimesh
    frame_mesh: trimesh.Trimesh


def _frame():
    g = P.geometry()
    return np.array(g["frame"]["cad_to_twin"]), np.array(g["frame"]["origin_mm"])


def _vertical_holes(part, rot, origin) -> list[tuple[float, float, float, float]]:
    """Вертикальные цилиндрические грани радиусом < 3 мм: (x, y, z середины, радиус) в координатах двойника."""
    out = []
    ex = TopExp_Explorer(part.shape, TopAbs_FACE)
    while ex.More():
        s = BRepAdaptor_Surface(TopoDS.Face(ex.Current()))
        if s.GetType() == GeomAbs_Cylinder and s.Cylinder().Radius() < 3:
            ax = s.Cylinder().Axis()
            d = np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()]) @ rot.T
            if abs(d[2]) > 0.99:
                loc = np.array([ax.Location().X(), ax.Location().Y(), ax.Location().Z()])
                q = s.Value((s.FirstUParameter() + s.LastUParameter()) / 2, (s.FirstVParameter() + s.LastVParameter()) / 2)
                mid = (np.array([q.X(), q.Y(), q.Z()]) - origin) @ rot.T
                xy = ((loc - origin) @ rot.T)[:2]
                out.append((float(xy[0]), float(xy[1]), float(mid[2]), s.Cylinder().Radius()))
        ex.Next()
    return out


def _usb_window(cover: trimesh.Trimesh, pitch: float = 0.5) -> tuple:
    """Окно внизу задней стенки крышки: столбцы (y, z) в задних 3 мм крышки, где нет пластика."""
    vox = cover.voxelized(pitch).fill()
    pts = vox.points
    back = cover.bounds[0][0]
    wall = pts[pts[:, 0] < back + 3]
    occupied = {(round(y / pitch), round(z / pitch)) for y, z in wall[:, 1:]}
    ys = np.arange(-8, 8 + pitch, pitch)
    zs = np.arange(cover.bounds[0][2], cover.bounds[0][2] + 10, pitch)
    holes = [(y, z) for y in ys for z in zs if (round(y / pitch), round(z / pitch)) not in occupied]
    if not holes:
        raise ValueError("в задней стенке крышки не найдено окно USB")
    h = np.array(holes)
    return (float(back), float(h[:, 0].min()), float(h[:, 0].max() + pitch), float(h[:, 1].min()), float(h[:, 1].max() + pitch))


@lru_cache(maxsize=1)
def load() -> V1:
    rot, origin = _frame()
    parts = {p.name: p for p in G.read_parts(P.STEP)}
    twin = {n: G._to_twin(p.mesh, rot, origin) for n, p in parts.items()}
    frame, cover, bottom = twin[FRAME], twin[COVER], twin[BOTTOM]

    holes = _vertical_holes(parts[COVER], rot, origin)
    screws = sorted({(round(x, 2), round(y, 2)) for x, y, _, r in holes if 0.8 < r < 1.0})

    frame_holes = [h for h in _vertical_holes(parts[FRAME], rot, origin) if 0.8 < h[3] < 0.95 and abs(abs(h[1]) - 25.4) < 0.3]
    servos = {}
    for hip, key in HIP_SERVO.items():
        name = next(n for n in parts if n.endswith(key) and "SG90" in n)
        case = twin[name]
        below = case.vertices[case.vertices[:, 2] < 22.5]
        lo, hi = below.min(axis=0), below.max(axis=0)
        # фланцы выше 22,5 — корпус сервы ниже деки уже без них
        c = (lo + hi) / 2
        near = sorted(frame_holes, key=lambda h: np.hypot(h[0] - c[0], h[1] - c[1]))[:2]
        servos[hip] = Servo(lo, hi, [np.array(h[:3]) for h in near], case)

    v1 = V1(deck_top_z=float(frame.bounds[1][2]), cover_screws=[np.array(s) for s in screws],
            cover_usb_window=_usb_window(cover), servos=servos,
            frame_v1_volume_cm3=float(frame.volume / 1000), bottom_v1_volume_cm3=float(bottom.volume / 1000),
            cover_mesh=cover, frame_mesh=frame)
    OUT.mkdir(exist_ok=True)
    (OUT / "interfaces.json").write_text(json.dumps({
        "deck_top_z": v1.deck_top_z, "cover_screws": [s.tolist() for s in v1.cover_screws],
        "cover_usb_window": v1.cover_usb_window,
        "servos": {h: {"body_lo": s.body_lo.tolist(), "body_hi": s.body_hi.tolist(),
                       "flange_holes": [p.tolist() for p in s.flange_holes]} for h, s in servos.items()},
        "frame_v1_volume_cm3": v1.frame_v1_volume_cm3, "bottom_v1_volume_cm3": v1.bottom_v1_volume_cm3}, indent=1))
    return v1
