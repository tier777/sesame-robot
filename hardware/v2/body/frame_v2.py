"""Рама Sesame v2 и крышка отсека: отсек 2×14500 внутри между сервами бёдер, открытые гнёзда серв,
пустая зона хода ног, окна облегчения по топооптимизации, места под зарядку, плату защиты, MPU-6500 и звук.
Оси двойника: X — к мордочке, Y — влево, Z — вверх; 0 по Z — низ нижней крышки v1. Мм.

Пересборка: python -m body.frame_v2  (STL в body/out/)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt, gaussian_filter, zoom

from body import sdf as S
from body.check import fits
from body.interfaces import load
from body.sweep import keep_out
from body.topopt import wall_density

OUT = Path(__file__).parent / "out"


@dataclass(frozen=True)
class Params:
    # отсек: держатель 58×31×16 с зазором 0,4
    holder: tuple = (58.0, 31.0, 16.0)
    holder_gap: float = 0.4
    bay_floor_z: float = -2.4          # низ держателя = верх крышки отсека
    lid_t: float = 1.6
    wall: float = 1.6
    deck_top: float = 23.0
    deck_t: float = 2.0
    fillet: float = 1.5
    servo_gap: float = 0.3
    servo_wall_bottom_z: float = 15.0  # стенки гнезда сервы — до середины корпуса, ниже открыто
    flange: tuple = (32.1, 12.4)       # фланец MG90S
    flange_bottom_z: float = 22.2      # как у v1: сервы на тех же высотах
    pilot: float = 1.7                 # саморез M2 в пластик
    clear: float = 2.3                 # проход M2
    keep_out_margin: float = 1.5
    window_threshold: float = 0.3
    # платы (до замера — с запасом)
    charger: tuple = (32.0, 22.0, 8.0)     # LX-USC-V2 (IP2326), USB-C назад в окно крышки
    bms: tuple = (46.0, 22.0, 5.0)         # плата защиты 2S, лежит на держателе в отсеке
    mpu: tuple = (25.0, 16.0, 3.0)
    amp: tuple = (19.0, 18.0, 3.0)         # MAX98357A
    speaker: tuple = (20.0, 4.5)           # ⌀ × толщина
    mic: tuple = (14.0, 3.0)               # INMP441, ⌀ × толщина
    speaker_center: tuple = (35.6, 0.0, 9.0)
    mic_center: tuple = (-37.6, 0.0, 9.0)
    lid_screws: tuple = ((0.0, 20.0), (0.0, -20.0))

    @property
    def bay_half(self) -> np.ndarray:      # внутренняя полуширина кармана отсека (x, y)
        return np.array([self.holder[0] / 2 + self.holder_gap, self.holder[1] / 2 + self.holder_gap])

    @property
    def bay_outer(self) -> np.ndarray:
        return self.bay_half + self.wall


DEFAULT = Params()


def _wall_windows(p: Params) -> S.Field:
    """Окна в длинных стенках отсека там, где топооптимизация оставила плотность ниже порога."""
    d = gaussian_filter(wall_density(), 1.0)
    length, height = d.shape[1], d.shape[0]
    up = 4                                                     # 0,25 мм
    fine = zoom(d, up, order=1)
    hole = fine < p.window_threshold
    # рамка по краю стенки остаётся сплошной: 2 мм сверху/снизу, 3 мм по торцам
    edge = int(2 * up)
    hole[:edge] = hole[-edge:] = False
    hole[:, :3 * up] = hole[:, -3 * up:] = False
    inside = distance_transform_edt(hole) / up
    outside = distance_transform_edt(~hole) / up
    signed = np.where(hole, -inside, outside)                  # минус — внутри окна
    x0, z0 = -length / 2, p.bay_floor_z
    step = 1 / up
    grid2 = S.from_grid(signed[None].transpose(2, 0, 1), (x0, 0, z0), step)  # оси (x, -, z)
    band = p.bay_outer[1]

    def f(q):
        qq = np.stack([q[:, 0], np.zeros(len(q)), q[:, 2]], 1)
        in_wall = np.abs(np.abs(q[:, 1]) - (band - p.wall / 2)) - p.wall
        return np.maximum(grid2(qq), in_wall)
    return f


def _servo_parts(p: Params):
    v1 = load()
    solids, cuts = [], []
    for s in v1.servos.values():
        lo, hi = s.body_lo - p.servo_gap, s.body_hi + p.servo_gap
        c = (lo + hi) / 2
        out = np.sign(c[1])
        # стенки вокруг корпуса сервы: снаружи и по торцам, от деки до середины высоты
        wl = np.array([lo[0] - p.wall, min(lo[1], hi[1]) - p.wall, p.servo_wall_bottom_z])
        wh = np.array([hi[0] + p.wall, max(lo[1], hi[1]) + p.wall, p.deck_top])
        solids.append(S.box(wh - wl, (wl + wh) / 2, r=0.8))
        cuts.append(S.box(hi - lo + [0, 0, 20], c + [0, 0, 10]))                       # корпус насквозь вверх
        fl = np.array([p.flange[0] + 2 * p.servo_gap, p.flange[1] + 2 * p.servo_gap, 20.0])
        cuts.append(S.box(fl, (c[0], c[1], p.flange_bottom_z + 10)))                     # фланец как у v1
        for h in s.flange_holes:
            cuts.append(S.cylinder(p.pilot / 2, 12, (h[0], h[1], p.deck_top - 6)))
        # внутренняя сторона (к отсеку) — без стенки, провод свободно уходит вниз и вбок
        cuts.append(S.box((hi[0] - lo[0], p.wall + 0.5, p.deck_top - p.servo_wall_bottom_z - p.deck_t),
                          (c[0], c[1] - out * ((hi[1] - lo[1]) / 2 + p.wall / 2),
                           (p.servo_wall_bottom_z + p.deck_top - p.deck_t) / 2)))
    return solids, cuts


def _deck(p: Params) -> S.Field:
    """Дека: полоса под крышку во всю длину и выносы под фланцы серв, углы скруглены."""
    z = (p.deck_top - p.deck_t / 2)
    v1 = load()
    lo, hi = v1.frame_mesh.bounds
    body = S.box((hi[0] - lo[0], 37.4, p.deck_t), ((lo[0] + hi[0]) / 2, 0, z), r=0.8)
    tabs = []
    for s in v1.servos.values():
        c = (s.body_lo + s.body_hi) / 2
        tabs.append(S.box((p.flange[0] + 4, 15.6, p.deck_t), (c[0], c[1], z), r=0.8))
    return S.union(body, *tabs)


def frame_field(p: Params = DEFAULT) -> S.Field:
    return _frame_cached(p)


@lru_cache(maxsize=4)
def _frame_cached(p: Params) -> S.Field:
    v1 = load()
    deck = _deck(p)
    bo, bi = p.bay_outer, p.bay_half
    zc = (p.bay_floor_z + p.deck_top - p.deck_t) / 2
    zh = p.deck_top - p.deck_t - p.bay_floor_z
    bay_walls = S.difference(S.box((2 * bo[0], 2 * bo[1], zh + 0.5), (0, 0, zc + 0.25), r=0.8),
                             S.box((2 * bi[0], 2 * bi[1], zh + 2), (0, 0, zc)))
    # стенки под звук: передняя с решёткой динамика и задняя с отверстием микрофона, от низа до деки
    fx, rx = v1.frame_mesh.bounds[1][0], v1.frame_mesh.bounds[0][0]
    front = S.box((p.wall, 2 * bo[1], zh + 0.5), (fx - p.wall / 2 - 0.3, 0, zc + 0.25), r=0.6)
    rear = S.box((p.wall, 2 * bo[1], zh + 0.5), (rx + p.wall / 2 + 0.3, 0, zc + 0.25), r=0.6)
    sides = [S.box((fx - rx - 0.6, p.wall, zh + 0.5), (0.5 * (fx + rx), s * (bo[1] - p.wall / 2), zc + 0.25), r=0.6)
             for s in (1, -1)]
    bosses = [S.cylinder(2.6, zh + 0.5, (x, y, zc + 0.25)) for x, y in p.lid_screws]
    servo_solids, servo_cuts = _servo_parts(p)
    shell = S.smooth_union(deck, S.union(bay_walls, front, rear, *sides, *bosses, *servo_solids), p.fillet)

    cuts = list(servo_cuts)
    cuts.append(S.box((2 * bi[0], 2 * bi[1], zh + 2), (0, 0, zc)))                        # карман отсека
    cuts.append(_wall_windows(p))
    for x, y in v1.cover_screws:                                                           # саморезы крышки снизу
        cuts.append(S.cylinder(p.clear / 2, 10, (x, y, p.deck_top - 2)))
    for x, y in p.lid_screws:
        cuts.append(S.cylinder(p.pilot / 2, 12, (x, y, p.bay_floor_z + 5.5)))
    for y in (8.0, -8.0):                                                                  # провода из отсека на деку
        cuts.append(S.cylinder(3.0, 6, (-24.0, y, p.deck_top - 1)))
    # динамик: гнездо в передней части, решётка в стенке; открыто снизу, держит крышка отсека
    sc = np.array(p.speaker_center)
    cuts.append(S.cylinder(p.speaker[0] / 2 + 0.3, p.speaker[1] + 0.4, sc - [p.speaker[1] / 2 + 0.6, 0, 0], axis="x"))
    cuts.append(S.box((p.speaker[1] + 0.4, p.speaker[0] + 0.6, 30), sc - [p.speaker[1] / 2 + 0.6, 0, 15]))
    for gy in np.arange(-6, 6.1, 3.0):
        for gz in np.arange(-6, 6.1, 3.0):
            if gy * gy + gz * gz <= 49:
                cuts.append(S.cylinder(0.75, 6, sc + [3, gy, gz], axis="x"))
    # микрофон: гнездо у задней стенки и отверстие напротив порта
    mc = np.array(p.mic_center)
    cuts.append(S.cylinder(p.mic[0] / 2 + 0.3, p.mic[1] + 0.4, mc, axis="x"))
    cuts.append(S.box((p.mic[1] + 0.4, p.mic[0] + 0.6, 30), mc - [0, 0, 15]))
    cuts.append(S.cylinder(1.0, 6, mc - [3, 0, 0], axis="x"))
    # прорези под стяжки на деке: зарядка у задней стенки, MPU-6500 по центру, усилитель и макетка спереди
    w = v1.cover_usb_window
    cy = (w[1] + w[2]) / 2
    for sx in (-30.0, -14.0, 0.0, 10.0, 24.0):
        for sy in (cy + p.charger[1] / 2 + 1.5, cy - p.charger[1] / 2 - 1.5):
            cuts.append(S.box((3.2, 1.6, 6), (sx, sy, p.deck_top - 1)))
    frame = S.difference(shell, *cuts, keep_out(margin=p.keep_out_margin))
    return frame


def lid_field(p: Params = DEFAULT) -> S.Field:
    v1 = load()
    fx, rx = v1.frame_mesh.bounds[1][0], v1.frame_mesh.bounds[0][0]
    z = p.bay_floor_z - p.lid_t / 2
    plate = S.box((fx - rx - 1.0, 2 * p.bay_outer[1], p.lid_t), ((fx + rx) / 2, 0, z), r=0.8)
    ears = [S.cylinder(3.0, p.lid_t, (x, y, z)) for x, y in p.lid_screws]
    holes = [S.cylinder(p.clear / 2, 4, (x, y, z)) for x, y in p.lid_screws]
    return S.difference(S.union(plate, *ears), *holes)


def build(out_dir: Path = OUT, step: float = 0.15, p: Params = DEFAULT) -> dict:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    problems = fits(frame_field(p), lid_field(p), p)
    if problems:
        raise ValueError("рама v2 не собирается:\n- " + "\n- ".join(problems))
    v1 = load()
    lo, hi = v1.frame_mesh.bounds
    bottom = p.bay_floor_z - p.lid_t - 2
    frame = S.mesh(frame_field(p), [lo[0] - 3, lo[1] - 3, bottom], hi + [3, 3, 3], step)
    lid = S.mesh(lid_field(p), [lo[0] - 3, lo[1] - 3, bottom], [hi[0] + 3, hi[1] + 3, p.bay_floor_z + 2], step)
    for name, m in (("frame_v2", frame), ("bay_lid_v2", lid)):
        if not m.is_watertight:
            raise ValueError(f"{name}: сетка не замкнута")
        m.export(out_dir / f"{name}.stl")
    return {"frame": out_dir / "frame_v2.stl", "lid": out_dir / "bay_lid_v2.stl", "meshes": (frame, lid)}


if __name__ == "__main__":
    r = build()
    f, l = r["meshes"]
    print(f"рама v2: {f.volume / 1000:.1f} см³, крышка отсека: {l.volume / 1000:.1f} см³ → {r['frame'].parent}")
