"""Калибровочный помощник: таблица каналов, безопасные диапазоны серв по геометрии, переодевание качалки.

  python -m twin.calibrate                 — out/calibration.md и картинки out/calib/<сустав>_<угол>.png
  python -m twin.calibrate R1 --stop 155   — на сколько зубцов переодеть качалку и какой servoSubtrim поставить
"""
from __future__ import annotations

import argparse
import json
from functools import lru_cache
from pathlib import Path
from typing import NamedTuple

import imageio.v2 as imageio
import mujoco
import numpy as np

from twin import model as M
from twin import params as P
from twin.simulate import INO


class HornFix(NamedTuple):
    teeth: int       # на сколько зубцов переодеть: минус — в сторону меньших углов
    subtrim: int     # остаток в servoSubtrim


def horn_fix(computed_stop: int, measured_stop: int, teeth: int = P.SPLINE_TEETH) -> HornFix:
    step = 360 / teeth
    offset = measured_stop - computed_stop
    k = int(round(offset / step))
    return HornFix(k, int(round(offset - k * step)))


def channel_table(ino: Path) -> list[tuple[int, str, int]]:
    pins = P.parse_pins(ino)
    return [(ch, j, pins[ch]) for ch, j in enumerate(P.JOINTS)]


PITCH_MM = 0.5
SAMPLES = 8000
AXIS_KEEP_OUT_MM = 5.0


def _link_meshes(link: str):
    """Детали звена по отдельности (замкнутые сетки из STEP) в координатах двойника, мм."""
    import trimesh
    from twin import geometry as G
    g = P.geometry()
    rot, origin = np.array(g["frame"]["cad_to_twin"]), np.array(g["frame"]["origin_mm"])
    names = set(g["links"][link]["parts"])
    meshes = [G._to_twin(p.mesh, rot, origin) for p in G.read_parts(P.STEP) if p.name in names]
    # детали не из STEP (рама v2 и т. п.) лежат готовыми STL в координатах двойника
    meshes += [trimesh.load(P.OUT / f) for f in g["links"][link].get("extra_parts", {}).values()]
    return meshes


@lru_cache(maxsize=None)
def _occupancy(link: str):
    """Занятые вокселы звена (с шагом 0,5 мм), поверхность снята на один воксел: касание — не упор."""
    from scipy.ndimage import binary_erosion
    meshes = _link_meshes(link)
    lo = np.min([m.bounds[0] for m in meshes], axis=0) - 2
    hi = np.max([m.bounds[1] for m in meshes], axis=0) + 2
    shape = np.ceil((hi - lo) / PITCH_MM).astype(int) + 1
    grid = np.zeros(shape, bool)
    for m in meshes:
        v = m.voxelized(PITCH_MM).fill()
        idx = np.round((v.points - lo) / PITCH_MM).astype(int)
        grid[tuple(idx.T)] = True
    return lo, binary_erosion(grid)


@lru_cache(maxsize=None)
def _moving_points(joint: str) -> np.ndarray:
    """Точки поверхности всего, что поворачивает сустав (для бедра — бедро и лапа), в позе сборки, мм."""
    import trimesh
    links = [("femur_" if joint in P.HIPS else "foot_") + joint]
    if joint in P.HIPS:
        links.append("foot_" + {v: k for k, v in P.PARENT_HIP.items()}[joint])
    mesh = trimesh.util.concatenate([m for l in links for m in _link_meshes(l)])
    pts, _ = trimesh.sample.sample_surface_even(mesh, SAMPLES, seed=1)
    pts = np.vstack([pts, mesh.vertices])
    # у самой оси — вал, шлиц и центральный винт качалки: они вращаются друг в друге по замыслу
    j = P.geometry()["joints"][joint]
    rel = pts - np.array(j["anchor_mm"])
    axis = np.array(j["axis"])
    radial = np.linalg.norm(rel - np.outer(rel @ axis, axis), axis=1)
    return pts[radial > AXIS_KEEP_OUT_MM]


def _parent_hits(joint: str, fw: float) -> np.ndarray:
    """Маска точек движущегося звена, попавших внутрь родителя (корпус для бедра, бедро для лапы)."""
    j = P.geometry()["joints"][joint]
    parent = "torso" if joint in P.HIPS else "femur_" + P.PARENT_HIP[joint]
    lo, grid = _occupancy(parent)
    a, axis = np.array(j["anchor_mm"]), np.array(j["axis"])
    ang = P.fw_to_rad(joint, fw)
    pts = _moving_points(joint) - a
    c, s = np.cos(ang), np.sin(ang)
    pts = pts * c + np.cross(axis, pts) * s + np.outer(pts @ axis, axis) * (1 - c) + a
    idx = np.round((pts - lo) / PITCH_MM).astype(int)
    ok = ((idx >= 0) & (idx < grid.shape)).all(axis=1)
    hit = np.zeros(len(pts), bool)
    hit[ok] = grid[tuple(idx[ok].T)]
    return hit


def _hull_hits(m, d, moving: set[int]) -> set[tuple[int, int]]:
    """Касания оболочек с чем угодно, кроме пола и своего родителя (его ловит _parent_hits)."""
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    pairs = set()
    for c in d.contact[:d.ncon]:
        if floor in (c.geom1, c.geom2) or c.dist > -1e-4:
            continue
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if (b1 in moving) != (b2 in moving):
            pairs.add(tuple(sorted((c.geom1, c.geom2))))
    return pairs


def _subtree(m: mujoco.MjModel, body: int) -> set[int]:
    out = {body}
    for b in range(m.nbody):
        p = b
        while p > 0:
            if p == body:
                out.add(b)
                break
            p = m.body_parentid[p]
    return out


@lru_cache(maxsize=None)
def safe_range(joint: str, step_deg: int = 1) -> tuple[int, int]:
    """Остальные суставы в stand, корпус в воздухе. От угла stand в обе стороны до первого нового касания:
    звено (и то, что на нём едет) входит в родителя глубже 0,5 мм или задевает другие звенья.
    Касания, которые есть уже в stand (качалка на валу), не считаются."""
    m, d = M.load()
    child = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ("femur_" if joint in P.HIPS else "foot_") + joint)
    moving = _subtree(m, child)
    qadr = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)]
    start = P.CAD_POSE_FW[joint]

    def hull_pairs(fw: int) -> set:
        M.set_pose(m, d, P.STAND_FW, on_floor=False)
        d.qpos[2] = 0.3
        d.qpos[qadr] = P.fw_to_rad(joint, fw)
        mujoco.mj_forward(m, d)
        return _hull_hits(m, d, moving)

    base_hull, base_pts = hull_pairs(start), _parent_hits(joint, start)

    def blocked(fw: int) -> bool:
        return bool(hull_pairs(fw) - base_hull) or int((_parent_hits(joint, fw) & ~base_pts).sum()) >= 3

    lo = hi = start
    for fw in range(start - step_deg, -1, -step_deg):
        if blocked(fw):
            break
        lo = fw
    for fw in range(start + step_deg, 181, step_deg):
        if blocked(fw):
            break
        hi = fw
    return lo, hi


def _picture(joint: str, fw: int, path: Path) -> None:
    m, d = M.load()
    fwv = list(P.STAND_FW)
    fwv[P.CHANNEL[joint]] = fw
    M.set_pose(m, d, fwv, on_floor=False)
    d.qpos[2] = 0.06
    mujoco.mj_forward(m, d)
    torso_vis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "torso_visual")
    saved = m.geom_rgba[torso_vis].copy()
    m.geom_rgba[torso_vis][3] = 0.35
    leg = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ("femur_" if joint in P.HIPS else "foot_") + joint)
    r = mujoco.Renderer(m, 360, 480)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = d.xpos[leg]
    cam.distance, cam.elevation = 0.22, -35
    side = np.sign(d.xpos[leg][1]) or 1
    cam.azimuth = 90 if side > 0 else -90
    r.update_scene(d, cam)
    img = r.render()
    r.close()
    m.geom_rgba[torso_vis] = saved
    path.parent.mkdir(parents=True, exist_ok=True)
    imageio.imwrite(path, img)


def write_calibration(out: Path = P.OUT) -> Path:
    lines = ["# Калибровка", "",
             "Безопасный диапазон: остальные суставы в stand, сустав ведётся от stand в обе стороны "
             "до первого касания корпуса или соседних деталей. Картинки — `calib/<сустав>_<угол>.png`.", "",
             "| Канал | Сустав | GPIO | Stand | Безопасно | 0° | 90° | 180° |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for ch, joint, gpio in channel_table(INO):
        lo, hi = safe_range(joint)
        pics = []
        for fw in (0, 90, 180):
            _picture(joint, fw, out / "calib" / f"{joint}_{fw}.png")
            pics.append(f"![{joint} {fw}](calib/{joint}_{fw}.png)")
        lines.append(f"| {ch} | {joint} | {gpio} | {P.CAD_POSE_FW[joint]} | {lo}–{hi} | " + " | ".join(pics) + " |")
    path = out / "calibration.md"
    path.write_text("\n".join(lines) + "\n")
    (out / "safe_ranges.json").write_text(json.dumps({j: safe_range(j) for j in P.JOINTS}))
    return path


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog="python -m twin.calibrate")
    ap.add_argument("joint", nargs="?", choices=P.JOINTS)
    ap.add_argument("--stop", type=int, help="угол прошивки, на котором сустав упёрся на живом роботе")
    a = ap.parse_args(argv)
    if not a.joint:
        print(f"готово: {write_calibration()}")
        return
    lo, hi = safe_range(a.joint)
    if a.stop is None:
        print(f"{a.joint}: безопасно {lo}–{hi}°")
        return
    computed = lo if abs(a.stop - lo) < abs(a.stop - hi) else hi
    fix = horn_fix(computed, a.stop)
    ch = P.CHANNEL[a.joint]
    if computed in (0, 180):
        print(f"{a.joint}: по модели упора нет до самого {computed}° — значит деталь надета со сдвигом "
              f"не меньше {abs(computed - a.stop)}°. Ниже — поправка на этот сдвиг; проверь второй упор тоже.")
    if fix.teeth:
        way = "меньших" if fix.teeth < 0 else "больших"
        print(f"{a.joint}: по модели упор на {computed}°, у тебя на {a.stop}°. Сними деталь с вала и переодень "
              f"на {abs(fix.teeth)} зуб. в сторону {way} углов (один зубец — {360 / P.SPLINE_TEETH:.0f}°).")
    else:
        print(f"{a.joint}: по модели упор на {computed}°, у тебя на {a.stop}° — переодевать не нужно.")
    print(f"Остаток: servoSubtrim[{ch}] = {fix.subtrim}")


if __name__ == "__main__":
    main()
