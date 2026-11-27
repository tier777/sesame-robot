"""Разбор логов симуляции: по сервам, столкновения, устойчивость, ток; report.md и видео."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np
from scipy.spatial import ConvexHull, QhullError

from twin import model as M
from twin import params as P
from twin.simulate import Log

STALL_MIN_S = 0.2
I_IDLE, I_PER_NM = 0.1, 3.3


@dataclass
class ServoStats:
    fw_min: float
    fw_max: float
    speed_peak_dps: float
    torque_peak_pct: float
    torque_mean_pct: float
    stall_s: float


@dataclass
class PoseStats:
    servos: list[ServoStats]
    collisions: list = field(default_factory=list)      # (тело_a, тело_b, t, fw[8])
    margin_min_mm: float = 0.0
    margin_typical_mm: float = 0.0                         # медиана по времени
    outside_share: float = 0.0                             # доля времени, когда центр масс вне опоры
    out_of_range: list = field(default_factory=list)      # (сустав, угол, (lo, hi)) — команда за упором
    status: str = "ok"
    walk_dx_mm_per_cycle: float | None = None
    current_peak_a: float = 0.0
    current_mean_a: float = 0.0
    duration_s: float = 0.0


def current_a(torque_nm: np.ndarray) -> np.ndarray:
    return I_IDLE + I_PER_NM * np.abs(torque_nm)


def _seg_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip((p - a) @ ab / (ab @ ab), 0, 1))
    return float(np.linalg.norm(p - (a + t * ab)))


def support_margin_mm(com_xy, contacts_xy) -> float:
    """Расстояние от проекции центра масс до края опоры: внутри — плюс, снаружи — минус.
    Меньше трёх точек (или все на одной линии) — минус расстояние до отрезка/точки."""
    p = np.asarray(com_xy, float)
    pts = np.unique(np.asarray(contacts_xy, float).reshape(-1, 2), axis=0)
    if len(pts) == 0:
        return -np.inf
    if len(pts) >= 3:
        try:
            hull = ConvexHull(pts)
        except QhullError:
            hull = None
        if hull is not None:
            # уравнения граней: n·x + c <= 0 внутри, n — единичная нормаль
            signed = hull.equations[:, :2] @ p + hull.equations[:, 2]
            if (signed <= 0).all():
                return float(-signed.max())
            ring = pts[hull.vertices]
            return -min(_seg_dist(p, ring[i], ring[(i + 1) % len(ring)]) for i in range(len(ring)))
    if len(pts) == 1:
        return -float(np.linalg.norm(p - pts[0]))
    far = max(((a, b) for a in pts for b in pts), key=lambda ab: np.linalg.norm(ab[0] - ab[1]))
    return -_seg_dist(p, far[0], far[1])


def _longest_run(mask: np.ndarray, dt: float) -> float:
    best = run = 0
    for v in mask:
        run = run + 1 if v else 0
        best = max(best, run)
    return best * dt


def analyze(log: Log, walk_cycles: int | None = None, safe: dict | None = None) -> PoseStats:
    dt = float(np.median(np.diff(log.t))) if len(log.t) > 1 else P.DT
    servos = []
    for ch in range(8):
        q, tau = log.q_fw[:, ch], np.abs(log.torque[:, ch])
        speed = np.abs(np.diff(q)) / dt if len(q) > 1 else np.zeros(1)
        stall = _longest_run(tau >= P.SERVO_TORQUE_NM * 0.999, dt)
        servos.append(ServoStats(float(q.min()), float(q.max()), float(speed.max()),
                                 float(tau.max() / P.SERVO_TORQUE_NM * 100),
                                 float(tau.mean() / P.SERVO_TORQUE_NM * 100),
                                 stall if stall >= STALL_MIN_S else 0.0))
    collisions, previous = [], set()
    for k, step in enumerate(log.contacts):
        pairs = {tuple(sorted((a, b))) for a, b, _ in step}
        for pair in sorted(pairs - previous):
            collisions.append((pair[0], pair[1], float(log.t[k]), log.q_fw[k].round().tolist()))
        previous = pairs
    margins = [support_margin_mm(log.com[k, :2] * 1000, log.foot_contacts[k] * 1000)
               for k in range(len(log.t)) if len(log.foot_contacts[k])]
    total_i = current_a(log.torque).sum(axis=1)
    out_of_range = []
    for joint, (lo, hi) in (safe or {}).items():
        cmd = log.target_fw[:, P.CHANNEL[joint]]
        if len(cmd) and cmd.min() < lo:
            out_of_range.append((joint, int(round(cmd.min())), (lo, hi)))
        if len(cmd) and cmd.max() > hi:
            out_of_range.append((joint, int(round(cmd.max())), (lo, hi)))
    dx = None
    if walk_cycles:
        tips = log.foot_tips[0]
        heading = (tips[0] + tips[2]) / 2 - (tips[1] + tips[3]) / 2
        heading = heading[:2] / np.linalg.norm(heading[:2])
        dx = float((log.torso_pos[-1, :2] - log.torso_pos[0, :2]) @ heading * 1000 / walk_cycles)
    return PoseStats(servos=servos, collisions=collisions,
                     margin_min_mm=float(min(margins)) if margins else float("-inf"),
                     margin_typical_mm=float(np.median(margins)) if margins else float("-inf"),
                     outside_share=float(np.mean(np.array(margins) < 0)) if margins else 1.0,
                     out_of_range=out_of_range,
                     status=log.status, walk_dx_mm_per_cycle=dx,
                     current_peak_a=float(total_i.max()) if len(total_i) else 0.0,
                     current_mean_a=float(total_i.mean()) if len(total_i) else 0.0,
                     duration_s=float(log.t[-1]) if len(log.t) else 0.0)


STATUS_RU = {"ok": "стоит", "fell": "упал", "unstable": "сломалась физика"}


def write(stats: dict[str, PoseStats], failed: dict[str, str], out: Path, videos: str = "videos") -> Path:
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    L = ["# Отчёт двойника Sesame", "",
         f"MG90S: упор {P.SERVO_TORQUE_NM} Н·м, скорость {P.SERVO_SPEED_DPS:.0f}°/с. "
         f"Ток — оценка {I_IDLE} А + {I_PER_NM} А/(Н·м) × момент на серву. "
         "Запас устойчивости — от проекции центра масс до края опоры: плюс — внутри, минус — снаружи; "
         "«мин» — худшее мгновение (переходы, шаг на двух ногах), «обычно» — медиана по времени.", ""]
    if stats:
        L += ["## Позы", "", "| Поза | Итог | Длится, с | Запас мин / обычно, мм | Вне опоры | Ток пик / средний, А | Шаг, мм/цикл | Видео |",
              "| --- | --- | --- | --- | --- | --- | --- | --- |"]
        for name, s in stats.items():
            step = "" if s.walk_dx_mm_per_cycle is None else f"{s.walk_dx_mm_per_cycle:+.1f}"
            L.append(f"| {name} | {STATUS_RU.get(s.status, s.status)} | {s.duration_s:.1f} | "
                     f"{s.margin_min_mm:.0f} / {s.margin_typical_mm:.0f} | {s.outside_share * 100:.0f} % | "
                     f"{s.current_peak_a:.2f} / {s.current_mean_a:.2f} | {step} | [mp4]({videos}/{name}.mp4) |")
        L += ["", "## Сервы", "", "Диапазон — во всех позах; пик момента и время в упоре — с позой, где они случились.", "",
              "| Канал | Сустав | Угол мин–макс | Скорость пик, °/с | Момент пик, % | Где | В упоре, с | Где |",
              "| --- | --- | --- | --- | --- | --- | --- | --- |"]
        for ch, joint in enumerate(P.JOINTS):
            per = {n: s.servos[ch] for n, s in stats.items()}
            lo = min(v.fw_min for v in per.values())
            hi = max(v.fw_max for v in per.values())
            sp = max(v.speed_peak_dps for v in per.values())
            tp_pose = max(per, key=lambda n: per[n].torque_peak_pct)
            st_pose = max(per, key=lambda n: per[n].stall_s)
            L.append(f"| {ch} | {joint} | {lo:.0f}–{hi:.0f} | {sp:.0f} | {per[tp_pose].torque_peak_pct:.0f} | {tp_pose} | "
                     f"{per[st_pose].stall_s:.2f} | {st_pose if per[st_pose].stall_s else '—'} |")
        L += ["", "## За упором", "",
              "Команда прошивки дальше безопасного диапазона из калибровки (там деталь упирается в корпус "
              "или соседнюю ногу): серва давит в упор, пока поза не сменится.", ""]
        over = [(n, j, a, r) for n, s in stats.items() for j, a, r in s.out_of_range]
        L += [f"- **{n}**: {j} → {a}°, безопасно {r[0]}–{r[1]}°" for n, j, a, r in over] or ["Нет."]
        L += ["", "## Столкновения", ""]
        any_hit = False
        for name, s in stats.items():
            for a, b, t, fw in s.collisions:
                any_hit = True
                L.append(f"- **{name}**, {t:.2f} с: {a} ↔ {b}; углы {', '.join(f'{j} {int(fw[c])}' for c, j in enumerate(P.JOINTS))}")
        if not any_hit:
            L.append("Нет: детали касаются только пола.")
    if failed:
        L += ["", "## Не записались", ""]
        for name, why in failed.items():
            L += [f"- **{name}**:", "", "```", why, "```"]
    path = out / "report.md"
    path.write_text("\n".join(L) + "\n")
    return path


def render_video(log: Log, path: Path, fps: int = 30) -> None:
    m, d = M.load()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    r = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera()
    cam.distance, cam.azimuth, cam.elevation = 0.42, 135, -25
    every = max(1, int(round(1 / fps / (log.t[1] - log.t[0])))) if len(log.t) > 1 else 1
    with imageio.get_writer(path, fps=fps, codec="libx264", quality=6, macro_block_size=8) as w:
        for k in range(0, len(log.t), every):
            d.qpos[:] = log.qpos[k]
            mujoco.mj_forward(m, d)
            cam.lookat[:] = [d.qpos[0], d.qpos[1], 0.03]
            r.update_scene(d, cam)
            w.append_data(r.render())
    r.close()
