"""Проигрывание записей поз прошивки в MuJoCo с ограничениями MG90S."""
from __future__ import annotations

import math
import pickle
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from twin import model as M
from twin import params as P

INO = Path(__file__).resolve().parents[3] / "firmware" / "sesame-firmware-main.ino"


@dataclass
class Log:
    t: np.ndarray                     # с, от начала записи (после успокоения)
    target_fw: np.ndarray             # N×8, цель после ограничения скорости
    q_fw: np.ndarray                  # N×8, факт в углах прошивки
    torque: np.ndarray                # N×8, Н·м
    com: np.ndarray                   # N×3, м
    torso_pos: np.ndarray             # N×3, м
    torso_tilt_deg: np.ndarray        # N
    foot_tips: np.ndarray             # N×4×3, кончики лап R3 R4 L3 L4, м
    foot_contacts: list = field(default_factory=list)   # на шаг: массив xy точек касания пола
    contacts: list = field(default_factory=list)        # на шаг: [(тело_a, тело_b, сила Н)] кроме пола
    status: str = "ok"                # ok | fell | unstable
    qpos: np.ndarray | None = None    # N×nq, для видео


def _tip_local(m: mujoco.MjModel) -> dict[str, tuple[int, np.ndarray]]:
    out = {}
    for foot in ("R3", "R4", "L3", "L4"):
        body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"foot_{foot}")
        anchor = np.array(P.geometry()["joints"][foot]["anchor_mm"])
        tip_world_cad = (anchor + P._foot_tip(foot)) * M.MM          # в позе сборки корпус в начале координат
        out[foot] = (body, tip_world_cad - anchor * M.MM)            # тело лапы стоит в якоре, без поворота
    return out


BLOWUP = (mujoco.mjtWarning.mjWARN_BADQACC, mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQPOS)


def blew_up(d: mujoco.MjData) -> bool:
    """MuJoCo при NaN сам сбрасывает состояние в исходное — qpos остаётся конечным; ловим по предупреждениям."""
    return any(d.warning[w].number > 0 for w in BLOWUP) or not np.isfinite(d.qpos).all()


def run(trace: dict, settle_s: float = P.SETTLE_S, dt: float = P.DT, tail_s: float = 0.5,
        subtrim: list[int] | None = None) -> Log:
    m, d = M.load()
    m.opt.timestep = dt
    subtrim = P.parse_subtrim(INO) if subtrim is None else subtrim
    M.set_pose(m, d, P.STAND_FW)
    command = np.array(P.STAND_FW, float)
    target = command.copy()
    for _ in range(int(settle_s / dt)):
        d.ctrl[:] = target
        mujoco.mj_step(m, d)

    events = sorted(trace.get("events", []), key=lambda e: e[0])
    steps = int((trace.get("duration_ms", 0) / 1000 + tail_s) / dt)
    qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in P.JOINTS]
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso")
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    tips = _tip_local(m)
    rec = {k: [] for k in ("t", "target", "q", "torque", "com", "pos", "tilt", "tips", "qpos")}
    foot_contacts, contacts, status = [], [], "ok"
    max_step = P.SERVO_SPEED_DPS * dt
    ei = 0
    force = np.zeros(6)
    for k in range(steps):
        t = k * dt
        while ei < len(events) and events[ei][0] / 1000 <= t + 1e-9:
            _, ch, angle = events[ei]
            angle = P.effective_fw(angle, subtrim[ch]) if float(angle).is_integer() else angle
            if abs(angle - command[ch]) >= P.SERVO_DEADBAND_DEG:
                command[ch] = angle
            ei += 1
        target += np.clip(command - target, -max_step, max_step)
        d.ctrl[:] = target
        mujoco.mj_step(m, d)
        if blew_up(d):
            status = "unstable"
            break
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, d.xmat[torso][8]))))
        rec["t"].append(t)
        rec["target"].append(target.copy())
        rec["q"].append([P.rad_to_fw(j, d.qpos[a]) for j, a in zip(P.JOINTS, qadr)])
        rec["torque"].append(d.actuator_force.copy())
        rec["com"].append(d.subtree_com[torso].copy())
        rec["pos"].append(d.xpos[torso].copy())
        rec["tilt"].append(tilt)
        rec["qpos"].append(d.qpos.copy())
        rec["tips"].append([d.xpos[b] + d.xmat[b].reshape(3, 3) @ off for b, off in tips.values()])
        fc, other = [], []
        for i, c in enumerate(d.contact[:d.ncon]):
            if floor in (c.geom1, c.geom2):
                fc.append(c.pos[:2].copy())
            else:
                mujoco.mj_contactForce(m, d, i, force)
                other.append((mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom1]),
                              mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[c.geom2]),
                              float(abs(force[0]))))
        foot_contacts.append(np.array(fc).reshape(-1, 2))
        contacts.append(other)
        if tilt > 45:
            status = "fell"
            break
    arr = {k: np.array(v, float) for k, v in rec.items()}
    if not len(arr["t"]):
        arr = {k: np.zeros((0,)) for k in rec}
    return Log(t=arr["t"], target_fw=arr["target"].reshape(-1, 8), q_fw=arr["q"].reshape(-1, 8),
               torque=arr["torque"].reshape(-1, 8), com=arr["com"].reshape(-1, 3),
               torso_pos=arr["pos"].reshape(-1, 3), torso_tilt_deg=arr["tilt"],
               foot_tips=arr["tips"].reshape(-1, 4, 3), foot_contacts=foot_contacts,
               contacts=contacts, status=status, qpos=arr["qpos"].reshape(-1, m.nq))


def foot_slip_mm(log: Log) -> float:
    """Наибольший сдвиг по полу кончика лапы, пока он стоит на полу (ниже 2 мм)."""
    worst = 0.0
    for f in range(4):
        z = log.foot_tips[:, f, 2]
        on = z < 0.002
        if on.any():
            xy = log.foot_tips[on, f, :2]
            worst = max(worst, float(np.linalg.norm(xy - xy[0], axis=1).max()) * 1000)
    return worst


def save(log: Log, path: Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(pickle.dumps(log))


def load(path: Path) -> Log:
    return pickle.loads(Path(path).read_bytes())
