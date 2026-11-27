"""Редактор поз: ползунки сервоприводов в градусах прошивки (панель Control вьюера MuJoCo),
зеркало на живого робота по USB и экспорт позы функцией C++ для movement-sequences.h.

  hardware/v2/.venv/bin/mjpython -m twin.poser [--mirror /dev/cu.usbserial-110] [--protocol main|tester]

Клавиши: K — записать кадр, E — экспорт в терминал и out/poses/, 9 — все 90°, 8 — stand.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import numpy as np

from twin import model as M
from twin import params as P


def format_command(ch: int, fw: int, protocol: str) -> str:
    return f"{ch},{fw}\n" if protocol == "tester" else f"{ch} {fw}\n"


def mirror_allowed(joint: str, fw: int, safe: dict[str, tuple[int, int]]) -> bool:
    lo, hi = safe.get(joint, (0, 180))
    return lo <= fw <= hi


def round_pause(seconds: float) -> int:
    return max(50, int(round(seconds * 1000 / 50)) * 50)


def export_cpp(frames: list[tuple[list[int], int]], name: str) -> str:
    """Кадры (8 углов по каналам, пауза мс) → функция в стиле movement-sequences.h.
    В первом кадре ставятся все сервы, дальше — только изменившиеся."""
    fn = "run" + "".join(w.capitalize() for w in name.replace("-", "_").split("_")) + "Pose"
    lines = [f"inline void {fn}() {{", f'  Serial.println(F("{name.upper()}"));']
    prev: list[int] | None = None
    for angles, pause in frames:
        for ch, joint in enumerate(P.JOINTS):
            if prev is None or angles[ch] != prev[ch]:
                lines.append(f"  setServoAngle({joint}, {int(angles[ch])});")
        lines.append(f"  delayWithFace({int(pause)});")
        prev = list(angles)
    lines += ["  runStandPose(1);", "}"]
    return "\n".join(lines) + "\n"


class Mirror:
    def __init__(self, port, protocol: str):
        self.port, self.protocol, self.sent = port, protocol, [None] * 8

    def send(self, ch: int, fw: int) -> None:
        if self.sent[ch] != fw:
            self.port.write(format_command(ch, fw, self.protocol).encode())
            self.sent[ch] = fw

    def close(self) -> None:
        self.port.close()


def open_mirror(port: str | None, protocol: str = "main") -> Mirror | None:
    if not port:
        return None
    try:
        import serial
        return Mirror(serial.Serial(port, 115200, timeout=0), protocol)
    except Exception as e:   # нет порта, занят, нет прав — редактор работает без зеркала
        print(f"зеркало выключено: {port} не открылся ({e})")
        return None


def load_safe_ranges() -> dict[str, tuple[int, int]]:
    path = P.OUT / "safe_ranges.json"
    if path.exists():
        return {k: tuple(v) for k, v in json.loads(path.read_text()).items()}
    from twin.calibrate import safe_range
    return {j: safe_range(j) for j in P.JOINTS}


def main(argv: list[str] | None = None) -> None:
    import mujoco.viewer

    ap = argparse.ArgumentParser(prog="mjpython -m twin.poser")
    ap.add_argument("--mirror", help="порт робота, например /dev/cu.usbserial-110")
    ap.add_argument("--protocol", choices=("main", "tester"), default="main")
    ap.add_argument("--name", default="my", help="имя позы для экспорта")
    a = ap.parse_args(argv)

    m, d = M.load()
    M.set_pose(m, d, P.STAND_FW)
    mirror = open_mirror(a.mirror, a.protocol)
    safe = load_safe_ranges() if mirror else {}
    frames: list[tuple[list[int], int]] = []
    last_frame = [time.time()]
    pending: list[str] = []

    def on_key(key: int) -> None:
        pending.append(chr(key) if 32 <= key < 127 else "")

    with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 1
        while v.is_running():
            t0 = time.time()
            for k in pending:
                if k == "K":
                    frames.append(([int(round(c)) for c in d.ctrl], round_pause(time.time() - last_frame[0])))
                    last_frame[0] = time.time()
                    print(f"кадр {len(frames)}: {frames[-1][0]}")
                elif k == "E" and frames:
                    src = export_cpp(frames, a.name)
                    out = P.OUT / "poses" / f"{a.name}-{time.strftime('%H%M%S')}.h"
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_text(src)
                    print(src + f"\nсохранено: {out}")
                elif k == "9":
                    d.ctrl[:] = 90
                elif k == "8":
                    d.ctrl[:] = P.STAND_FW
            pending.clear()
            mujoco.mj_step(m, d)
            if mirror:
                for ch, joint in enumerate(P.JOINTS):
                    fw = int(round(d.ctrl[ch]))
                    if mirror_allowed(joint, fw, safe):
                        mirror.send(ch, fw)
            with v.lock():
                v.user_scn.ngeom = 0
                com = d.subtree_com[1]
                mujoco.mjv_initGeom(v.user_scn.geoms[0], mujoco.mjtGeom.mjGEOM_SPHERE, [0.004, 0, 0],
                                    np.array([com[0], com[1], 0.001]), np.eye(3).ravel(), np.array([1, 0.1, 0.1, 1], np.float32))
                v.user_scn.ngeom = 1
            v.sync()
            time.sleep(max(0.0, m.opt.timestep - (time.time() - t0)))
    if mirror:
        mirror.close()


if __name__ == "__main__":
    main()
