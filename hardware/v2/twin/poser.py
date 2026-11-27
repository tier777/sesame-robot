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
    lines = [f"inline void {fn}() {{", f'  Serial.println(F("{name.upper()}"));', "  runStandPose(0);",
             "  delayWithFace(200);"]
    prev: list[int] | None = None
    for angles, pause in frames:
        for ch, joint in enumerate(P.JOINTS):
            if prev is None or angles[ch] != prev[ch]:
                lines.append(f"  setServoAngle({joint}, {int(angles[ch])});")
        lines.append(f"  delayWithFace({int(pause)});")
        prev = list(angles)
    lines += ["  runStandPose(1);", f'  if (currentCommand == "{name}") currentCommand = "";', "}"]
    return "\n".join(lines) + "\n"


MIRROR_INTERVAL_S = 0.05     # не чаще 20 команд в секунду: прошивка разбирает ~40/с (каждая ждёт motorCurrentDelay)


class Mirror:
    """Зеркало на робота: по каналу хранится только последнее значение ползунка; в порт — одна команда
    за интервал, по очереди каналов. Так входной буфер платы не переполняется и строки не склеиваются."""

    def __init__(self, port, protocol: str):
        self.port, self.protocol = port, protocol
        self.sent: list[int | None] = [None] * 8
        self.want: list[int | None] = [None] * 8
        self.alive = True
        self._last = -1e9
        self._next = 0

    def update(self, ch: int, fw: int) -> None:
        self.want[ch] = fw

    def flush(self, now: float) -> None:
        if not self.alive or now - self._last < MIRROR_INTERVAL_S:
            return
        for i in range(8):
            ch = (self._next + i) % 8
            if self.want[ch] is not None and self.want[ch] != self.sent[ch]:
                try:
                    self.port.write(format_command(ch, self.want[ch], self.protocol).encode())
                except Exception as e:     # кабель выдернут, порт пропал — редактор работает дальше
                    print(f"зеркало выключено: {e}")
                    self.alive = False
                    return
                self.sent[ch] = self.want[ch]
                self._last, self._next = now, ch + 1
                return

    def close(self) -> None:
        try:
            self.port.close()
        except Exception:
            pass


def open_mirror(port: str | None, protocol: str = "main", factory=None) -> Mirror | None:
    if not port:
        return None
    try:
        if factory is None:
            import serial
            factory = serial.Serial
        s = factory()
        s.port, s.baudrate, s.timeout, s.write_timeout = port, 115200, 0, 0.2
        s.dtr = s.rts = False          # иначе открытие порта перезагружает ESP32 DevKit
        s.open()
        return Mirror(s, protocol)
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
            if mirror and mirror.alive:
                for ch, joint in enumerate(P.JOINTS):
                    fw = int(round(d.ctrl[ch]))
                    if mirror_allowed(joint, fw, safe):
                        mirror.update(ch, fw)
                mirror.flush(time.monotonic())
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
