"""Запись поз прошивки: каждая поза собирается на компьютере с заглушкой Arduino
и печатает время, канал и угол на каждый setServoAngle."""
from __future__ import annotations

import json
import os
import re
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).parent

# имя записи → (функция в movement-sequences.h, значение currentCommand из диспетчера прошивки)
POSES: dict[str, tuple[str, str]] = {
    **{n: (f"run{n.capitalize()}Pose", n) for n in (
        "rest", "stand", "wave", "dance", "swim", "point", "pushup", "bow", "cute",
        "freaky", "worm", "shake", "shrug", "dead", "crab")},
    "walk": ("runWalkPose", "forward"),
    "walk_backward": ("runWalkBackward", "backward"),
    "turn_left": ("runTurnLeft", "left"),
    "turn_right": ("runTurnRight", "right"),
}


@dataclass
class TraceResult:
    traces: dict[str, list[tuple[int, int, int]]] = field(default_factory=dict)
    failed: dict[str, str] = field(default_factory=dict)


def _first_lines(text: str, n: int = 6) -> str:
    return "\n".join([line for line in text.splitlines() if line.strip()][:n])


_SYSROOT: list[str] | None = None


def _sysroot_flags() -> list[str]:
    """Флаги SDK, с которыми clang++ на этой машине собирает и линкует программу.
    Самый новый SDK из Command Line Tools бывает новее линкера — тогда берётся следующий."""
    global _SYSROOT
    if _SYSROOT is not None:
        return _SYSROOT
    probe = HERE / "build" / "probe"
    probe.parent.mkdir(exist_ok=True)
    src = probe.with_suffix(".cpp")
    src.write_text("int main() { return 0; }\n")
    sdks = sorted(Path("/Library/Developer/CommandLineTools/SDKs").glob("MacOSX*.*.sdk"),
                  key=lambda p: [int(x) for x in re.findall(r"\d+", p.name)], reverse=True)
    for flags in [[]] + [["-isysroot", str(s)] for s in sdks]:
        ok = subprocess.run(["clang++", *flags, str(src), "-o", str(probe)], capture_output=True)
        if ok.returncode == 0:
            _SYSROOT = flags
            return flags
    raise RuntimeError("clang++ не может собрать даже пустую программу: проверь Xcode Command Line Tools")


_DEF = re.compile(r"^inline\s+[\w:<>&\s\*]+?\b(\w+)\s*\([^)]*\)\s*\{", re.M)


def _split_header(text: str) -> list[tuple[str | None, str]]:
    """Делит заголовок на куски: (имя функции, текст определения) и (None, всё остальное)."""
    parts, pos = [], 0
    for m in _DEF.finditer(text):
        if m.start() < pos:
            continue
        depth, i = 0, m.end() - 1
        while True:
            depth += {"{": 1, "}": -1}.get(text[i], 0)
            i += 1
            if depth == 0:
                break
        parts.append((None, text[pos:m.start()]))
        parts.append((m.group(1), text[m.start():i]))
        pos = i
    parts.append((None, text[pos:]))
    return parts


def _pose_header(parts: list[tuple[str | None, str]], fn: str) -> str:
    """Заголовок только с нужной позой и тем, что она вызывает: ошибка в одной позе не ломает остальные."""
    bodies = {name: body for name, body in parts if name}
    need, todo = set(), [fn, "runStandPose"]  # runStandPose зовёт и обвязка (pressingCheck)
    while todo:
        name = todo.pop()
        if name in need or name not in bodies:
            continue
        need.add(name)
        todo += [other for other in bodies if other != name and re.search(rf"\b{other}\b", bodies[name])]
    return "".join(body for name, body in parts if name is None or name in need)


MAX_OUTPUT = 2_000_000      # байт: поза, которая пишет без конца, тоже считается зависшей


def _run_capped(exe: Path, timeout_s: float) -> tuple[str, str]:
    """Запуск позы: вывод не больше MAX_OUTPUT, не дольше timeout_s. Возвращает (вывод, ошибка)."""
    import tempfile
    with tempfile.TemporaryFile() as out:
        proc = subprocess.Popen([str(exe)], stdout=out, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + timeout_s
        while proc.poll() is None:
            too_long = time.monotonic() > deadline
            too_big = os.fstat(out.fileno()).st_size > MAX_OUTPUT
            if too_long or too_big:
                proc.kill()
                proc.wait()
                why = f"не закончилась за {timeout_s:g} с" if too_long else f"вывод больше {MAX_OUTPUT} байт"
                return "", f"зависла: {why} (цикл без pressingCheck?)"
            time.sleep(0.02)
        out.seek(0)
        text = out.read().decode(errors="replace")
    if proc.returncode != 0 or "END" not in text:
        return "", _first_lines(text) or f"выход {proc.returncode}"
    return text, ""


def build_traces(firmware_dir: Path, out_dir: Path, frame_delay: int = 100,
                 walk_cycles: int = 10, motor_delay: int = 20, timeout_s: float = 20) -> TraceResult:
    firmware_dir, out_dir = Path(firmware_dir), Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    build = HERE / "build"
    build.mkdir(exist_ok=True)
    result = TraceResult()
    chunks = _split_header((firmware_dir / "movement-sequences.h").read_text())
    for name, (fn, cmd) in POSES.items():
        inc = build / name
        inc.mkdir(exist_ok=True)
        (inc / "movement-sequences.h").write_text(_pose_header(chunks, fn))
        exe = inc / "pose"
        cc = subprocess.run(
            ["clang++", *_sysroot_flags(), "-std=c++17", "-w", "-I", str(HERE / "arduino_stub"), "-I", str(inc),
             f"-DPOSE_FN={fn}", f'-DPOSE_CMD="{cmd}"', f"-DFRAME_DELAY={frame_delay}",
             f"-DWALK_CYCLES={walk_cycles}", f"-DMOTOR_DELAY={motor_delay}",
             str(HERE / "harness.cpp"), "-o", str(exe)],
            capture_output=True, text=True)
        if cc.returncode != 0:
            result.failed[name] = _first_lines(cc.stderr)
            continue
        stdout, err = _run_capped(exe, timeout_s)
        if err:
            result.failed[name] = err
            continue
        events, duration = [], 0
        for line in stdout.splitlines():
            parts = line.split()
            if parts[0] == "END":
                duration = int(parts[1])
            else:
                events.append(tuple(int(p) for p in parts))
        result.traces[name] = events
        (out_dir / f"{name}.json").write_text(json.dumps({
            "pose": name, "frame_delay": frame_delay, "walk_cycles": walk_cycles,
            "motor_delay": motor_delay, "events": events, "duration_ms": duration}))
    return result
