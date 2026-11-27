"""Все позы прошивки: запись → симуляция → отчёт и видео.  python -m twin.pipeline"""
from __future__ import annotations

from twin import params as P
from twin.firmware_trace.trace import build_traces
from twin.report import analyze, render_video, write
from twin.simulate import INO, run, save

FIRMWARE = INO.parent


def main() -> None:
    import json
    traces_dir = P.HERE / "traces"
    safe_file = P.OUT / "safe_ranges.json"
    safe = {k: tuple(v) for k, v in json.loads(safe_file.read_text()).items()} if safe_file.exists() else None
    result = build_traces(FIRMWARE, traces_dir)
    stats = {}
    for name in result.traces:
        trace = json.loads((traces_dir / f"{name}.json").read_text())
        log = run(trace)
        save(log, P.OUT / "logs" / f"{name}.pkl")
        walk = trace["walk_cycles"] if name.startswith(("walk", "turn")) else None
        stats[name] = analyze(log, walk_cycles=walk, safe=safe)
        render_video(log, P.OUT / "videos" / f"{name}.mp4")
        print(f"{name}: {stats[name].status}")
    print(f"готово: {write(stats, result.failed, P.OUT).relative_to(P.HERE.parents[2])}")


if __name__ == "__main__":
    main()
