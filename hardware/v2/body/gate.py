"""Ворота сборки v2: прошивка не должна упираться в раме v2 там, где не упиралась в v1.
  python -m body.gate   (сравнивает twin/out/safe_ranges.json и twin/out_v2/safe_ranges.json по записям поз)"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from twin import params as P


def over_range(traces_dir: Path, safe: dict) -> set[tuple[str, str, int]]:
    """(поза, сустав, крайний угол), где команда прошивки выходит за безопасный диапазон."""
    out = set()
    for f in sorted(Path(traces_dir).glob("*.json")):
        cmds = {}
        for _, ch, angle in json.loads(f.read_text())["events"]:
            cmds.setdefault(P.JOINTS[ch], []).append(angle)
        for joint, (lo, hi) in safe.items():
            a = cmds.get(joint, [])
            if a and min(a) < lo:
                out.add((f.stem, joint, min(a)))
            if a and max(a) > hi:
                out.add((f.stem, joint, max(a)))
    return out


def new_stops(traces_dir: Path, v1_safe: dict, v2_safe: dict) -> set[tuple[str, str, int]]:
    return over_range(traces_dir, v2_safe) - over_range(traces_dir, v1_safe)


def main() -> None:
    load = lambda d: {k: tuple(v) for k, v in json.loads((P.HERE / d / "safe_ranges.json").read_text()).items()}
    new = new_stops(P.HERE / "traces", load("out"), load("out_v2"))
    if new:
        for pose, joint, a in sorted(new):
            print(f"v2 упирается там, где v1 нет: {pose} — {joint} {a}°", file=sys.stderr)
        sys.exit(1)
    print("ворота: новых упоров у v2 нет")


if __name__ == "__main__":
    main()
