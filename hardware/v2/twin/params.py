"""Все числа двойника: каналы, перевод углов прошивки в углы суставов, приводы, массы."""
from __future__ import annotations

import json
import math
import os
import re
from functools import lru_cache
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).parent
VARIANT = os.environ.get("SESAME_TWIN_VARIANT", "v1")   # v1 — корпус автора; v2 — рама v2 с батареей внутри
OUT = HERE / ("out" if VARIANT == "v1" else f"out_{VARIANT}")
STEP = HERE.parents[2] / "hardware" / "cad" / "Sesame-ESP32-v122.step"

# каналы прошивки (enum ServoName в movement-sequences.h)
CHANNEL = {"R1": 0, "R2": 1, "L1": 2, "L2": 3, "R4": 4, "R3": 5, "L3": 6, "L4": 7}
JOINTS = sorted(CHANNEL, key=CHANNEL.get)                      # в порядке каналов
HIPS = ("R1", "R2", "L1", "L2")
FEET = ("R3", "R4", "L3", "L4")
PARENT_HIP = {"R3": "R1", "R4": "R2", "L3": "L1", "L4": "L2"}

STAND_FW = [135, 45, 45, 135, 0, 180, 0, 180]                  # runStandPose, по каналам
# Сборка автора в STEP стоит в позе stand: бёдра по диагонали, лапы вниз.
CAD_POSE_FW = {j: STAND_FW[CHANNEL[j]] for j in CHANNEL}
# Схема автора sesame-angle-guide.png: при каком угле бедро смотрит вперёд вдоль корпуса,
# при каком лапа смотрит строго вниз. 90° — бедро вбок, лапа горизонтально наружу.
HIP_FORWARD_AT = {"R1": 180, "R2": 180, "L1": 0, "L2": 0}
FOOT_DOWN_AT = {"R3": 180, "R4": 0, "L3": 0, "L4": 180}
SPLINE_TEETH = 20

# MG90S
SERVO_MASS_G = 13.4
SERVO_TORQUE_NM = 0.2
SERVO_SPEED_DPS = 600.0
SERVO_DEADBAND_DEG = 1.0
KP = 2.3            # Н·м/рад: 0,2 Н·м при отклонении ~5°
KV = 0.02

# материалы и электроника
DENSITY_PETG = 1270.0      # кг/м³
DENSITY_PLA = 1240.0
INFILL_FACTOR = 0.55
DEVKIT_G, OLED_G, MINI560_G = 10.0, 3.0, 3.0
BATTERY_V1 = {"mass_g": 50.0, "size_mm": [58.0, 31.0, 16.0], "center_mm": [-1.0, 0.0, 74.0]}  # отсек на седле
BATTERY_V2 = {"mass_g": 50.0, "size_mm": [58.0, 31.0, 16.0], "center_mm": [0.0, 0.0, 5.6]}   # отсек внутри рамы v2
BATTERY = BATTERY_V1 if VARIANT == "v1" else BATTERY_V2

# симуляция
DT = 0.002
FLOOR_FRICTION = 0.8
SETTLE_S = 0.5


@lru_cache(maxsize=1)
def geometry(path: Path | None = None) -> dict:
    path = path or OUT / "geometry.json"          # OUT меняется при use_variant — не запоминать его при импорте
    if not Path(path).exists():
        raise FileNotFoundError(f"нет {path}: сначала python -m twin.geometry")
    return json.loads(Path(path).read_text())


def _sign_from_tables(joint: str) -> int:
    # бедро: «вперёд» из схемы автора (в позе сборки бедро по диагонали, так что поворот ненулевой);
    # лапа: «вниз» — это и есть поза сборки, поэтому знак берётся из «на 90° лапа горизонтально наружу»
    for s in (1, -1):
        if joint in HIPS and _femur(joint, HIP_FORWARD_AT[joint], s) @ [1, 0, 0] > 0.9:
            return s
        if joint in FEET:
            outward = _femur(PARENT_HIP[joint], CAD_POSE_FW[PARENT_HIP[joint]], 1)
            if _foot(joint, 90, s) @ outward > 0.9:
                return s
    raise ValueError(f"не получается совместить ось {joint} со схемой углов автора")


@lru_cache(maxsize=None)
def joint_sign(joint: str) -> int:
    """+1, если рост угла прошивки — поворот по оси сустава из геометрии, −1 — против."""
    return _sign_from_tables(joint)


def fw_to_rad(joint: str, fw: float) -> float:
    return joint_sign(joint) * math.radians(fw - CAD_POSE_FW[joint])


def rad_to_fw(joint: str, q: float) -> float:
    return CAD_POSE_FW[joint] + math.degrees(q) * joint_sign(joint)


def effective_fw(fw: int, subtrim: int) -> int:
    """Угол, который реально встаёт на идеальном роботе: setServoAngle обрезает fw+subtrim до 0–180."""
    return min(max(fw + subtrim, 0), 180) - subtrim


def _array_from_ino(ino: Path, name: str) -> list[int]:
    m = re.search(rf"^(?!\s*//).*?\b{name}\[8\]\s*=\s*\{{([^}}]*)\}}", Path(ino).read_text(), re.M)
    if not m:
        raise ValueError(f"в {ino} нет {name}[8]")
    return [int(v) for v in m.group(1).split(",")]


def parse_subtrim(ino: Path) -> list[int]:
    return _array_from_ino(ino, "servoSubtrim")


def parse_pins(ino: Path) -> list[int]:
    return _array_from_ino(ino, "servoPins")


def rotate(v: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    v = np.asarray(v, float)
    return v * math.cos(angle) + np.cross(axis, v) * math.sin(angle) + axis * (axis @ v) * (1 - math.cos(angle))


def _femur(hip: str, fw: float, sign: int) -> np.ndarray:
    g = geometry()
    knee = g["joints"][_foot_of(hip)]
    hip_j = g["joints"][hip]
    k = np.array(knee["axis"])
    d = np.cross([0, 0, 1], k)                       # горизонталь поперёк оси колена
    d /= np.linalg.norm(d)
    if d @ (np.array(knee["anchor_mm"]) - np.array(hip_j["anchor_mm"])) < 0:
        d = -d
    return rotate(d, hip_j["axis"], sign * math.radians(fw - CAD_POSE_FW[hip]))


def _foot_of(hip: str) -> str:
    return {v: k for k, v in PARENT_HIP.items()}[hip]


@lru_cache(maxsize=None)
def _foot_tip(foot: str) -> np.ndarray:
    g = geometry()
    j = g["joints"][foot]
    mesh = trimesh.load(OUT / g["links"][f"foot_{foot}"]["visual"])
    a, k = np.array(j["anchor_mm"]), np.array(j["axis"])
    rel = mesh.vertices - a
    rel -= np.outer(rel @ k, k)                      # в плоскости вращения
    return rel[np.argmax(np.linalg.norm(rel, axis=1))]


def _foot(foot: str, fw: float, sign: int) -> np.ndarray:
    tip = _foot_tip(foot)
    d = tip / np.linalg.norm(tip)
    return rotate(d, geometry()["joints"][foot]["axis"], sign * math.radians(fw - CAD_POSE_FW[foot]))


def femur_direction(hip: str, fw: float) -> np.ndarray:
    """Единичный вектор «куда смотрит бедро» (горизонталь, поперёк оси колена) при угле прошивки fw."""
    return _femur(hip, fw, joint_sign(hip))


def foot_direction(foot: str, fw: float) -> np.ndarray:
    """Единичный вектор от оси колена к кончику лапы при угле прошивки fw (бедро в позе сборки)."""
    return _foot(foot, fw, joint_sign(foot))


def use_variant(name: str) -> None:
    """Переключить двойник на вариант корпуса (v1 / v2) в этом процессе: OUT, батарея и все кеши."""
    global VARIANT, OUT, BATTERY
    VARIANT = name
    OUT = HERE / ("out" if name == "v1" else f"out_{name}")
    BATTERY = BATTERY_V1 if name == "v1" else BATTERY_V2
    geometry.cache_clear()
    joint_sign.cache_clear()
    _foot_tip.cache_clear()
    import sys
    for mod, names in (("twin.model", ("_compiled",)),
                       ("twin.calibrate", ("safe_range", "_occupancy", "_moving_points"))):
        if mod in sys.modules:
            for n in names:
                getattr(sys.modules[mod], n).cache_clear()
