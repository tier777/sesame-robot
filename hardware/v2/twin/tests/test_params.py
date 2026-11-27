import numpy as np
import pytest

from conftest import FIRMWARE
from twin import params as P

INO = FIRMWARE / "sesame-firmware-main.ino"


def test_cad_pose_is_zero():
    assert all(abs(P.fw_to_rad(j, P.CAD_POSE_FW[j])) < 1e-12 for j in P.CHANNEL)


def test_cad_pose_is_stand():
    assert [P.CAD_POSE_FW[j] for j in sorted(P.CHANNEL, key=P.CHANNEL.get)] == P.STAND_FW


def test_roundtrip():
    assert abs(P.rad_to_fw("L3", P.fw_to_rad("L3", 37)) - 37) < 1e-9


def test_effective_fw_clamps():
    assert P.effective_fw(178, 5) == 175
    assert P.effective_fw(90, 0) == 90
    assert P.effective_fw(3, -5) == 5


def test_subtrim_and_pins_from_firmware():
    assert P.parse_subtrim(INO) == [0] * 8
    assert P.parse_pins(INO) == [15, 2, 23, 19, 4, 16, 17, 18]


def test_hip_forward_at_table():
    assert P.femur_direction("R1", 180) @ [1, 0, 0] > 0.996
    assert P.femur_direction("R2", 180) @ [1, 0, 0] > 0.996
    assert P.femur_direction("L1", 0) @ [1, 0, 0] > 0.996
    assert P.femur_direction("L2", 0) @ [1, 0, 0] > 0.996


def test_hips_sideways_at_90():
    # независимая проверка знака и масштаба: 90° — бедро прямо вбок
    assert P.femur_direction("R1", 90) @ [0, -1, 0] > 0.996
    assert P.femur_direction("L2", 90) @ [0, 1, 0] > 0.996


def test_foot_down_at_table():
    # «строго вниз» у автора номинально: в сборке кончик лапы завален внутрь на ~6°
    for j in ("R3", "R4", "L3", "L4"):
        assert P.foot_direction(j, P.FOOT_DOWN_AT[j]) @ [0, 0, -1] > 0.985


def test_feet_horizontal_outward_at_90():
    for foot, hip in (("R3", "R1"), ("R4", "R2"), ("L3", "L1"), ("L4", "L2")):
        d = P.foot_direction(foot, 90)
        assert abs(d[2]) < 0.2
        assert d @ P.femur_direction(hip, P.CAD_POSE_FW[hip]) > 0.98
