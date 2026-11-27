import json

import numpy as np
import pytest

from twin import params as P

V2_STL = P.HERE.parent / "body" / "out"


@pytest.fixture
def v2():
    if not (V2_STL / "frame_v2.stl").exists():
        pytest.skip("нет body/out/frame_v2.stl: сначала python -m body.frame_v2")
    P.use_variant("v2")
    yield
    P.use_variant("v1")


def test_variant_swaps_frame(tmp_path):
    from twin.geometry import build_variant
    if not (V2_STL / "frame_v2.stl").exists():
        pytest.skip("нет STL v2")
    g = build_variant(P.HERE / "out", {"Internal-Frame:1": V2_STL / "frame_v2.stl",
                                       "Bottom-Cover:1": V2_STL / "bay_lid_v2.stl"}, tmp_path)
    parts = g["links"]["torso"]["parts"]
    assert "Internal-Frame:1" not in parts and "Bottom-Cover:1" not in parts
    assert "frame_v2" in parts and "bay_lid_v2" in parts
    base = json.loads((P.HERE / "out" / "geometry.json").read_text())
    assert g["links"]["femur_R1"] == base["links"]["femur_R1"] and g["joints"] == base["joints"]
    assert (tmp_path / g["links"]["femur_R1"]["visual"]).exists()


def test_use_variant_switches_out_and_battery():
    P.use_variant("v2")
    try:
        assert P.OUT.name == "out_v2" and P.BATTERY["center_mm"][2] < 10
    finally:
        P.use_variant("v1")
    assert P.OUT.name == "out" and P.BATTERY["center_mm"][2] > 60


def test_battery_moves_down(v2):
    from twin import model as M
    import mujoco
    if not (P.OUT / "geometry.json").exists():
        pytest.skip("нет out_v2/geometry.json")
    m, d = M.load()
    M.set_pose(m, d, P.STAND_FW, on_floor=False)
    com_v2 = d.subtree_com[1][2]
    P.use_variant("v1")
    m1, d1 = M.load()
    M.set_pose(m1, d1, P.STAND_FW, on_floor=False)
    assert com_v2 < d1.subtree_com[1][2] - 0.010
