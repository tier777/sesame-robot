import numpy as np
import pytest
import trimesh

from testpaths import REPO
from twin.geometry import Servo, assign_servos, build, read_parts

STEP = REPO / "hardware/cad/Sesame-ESP32-v122.step"
FRAME_STL = REPO / "hardware/printing/stl/Internal-Frame-v121.stl"


@pytest.fixture(scope="module")
def geom(tmp_path_factory):
    return build(STEP, tmp_path_factory.mktemp("geom"))


def test_eight_joints_with_expected_axes(geom):
    j = geom["joints"]
    assert sorted(j) == sorted(["R1", "R2", "L1", "L2", "R3", "R4", "L3", "L4"])
    for name in ("R1", "R2", "L1", "L2"):
        assert abs(j[name]["axis"][2]) > 0.99
    for name in ("R3", "R4", "L3", "L4"):
        assert abs(j[name]["axis"][2]) < 0.05


def test_sides_and_front(geom):
    a = {n: geom["joints"][n]["anchor_mm"] for n in geom["joints"]}
    assert a["R1"][1] < 0 and a["L1"][1] > 0
    assert a["R1"][0] > 0 and a["R2"][0] < 0


def test_hips_form_a_rectangle(geom):
    a = {n: geom["joints"][n]["anchor_mm"] for n in ("R1", "R2", "L1", "L2")}
    assert abs(a["R1"][0] - a["L1"][0]) < 0.5 and abs(a["R1"][1] + a["L1"][1]) < 0.5
    assert abs(a["R1"][0] + a["R2"][0]) < 0.5


def test_chains(geom):
    j = geom["joints"]
    assert (j["R1"]["parent"], j["R1"]["child"]) == ("torso", "femur_R1")
    assert (j["R3"]["parent"], j["R3"]["child"]) == ("femur_R1", "foot_R3")
    assert (j["L4"]["parent"], j["L4"]["child"]) == ("femur_L2", "foot_L4")


def test_frame_matches_printing_stl():
    frame = next(p for p in read_parts(STEP) if p.name.startswith("Internal-Frame"))
    stl = trimesh.load(FRAME_STL)
    assert np.allclose(frame.mesh.bounds, stl.bounds, atol=0.5)


def test_missing_servo_is_named():
    parts = read_parts(STEP)
    servos = [s for s in _servos(parts) if not s.name.endswith("(Mirror) (1):1")]
    with pytest.raises(ValueError, match="L3"):
        assign_servos(servos, parts)


def _servos(parts):
    from twin.geometry import find_servos
    return find_servos(parts)


def test_screws_ride_with_the_part_they_hold(geom):
    # винты крепления сервы лапы едут вместе с лапой, а не с корпусом
    for foot in ("foot_R3", "foot_R4", "foot_L3", "foot_L4"):
        assert sum("Screw" in p for p in geom["links"][foot]["parts"]) >= 2


def test_horns_turn_with_the_femur(geom):
    # качалка сидит на валу и крутится вместе с бедром; у серв лапы качалка прикручена к бедру
    for hip in ("R1", "R2", "L1", "L2"):
        assert sum(p.endswith("::horn") for p in geom["links"][f"femur_{hip}"]["parts"]) >= 1
    assert not any(p.endswith("::horn") for p in geom["links"]["torso"]["parts"])
