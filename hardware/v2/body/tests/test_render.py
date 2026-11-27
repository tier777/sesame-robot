import zipfile

import numpy as np
import trimesh

from body.render import plate_3mf, print_pose


def test_print_pose_deck_down():
    frame = trimesh.creation.box((80, 70, 27))
    frame.apply_translation([0, 0, 23 - 13.5])            # верх на z = 23, как у рамы
    m = print_pose(frame, deck_down=True)
    assert abs(m.bounds[0][2]) < 1e-6 and np.isclose(m.bounds[1][2], 27)


def test_plate_has_two_objects_on_the_bed(tmp_path):
    a = trimesh.creation.box((80, 70, 27))
    b = trimesh.creation.box((79, 35, 1.6))
    path = plate_3mf({"frame_v2": a, "bay_lid_v2": b}, tmp_path / "p.3mf")
    with zipfile.ZipFile(path) as z:
        xml = z.read("3D/3dmodel.model").decode()
    assert xml.count("<object ") == 2 and 'name="frame_v2"' in xml
    for obj in xml.split("<object ")[1:]:
        xs = [float(v.split('"')[0]) for v in obj.split(' x="')[1:]]
        assert 0 < min(xs) and max(xs) < 300
