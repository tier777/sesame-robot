import json
import shutil

import numpy as np
import pytest

from body.sweep import TRACES, commanded_ranges, keep_out, leg_points


def test_ranges_from_traces():
    r = commanded_ranges(TRACES)
    assert r["L2"][1] >= 164 and r["R2"][0] <= 0 and r["R1"] == (min(r["R1"]), max(r["R1"]))


def test_new_pose_widens_range(tmp_path):
    for f in TRACES.glob("*.json"):
        shutil.copy(f, tmp_path)
    (tmp_path / "x.json").write_text(json.dumps({"events": [[0, 0, 60]], "duration_ms": 100}))
    assert commanded_ranges(tmp_path)["R1"][0] == 60


@pytest.fixture(scope="module")
def field():
    return keep_out(step=0.5, margin=1.5)


def test_keep_out_contains_femur_at_stand(field):
    femur, _ = leg_points("R1")
    near = femur[(np.abs(femur[:, 0]) < 44) & (np.abs(femur[:, 1]) < 44) & (femur[:, 2] > -5) & (femur[:, 2] < 34)]
    assert len(near) > 50 and (field(near) < 0).mean() > 0.99


def test_keep_out_free_at_body_center(field):
    assert field(np.array([[0.0, 0.0, 10.0]]))[0] > 0
