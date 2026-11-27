import mujoco
import numpy as np

from twin import params as P
from twin.model import actuator_index, is_floor_pair, load, set_pose


def test_compiles_and_counts():
    m, _ = load()
    assert m.njnt == 9 and m.nu == 8


def test_mass():
    m, _ = load()
    assert 0.23 < m.body_subtreemass[1] < 0.30


def test_ctrl_in_firmware_degrees():
    m, d = load()
    set_pose(m, d, [90] * 8)
    d.ctrl[:] = 90
    mujoco.mj_forward(m, d)
    assert np.allclose(d.actuator_force, 0, atol=1e-6)


def test_actuator_order_is_channel_order():
    assert [actuator_index(j) for j in ("R1", "R2", "L1", "L2", "R4", "R3", "L3", "L4")] == list(range(8))


def test_no_self_contact_at_stand():
    m, d = load()
    set_pose(m, d, P.STAND_FW)
    mujoco.mj_forward(m, d)
    assert d.ncon > 0
    assert all(is_floor_pair(m, c) for c in d.contact[:d.ncon])


def test_meshes_sit_where_the_cad_has_them():
    # в позе сборки сетка лапы R3 лежит там же, где в geometry.json (координаты двойника)
    m, d = load()
    set_pose(m, d, P.STAND_FW, on_floor=False)
    mujoco.mj_forward(m, d)
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot_R3_visual")
    vid = m.geom_dataid[gid]
    adr, n = m.mesh_vertadr[vid], m.mesh_vertnum[vid]
    world = d.geom_xpos[gid] + m.mesh_vert[adr:adr + n] @ d.geom_xmat[gid].reshape(3, 3).T
    import trimesh
    ref = trimesh.load(P.OUT / "meshes/foot_R3.stl").vertices / 1000
    assert np.allclose(world.min(axis=0), ref.min(axis=0), atol=2e-4)
